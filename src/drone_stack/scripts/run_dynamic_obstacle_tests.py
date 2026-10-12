#!/usr/bin/env python3
"""Own the clean restarts for the dynamic-obstacle experiment.

One group == one flight == one fresh stack.  That is deliberate: with
retain_cloud_obstacles the deleted box lingers in the log-odds map for two or
three fresh sweeps, so a second flight in the same process would be flying
against a ghost.  The price is a full readiness wait per group.

Infrastructure failures (F7) abort the whole run -- they mean the harness is
broken, not that the vehicle failed, and continuing would only burn hours.
Behavioural outcomes F1..F6 are legitimate results and the run continues.

  --feasibility    stage 1: a single short group that also asserts the
                   spawn -> perceive -> delete chain.  Run this first.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rosgraph

# Deliberately lean.  The recorder runs on the same four cores as the
# perception pipeline, and on this machine the grid map already reaches
# flight_manager's 2.0 simulation-second freshness deadline during ordinary
# hovering, so every avoided subscriber buys margin for the flight itself.
# The obstacle-perception evidence is kept by the driver's own VoxelWatch
# (which records the voxel coordinates it decoded), not by recording the whole
# /grid_map/voxel_delta bitmap stream here.
TOPICS = ['/gazebo/model_states', '/mavros/state', '/mavros/local_position/odom',
          '/drone/flight_state', '/drone/flight_health',
          '/drone/navigation_stage', '/drone/navigation_wait_reason',
          '/drone/navigation_terminal_stage', '/drone/flight_error',
          '/drone/navigation_goal', '/drone/map_mode', '/clock']

LATERALS = (-1.5, 0.0, 1.5)
ROW_FIELDS = ['group', 'lateral', 'ok', 'code', 'reason', 'min_clearance',
              'max_deviation_from_line', 'deviation_after_box', 'space_wait_samples',
              'perception_hits', 'final_goal_error', 'arrival_state', 'trigger_distance',
              'collision_free', 'goal_completed', 'returned_to_route', 'wall_seconds']


class MemorySampler(threading.Thread):
    """MemAvailable plus the RSS of the simulation processes, at 1 Hz."""

    NAMES = ('gzserver', 'gzclient', 'px4', 'faster_lio', 'ego_planner', 'roslaunch',
             'drone_operator_gui', 'inspection_manager', 'drone_ai')

    def __init__(self, path, append=False):
        super().__init__(daemon=True)
        self.path = Path(path)
        self.append = append
        self.stop = threading.Event()
        self.peak_rss = 0.
        self.min_available = float('inf')

    def run(self):
        with self.path.open('a' if self.append else 'w') as handle:
            while not self.stop.is_set():
                available, rss = self._sample()
                self.min_available = min(self.min_available, available)
                self.peak_rss = max(self.peak_rss, rss)
                handle.write(json.dumps(dict(t=round(time.time(), 2),
                                             available_mb=round(available, 1),
                                             sim_rss_mb=round(rss, 1))) + '\n')
                handle.flush()
                time.sleep(1.)

    def _sample(self):
        available = float('inf')
        try:
            with open('/proc/meminfo') as handle:
                for line in handle:
                    if line.startswith('MemAvailable:'):
                        available = float(line.split()[1]) / 1024.
                        break
        except OSError:
            pass
        rss = 0.
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                name = (entry / 'comm').read_text().strip()
                if not any(key in name for key in self.NAMES):
                    continue
                pages = int((entry / 'statm').read_text().split()[1])
                rss += pages * os.sysconf('SC_PAGE_SIZE') / 1048576.
            except (OSError, ValueError, IndexError):
                continue
        return available, rss


def launcher_endpoint():
    """The master endpoint start_simulation.sh pins its children to.

    The launcher exports ROS_MASTER_URI/ROS_HOSTNAME for its own children, but
    this process -- and the rosbag and driver it spawns -- need the same values.
    A shell configured for a LAN master (ROS_MASTER_URI=http://<host>.local:11311)
    would otherwise poll an address the loopback-bound master never answers, and
    report "ROS master never came up" while the whole stack is running fine.
    """
    return (os.environ.get('DRONE_ROS_MASTER_URI', 'http://127.0.0.1:11311'),
            os.environ.get('DRONE_ROS_HOSTNAME', '127.0.0.1'))


def master_online():
    try:
        return rosgraph.is_master_online()
    except Exception:
        return False


def wait_master(online, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if master_online() == online:
            return True
        time.sleep(0.25)
    return False


def teardown(sim, bag):
    if bag is not None and bag.poll() is None:
        bag.send_signal(signal.SIGINT)
        try:
            bag.wait(timeout=15)
        except subprocess.TimeoutExpired:
            bag.terminate()
            try:
                bag.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bag.kill()
    if sim is not None and sim.poll() is None:
        sim.terminate()
        try:
            sim.wait(timeout=25)
        except subprocess.TimeoutExpired:
            sim.kill()
            sim.wait()


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def run_group(index, directory, args, root, base):
    directory.mkdir(parents=True, exist_ok=False)
    (directory / 'parameters.json').write_text(json.dumps(vars(args), indent=2))
    master_uri, host = launcher_endpoint()
    env = dict(os.environ, DRONE_PREBUILT_MAP=args.prebuilt_map, DRONE_SHOW_GAZEBO='0',
               DRONE_OPTIONAL_MANAGERS='false',
               DRONE_STARTUP_MIN_TIME_S=str(args.startup_min_time),
               DRONE_SIM_OBSTACLE_INFLATION_M=str(args.obstacle_inflation),
               DRONE_LOG_DIR=str(directory / 'stack_logs'),
               DRONE_MAX_MAP_AGE_S=str(args.max_map_age),
               ROS_MASTER_URI=master_uri, ROS_HOSTNAME=host)
    env.pop('ROS_IP', None)
    sim = bag = None
    record = dict(group=directory.name, lateral=None, ok=False)
    try:
        with (directory / 'launcher.log').open('w') as out:
            sim = subprocess.Popen(
                [str(root / 'src/drone_stack/scripts/start_simulation.sh'), '--no-gui'],
                cwd=root, env=env, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
            if not wait_master(True, 90):
                tail = ''
                try:
                    tail = (directory / 'launcher.log').read_text()[-1500:]
                except OSError:
                    pass
                raise RuntimeError('ROS master never came up at %s '
                                   '(launcher exited=%s)\n--- launcher.log ---\n%s'
                                   % (master_uri, sim.poll(), tail))
            with (directory / 'rosbag.log').open('w') as bag_out:
                bag = subprocess.Popen(['rosbag', 'record', '--lz4', '-O',
                                        str(directory / 'telemetry.bag')] + TOPICS,
                                       env=env, stdout=bag_out, stderr=subprocess.STDOUT)
                command = [sys.executable, str(Path(__file__).with_name(
                    'dynamic_obstacle_experiment.py')), '--output', str(directory),
                    '--goal-retries', str(args.goal_retries),
                    '--map-mode', 'PRIOR_NAV' if args.prebuilt_map else 'MAPPING']
                if args.feasibility:
                    command += ['--feasibility', '--forward', '3.2', '--height', str(args.height)]
                else:
                    lateral = LATERALS[(index - 1) % len(LATERALS)]
                    record['lateral'] = lateral
                    command += ['--forward', str(args.forward), '--lateral', str(lateral),
                                '--height', str(args.height),
                                '--obstacle-ahead', str(args.obstacle_ahead),
                                '--obstacle-size'] + [str(v) for v in args.obstacle_size]
                with (directory / 'driver.log').open('w') as driver_log:
                    code = subprocess.call(command, env=env, stdout=driver_log,
                                           stderr=subprocess.STDOUT)
                record['driver_exit'] = code
    except Exception as exc:
        record['error'] = str(exc)
    finally:
        teardown(sim, bag)
        record['master_stopped'] = wait_master(False, 30)
        time.sleep(2)

    group = read_json(directory / 'group.json') or {}
    record['ok'] = bool(group.get('ok'))
    if 'passed' in group:
        record['passed'] = group['passed']
    for field in ROW_FIELDS:
        if field in group:
            record[field] = group[field]
    record.setdefault('code', None if record['ok'] else 'F7_INFRASTRUCTURE')
    record.setdefault('reason', record.get('error'))
    record['wall_seconds'] = group.get('wall_seconds', record.get('wall_seconds'))
    return record, group


def write_report(base, args, records, summary):
    counted = [] if args.feasibility else [r for r in records if r.get('code') != 'F7_INFRASTRUCTURE']
    collision_free = [r for r in counted if r.get('collision_free',
                       r.get('perception_hits', 0) > 0 and r.get('min_clearance') is not None and r['min_clearance'] > 0)]
    completed = [r for r in counted if r.get('goal_completed', r.get('code') == 'SUCCESS')]
    lines = ['# 动态避障实验记录', '',
             '本报告按“新增障碍已被感知且全程无碰撞”判定避障成功；到达导航目标和回归原路径另行统计。',
             '碰撞判据采用真值下全机体含桨保守0.40m球形包络与箱体的最小间距>0。这是无碰撞的保守几何核验。',
             '本项目规划器没有移动障碍运动预测模型，本试验测的是飞行途中出现静止箱体后的反应式避障。', '', '## 汇总', '']
    lines += ['- 有效障碍暴露组数 N = %d' % len(counted),
              '- 无碰撞且有感知证据 M = %d' % len(collision_free),
              '- 避障无碰撞成功率 = %s' % ('%.1f %%' % (100. * len(collision_free) / len(counted)) if counted else '无'),
              '- 到达原目标组数 = %d / %d' % (len(completed), len(counted)),
              '- 规划回归原路径组数 = %d / %d' % (sum(bool(r.get('returned_to_route')) for r in counted), len(counted)),
              '- 基础设施失效（F7，不计入 N）= %d' % sum(r.get('code')=='F7_INFRASTRUCTURE' for r in records),
              '- 峰值仿真进程 RSS = %.0f MB，最低 MemAvailable = %.0f MB' %
              (summary['peak_sim_rss_mb'], summary['min_available_mb']), '',
              '## 逐组结果', '',
              '| 组 | 横向偏移 Δy | 无碰撞 | 导航结果 | 回归原路径 | 最小整机间距/m | 最大三维偏差/m | 回归后偏差/m | 感知体素 | 终点误差/m |',
              '|---|---:|---|---|---|---:|---:|---:|---:|---:|']
    for r in records:
        def num(key, digits=3):
            value = r.get(key)
            return '—' if value is None else ('%.*f' % (digits, value))
        cf = r.get('collision_free', r.get('perception_hits', 0) > 0 and r.get('min_clearance') is not None and r['min_clearance'] > 0)
        lines.append('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' %
                     (r['group'], '—' if r.get('lateral') is None else '%.1f' % r['lateral'],
                      '是' if cf else '否', r.get('code') or '—', '是' if r.get('returned_to_route') else '否', num('min_clearance'),
                      num('max_deviation_from_line'), num('deviation_after_box'),
                      r.get('perception_hits', '—'), num('final_goal_error')))
    lines += ['', '口径：成功要求障碍实际进入规划占据地图并产生运动响应，且整个被记录绕行区间整机包络间距始终>0。该口径只回答“是否安全避开障碍”；终态COMPLETED、目标误差≤0.30m和回归原路线≤0.35m是单独的导航完成指标，不作为碰撞判据。']
    (base / 'REPORT.md').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--groups', type=int, default=12)
    parser.add_argument('--prebuilt-map',default='',help='Use a measured static map with online obstacle updates')
    parser.add_argument('--startup-min-time',type=float,default=65.,help='Minimum simulation startup time; health/map readiness still required')
    parser.add_argument('--obstacle-inflation',type=float,default=.4,help='Horizontal map inflation for this simulation cohort')
    parser.add_argument('--output')
    parser.add_argument('--feasibility', action='store_true')
    parser.add_argument('--forward', type=float, default=5.5)
    parser.add_argument('--height', type=float, default=1.2)
    parser.add_argument('--obstacle-ahead', type=float, default=2.2)
    parser.add_argument('--obstacle-size', type=float, nargs=3, default=[0.30, 2.00, 1.80])
    parser.add_argument('--goal-retries', type=int, default=8)
    parser.add_argument('--max-map-age', type=float, default=4.0,
                        help='DRONE_MAX_MAP_AGE_S for flight_manager; the shipped default '
                             'is 2.0, which a quarter-speed simulator crosses on its own')
    parser.add_argument('--resume', action='store_true',
                        help='continue an existing --output directory instead of refusing')
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[3]
    # Match the launcher's endpoint before anything reads ROS_MASTER_URI.
    master_uri, host = launcher_endpoint()
    os.environ['ROS_MASTER_URI'] = master_uri
    os.environ['ROS_HOSTNAME'] = host
    os.environ.pop('ROS_IP', None)
    print('ROS master endpoint: %s (hostname %s)' % (master_uri, host), flush=True)
    if args.output:
        base = Path(args.output).resolve()
        if base.exists() and any(base.iterdir()) and not args.resume:
            raise SystemExit('%s is not empty; pass --resume to append' % base)
    else:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = root / 'reports' / ('dynamic_obstacle_%s' % stamp)
    base.mkdir(parents=True, exist_ok=True)
    if master_online():
        raise SystemExit('Stop the existing simulation launcher before running this.')

    groups = 1 if args.feasibility else args.groups
    previous_metadata = read_json(base / 'metadata.json')
    if previous_metadata:
        with (base / 'metadata_history.jsonl').open('a') as handle:
            handle.write(json.dumps(previous_metadata) + '\n')
    (base / 'metadata.json').write_text(json.dumps(dict(
        started=datetime.now().isoformat(timespec='seconds'),
        feasibility=args.feasibility, groups=groups, forward=args.forward,
        height=args.height, obstacle_ahead=args.obstacle_ahead,
        obstacle_size=args.obstacle_size, laterals=list(LATERALS),
        goal_retries=args.goal_retries,
        max_map_age_s=args.max_map_age,
        startup_min_time_s=args.startup_min_time,
        horizontal_obstacle_inflation_m=args.obstacle_inflation,
        prebuilt_map=args.prebuilt_map,
        launch='start_simulation.sh --no-gui, measured prebuilt map' if args.prebuilt_map else 'start_simulation.sh --no-gui, online mapping',
        slow_simulator_note=('The simulated clock runs at roughly 0.25x real time on this '
                             'machine. What ages a grid map is wall-clock pipeline delay, '
                             'but flight_manager compares that age against max_map_age_s in '
                             'simulation seconds, so the same Faster-LIO/voxel-carving lag '
                             'has to be compared in the same clock domain. The 2.0 s shipped '
                             'default is crossed during ordinary flight and navigation is '
                             'cancelled before the obstacle exists. DRONE_MAX_MAP_AGE_S is '
                             'therefore raised to %g simulation seconds for this experiment '
                             '(the shipped default is unchanged), the driver records the age '
                             'on every sample and the per-delta peak in group.json, and any '
                             'navigation the deadline does cancel is re-issued by the driver '
                             'as a bounded, recorded goal_resend event.'
                             % args.max_map_age),
        criteria=dict(trigger='box voxel in /grid_map/voxel_delta + motion response',
                      collision='truth conservative 0.40 m whole-airframe envelope clearance > 0',
                      arrival='terminal COMPLETED and truth endpoint error <= 0.30 m',
                      route_return='lateral deviation <= 0.35 m after passing the box'),
    ), indent=2))

    sampler = MemorySampler(base / 'memory.jsonl', append=args.resume)
    sampler.start()

    previous=read_json(base/'summary.json') if args.resume else None
    records = list(previous.get('groups',[])) if previous else []
    if args.resume:
        # A landed group can be archived before an interrupted orchestrator
        # writes its summary. Recover only contiguous, fully finalized groups.
        for index in range(len(records) + 1, groups + 1):
            directory = base / 'groups' / ('group_%02d' % index)
            recovered = read_json(directory / 'group.json')
            if not recovered or not any(e.get('event') == 'landed'
                                        for e in recovered.get('events', [])):
                break
            record = dict(group=directory.name,
                          lateral=LATERALS[(index - 1) % len(LATERALS)],
                          recovered_from_completed_archive=True)
            record.update({key: recovered[key] for key in ROW_FIELDS if key in recovered})
            record['collision_free'] = bool(recovered.get('perception_hits',0)>0 and recovered.get('min_clearance') is not None and recovered['min_clearance']>0)
            record['goal_completed'] = recovered.get('code') == 'SUCCESS'
            records.append(record)
    previous_stats = previous.get('summary', {}) if previous else {}
    sampler.peak_rss = max(sampler.peak_rss, previous_stats.get('peak_sim_rss_mb', 0.))
    sampler.min_available = min(sampler.min_available, previous_stats.get('min_available_mb', float('inf')))
    groups_data = []
    for index in range(len(records)+1, groups + 1):
        name = 'feasibility' if args.feasibility else 'group_%02d' % index
        directory = base / 'groups' / name
        print('=== %s ===' % name, flush=True)
        record, group = run_group(index, directory, args, root, base)
        record['collision_free'] = bool(record.get('perception_hits',0)>0 and record.get('min_clearance') is not None and record['min_clearance']>0 and record.get('code')!='F7_INFRASTRUCTURE')
        record['goal_completed'] = record.get('code') == 'SUCCESS'
        records.append(record)
        if group:
            groups_data.append(group)
        print(json.dumps({k: record.get(k) for k in
                          ('group', 'ok', 'code', 'reason', 'min_clearance',
                           'wall_seconds')}, ensure_ascii=False), flush=True)
        (base / 'summary.json').write_text(json.dumps(
            dict(groups=records, finished=index, planned=groups), indent=2))
        if record.get('code') == 'F7_INFRASTRUCTURE':
            print('infrastructure failure; stopping the run', flush=True)
            break

    sampler.stop.set()
    sampler.join(timeout=3.)

    counted = [] if args.feasibility else [r for r in records if r.get('code') != 'F7_INFRASTRUCTURE']
    success = [r for r in counted if r.get('collision_free')]
    goal_success = [r for r in counted if r.get('goal_completed')]
    summary = dict(records=len(records), counted=len(counted), success=len(success),
                   rate=None if not counted else round(100. * len(success) / len(counted), 2),
                   collision_free_success=len(success), collision_free_rate=None if not counted else round(100.*len(success)/len(counted),2),
                   goal_completed=len(goal_success), goal_completion_rate=None if not counted else round(100.*len(goal_success)/len(counted),2),
                   infrastructure=sum(r.get('code')=='F7_INFRASTRUCTURE' for r in records),
                   peak_sim_rss_mb=round(sampler.peak_rss, 1),
                   min_available_mb=round(sampler.min_available, 1),
                   finished=datetime.now().isoformat(timespec='seconds'))
    (base / 'summary.json').write_text(json.dumps(dict(summary=summary, groups=records), indent=2))

    with (base / 'results.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=ROW_FIELDS)
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key) for key in ROW_FIELDS})

    write_report(base, args, records, summary)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if counted:
        print('共%d组：无碰撞避障%d组（%.1f%%），到达原目标%d组。'
              % (len(counted), len(success), 100.*len(success)/len(counted), len(goal_success)), flush=True)
    sys.exit(0 if args.feasibility and records[0].get('ok') else
             (0 if counted and len(records) == groups else 1))


if __name__ == '__main__':
    main()
