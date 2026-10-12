#!/usr/bin/env python3
"""Read-only Gazebo evaluation. Ground truth is never sent to the controller."""
import argparse
import bisect
import csv
import json
import math
import signal
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from tf.transformations import quaternion_matrix, quaternion_slerp

ACTIVE = {'ARMING', 'TAKEOFF', 'TRANSITING', 'INSPECTING', 'RETURNING', 'LANDING'}


def pose_values(p):
    return [p.position.x, p.position.y, p.position.z], [
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


class Collector:
    def __init__(self, map_path, output, model):
        self.output = Path(output)
        (self.output / 'csv').mkdir(parents=True, exist_ok=True)
        self.task = json.loads(Path(str(map_path) + '.inspection.json').read_text())
        self.model = model
        self.latest = {}
        self.truth = deque(maxlen=3000)
        self.lock = threading.RLock()
        self.events = deque()
        self.stop = False
        self.started = False
        self.success = False
        self.alignment = None
        self.last_status_key = None
        self.metadata = dict(map_path=str(Path(map_path).resolve()),
            map_digest=self.task['map_digest'], sample_sim_hz=5.,
            world_from_map_translation=[0., 0., .17],
            world_from_map_yaw=0., lio_alignment='fixed SE(3) from first synchronized ground sample',
            ground_truth_topic='/gazebo/model_states',
            ground_truth_timestamp='ROS clock at callback; acquisition timestamp unavailable in ModelStates',
            coverage='actual down-camera geometric floor visibility; separately nominal configured swath',
            strip_spacing_m=self.task['topology'].get('strip_spacing_m', 1.8),
            hall_height_above_floor_m=self.task['topology'].get('hall_height_above_floor_m',1.6),
            hall_inset_m=self.task['topology'].get('hall_inset_m',.8),
            simulation_aggressive=rospy.get_param('/drone/sim_aggressive',False),
            obstacles_inflation_m=rospy.get_param('/ego_planner_node/grid_map/obstacles_inflation',.4),
            health_hold_s=rospy.get_param('/drone/health_hold_s',1.),
            health_land_s=rospy.get_param('/drone/health_land_s',2.),
            progress_timeout_s=rospy.get_param('/drone/building/progress_timeout_s',25.),
            corridor_height_above_floor_m=self.task['topology'].get('corridor_height_above_floor_m',1.2),
            camera_pose_body=[0., 0., -.15, 0., math.pi / 2, 0.],
            camera_horizontal_fov_rad=1.3962634, camera_width=640, camera_height=480,
            simulation_lio_min_support=rospy.get_param('/mapping/min_translation_support'),
            created_wall=time.time())
        (self.output / 'task.json').write_text(json.dumps(self.task, ensure_ascii=False, indent=2))
        self.fields = ['sim_time_s', 'wall_time_s', 'mission_id', 'building_state', 'segment',
            'segment_kind', 'phase', 'health', 'armed', 'truth_valid', 'lio_pair_valid',
            'lio_stamp_s', 'truth_pair_gap_s', 'gt_x_m', 'gt_y_m', 'gt_z_m',
            'gt_qx', 'gt_qy', 'gt_qz', 'gt_qw', 'fcu_map_x_m', 'fcu_map_y_m', 'fcu_map_z_m',
            'lio_x_m', 'lio_y_m', 'lio_z_m', 'lio_world_x_m', 'lio_world_y_m', 'lio_world_z_m',
            'lio_ate_m', 'lio_ate_xy_m', 'lio_ate_z_m', 'lio_attitude_error_deg',
            'target_map_x_m', 'target_map_y_m', 'target_map_z_m',
            'true_target_error_m', 'estimated_target_error_m', 'speed_cap_mps',
            'camera_stamp_s', 'camera_age_s', 'min_translation_support', 'effective_points']
        self.file = (self.output / 'csv' / 'samples.csv').open('w', encoding='utf-8-sig', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields)
        self.writer.writeheader()
        self.event_file = (self.output / 'csv' / 'events.csv').open('w', encoding='utf-8-sig', newline='')
        self.event_writer = csv.DictWriter(self.event_file,
            fieldnames=['sim_time_s', 'wall_time_s', 'mission_id', 'state', 'segment', 'task_id', 'message'])
        self.event_writer.writeheader()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_truth, queue_size=1)
        for topic, cls, key in [
            ('/drone/lio/odom', Odometry, 'lio'),
            ('/drone/fcu/odom', Odometry, 'fcu'),
            ('/drone/map_transform', PoseStamped, 'transform'),
            ('/drone/down/image_raw', Image, 'camera'),
            ('/mavros/state', State, 'state'),
            ('/mavros/extended_state', ExtendedState, 'extended'),
            ('/faster_lio/translation_observability', Float64MultiArray, 'geometry')]:
            rospy.Subscriber(topic, cls, lambda m, k=key: self.receive(k, m), queue_size=1)
        for key in ('flight_state', 'flight_health'):
            rospy.Subscriber('/drone/' + key, String,
                lambda m, k=key: self.receive(k, m.data), queue_size=1)
        rospy.Subscriber('/drone/building/status', String, self.on_status, queue_size=10)

    def receive(self, key, value):
        with self.lock:
            self.latest[key] = value

    def on_truth(self, m):
        try:
            index = m.name.index(self.model)
        except ValueError:
            return
        stamp = rospy.Time.now().to_sec()
        p, q = pose_values(m.pose[index])
        with self.lock:
            if self.truth and stamp < self.truth[-1][0]:
                self.truth.clear()
            if not self.truth or stamp > self.truth[-1][0]:
                self.truth.append((stamp, np.array(p), np.array(q)))

    def on_status(self, m):
        try:
            status = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.latest['building'] = status
            key = (status.get('mission_id'), status.get('state'), status.get('segment'), status.get('task_id'))
            if key != self.last_status_key:
                self.last_status_key = key
                self.events.append(dict(sim_time_s=status.get('stamp_ns', 0) / 1e9,
                    wall_time_s=time.time(), mission_id=status.get('mission_id', ''),
                    state=status.get('state', ''), segment=status.get('segment', -1),
                    task_id=status.get('task_id', 0), message=status.get('message', '')))

    @staticmethod
    def interpolate(truth, stamp):
        if not truth:
            return None
        stamps = [v[0] for v in truth]
        i = bisect.bisect_left(stamps, stamp)
        if 0 < i < len(truth):
            a, b = truth[i - 1], truth[i]
            gap = b[0] - a[0]
            if gap > .15:
                return None
            u = (stamp - a[0]) / gap
            return (a[1] * (1 - u) + b[1] * u,
                    quaternion_slerp(a[2], b[2], u), max(stamp - a[0], b[0] - stamp))
        nearest = truth[min(i, len(truth) - 1)]
        gap = abs(stamp - nearest[0])
        return (nearest[1], nearest[2], gap) if gap <= .12 else None

    def sample(self, now):
        with self.lock:
            d = dict(self.latest)
            truth = list(self.truth)
            events = list(self.events)
            self.events.clear()
        for event in events:
            self.event_writer.writerow(event)
        self.event_file.flush()
        b = d.get('building', {})
        state = b.get('state', '')
        if state in ACTIVE:
            self.started = True
        if self.started and state == 'COMPLETED' and d.get('state') and not d['state'].armed and d.get('extended') and d['extended'].landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
            self.success = True
            self.stop = True
        row = dict(sim_time_s=now, wall_time_s=time.time(), mission_id=b.get('mission_id', ''),
            building_state=state, segment=b.get('segment', -1),
            phase=d.get('flight_state', ''), health=d.get('flight_health', ''),
            armed=bool(d.get('state') and d['state'].armed),
            truth_valid=False, lio_pair_valid=False)
        current = self.interpolate(truth, now)
        if current:
            p, q, _ = current
            row.update(dict(zip(['gt_x_m', 'gt_y_m', 'gt_z_m'], p)))
            row.update(dict(zip(['gt_qx', 'gt_qy', 'gt_qz', 'gt_qw'], q)))
            row['truth_valid'] = True
        if 'fcu' in d and 'transform' in d:
            p, _ = pose_values(d['fcu'].pose.pose)
            t, q = pose_values(d['transform'].pose)
            pm = quaternion_matrix(q)[:3, :3] @ p + t
            row.update(dict(zip(['fcu_map_x_m', 'fcu_map_y_m', 'fcu_map_z_m'], pm)))
        if 'lio' in d:
            m = d['lio']
            p, q = pose_values(m.pose.pose)
            stamp = m.header.stamp.to_sec()
            pair = self.interpolate(truth, stamp)
            row.update(dict(zip(['lio_x_m', 'lio_y_m', 'lio_z_m'], p)))
            row['lio_stamp_s'] = stamp
            if pair and -.02 <= now - stamp <= .6:
                pg, qg, gap = pair
                if self.alignment is None and not row['armed']:
                    rotation = quaternion_matrix(qg)[:3, :3] @ quaternion_matrix(q)[:3, :3].T
                    translation = pg - rotation @ p
                    self.alignment = rotation, translation
                    self.metadata['lio_world_rotation'] = rotation.tolist()
                    self.metadata['lio_world_translation'] = translation.tolist()
                if self.alignment is not None:
                    rotation, translation = self.alignment
                    world = rotation @ p + translation
                    error = world - pg
                    relative = (rotation @ quaternion_matrix(q)[:3, :3]).T @ quaternion_matrix(qg)[:3, :3]
                    row.update(dict(zip(['lio_world_x_m', 'lio_world_y_m', 'lio_world_z_m'], world)))
                    row.update(lio_pair_valid=True, truth_pair_gap_s=gap,
                        lio_ate_m=float(np.linalg.norm(error)), lio_ate_xy_m=float(np.linalg.norm(error[:2])),
                        lio_ate_z_m=float(error[2]),
                        lio_attitude_error_deg=math.degrees(math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1)))))
        index = b.get('segment', -1)
        if 0 <= index < len(self.task['segments']):
            segment = self.task['segments'][index]
            target = np.array(segment['points'][-1])
            row.update(segment_kind=segment['kind'], speed_cap_mps=segment['speed'])
            row.update(dict(zip(['target_map_x_m', 'target_map_y_m', 'target_map_z_m'], target)))
            if current:
                row['true_target_error_m'] = float(np.linalg.norm(current[0] - target - [0., 0., .17]))
            if 'fcu_map_x_m' in row:
                row['estimated_target_error_m'] = float(np.linalg.norm(pm - target))
        if 'camera' in d:
            stamp = d['camera'].header.stamp.to_sec()
            row.update(camera_stamp_s=stamp, camera_age_s=now-stamp)
        if 'geometry' in d and len(d['geometry'].data) == 6:
            row.update(min_translation_support=d['geometry'].data[1], effective_points=d['geometry'].data[4])
        self.writer.writerow(row)
        self.file.flush()
        return row

    def run(self):
        last = -1.
        while not self.stop and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now > 0 and now - last >= .2:
                self.sample(now)
                last = now
            time.sleep(.01)
        # Flush the actual terminal state, even if shutdown was requested externally.
        self.sample(rospy.Time.now().to_sec())
        self.file.close()
        self.event_file.close()
        self.metadata.update(mission_completed=self.success, finished_wall=time.time(),final_phase=self.latest.get('flight_state'),final_health=self.latest.get('flight_health'),final_armed=bool(self.latest.get('state') and self.latest['state'].armed),final_landed_state=self.latest['extended'].landed_state if self.latest.get('extended') else None)
        (self.output / 'metadata.json').write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--model', default='inspection_quad')
    parser.add_argument('--world', required=True)
    args = parser.parse_args()
    rospy.init_node('patrol_metrics_recorder', disable_signals=True)
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Ground-truth metrics are simulation only')
    collector = Collector(args.map, args.output, args.model)
    collector.metadata['world_file'] = str(Path(args.world).resolve())
    signal.signal(signal.SIGINT, lambda *_: setattr(collector, 'stop', True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(collector, 'stop', True))
    collector.run()
    from patrol_report import export_report
    export_report(collector.output)


if __name__ == '__main__':
    main()
