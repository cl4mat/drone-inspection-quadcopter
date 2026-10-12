#!/usr/bin/env python3
"""One flight of the dynamic-obstacle experiment: fly, inject, decide, land.

Run by run_dynamic_obstacle_tests.py against an already-live stack, or by hand
for a single group.  What it measures is the *reactive* behaviour: a static box
appears on the route after the vehicle is already under way, and the run counts
as a success only when the vehicle saw it, did not touch it, still reached the
original goal, and was back on the original route line before the final approach.

Nothing here feeds Gazebo truth into the planner; truth is read on the side, for
clearance and for the endpoint check only.

  --feasibility   stage-1 smoke run: short goal, then spawn -> perceive -> delete
                  assertions on a box parked away from the route.  No outcome
                  codes, just pass/fail on the injection chain.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from drone_stack.srv import LocalGoal, Takeoff, MapSession, MapSessionRequest
from dynamic_obstacle_injector import Injector, MapTransform, VoxelWatch, aabb_clearance, BODY_RADIUS

DRONE = 'inspection_quad'
BOX = 'dynamic_obstacle'
READY_TIMEOUT = 1200.
SETTLE = 1.0

# This stack runs Gazebo in lockstep with PX4 well below real time -- measured at
# roughly 0.25x on this machine, so a 1 Hz *simulation* topic such as
# /mavros/state arrives only every ~4 *wall* seconds.  Everything below is in
# wall time, so the liveness windows and the phase timeouts carry a wide margin;
# a 3 s window against a 4 s arrival period is what wedged the readiness gate.
LIVENESS = 15.
PHASE_SCALE = 3.0

# flight_manager and grid_map publish these with latch=True and only on change,
# so a reading that stops arriving means "unchanged", not "stale".  Treating them
# as stale makes the readiness gate wait forever on a steady READY state.
LATCHED = {'flight_state', 'flight_health', 'flight_error', 'navigation_stage',
           'wait_reason', 'terminal_stage', 'goal', 'map_mode', 'map_alignment_ready'}

TOPICS = {
    'clock': ('/clock', Clock),
    'state': ('/mavros/state', State),
    'odom': ('/mavros/local_position/odom', Odometry),
    'models': ('/gazebo/model_states', ModelStates),
    'flight_state': ('/drone/flight_state', String),
    'flight_health': ('/drone/flight_health', String),
    'navigation_stage': ('/drone/navigation_stage', String),
    'wait_reason': ('/drone/navigation_wait_reason', String),
    'terminal_stage': ('/drone/navigation_terminal_stage', String),
    'flight_error': ('/drone/flight_error', String),
    'goal': ('/drone/navigation_goal', PoseStamped),
    'map_mode': ('/drone/map_mode', String),
    'map_alignment_ready': ('/drone/map_alignment_ready', Bool),
}


class Timeout(RuntimeError):
    pass


class Driver:
    def __init__(self, args):
        self.args = args
        self.output = os.path.abspath(args.output)
        os.makedirs(self.output, exist_ok=True)
        self.data = {}
        self.lock = threading.Lock()
        self.subscribers = [rospy.Subscriber(topic, kind, self._store(name),
                                             queue_size=20 if name in ('clock', 'odom') else 5)
                            for name, (topic, kind) in TOPICS.items()]
        self.injector = Injector()
        self.transform = MapTransform()
        self.voxels = VoxelWatch()
        self.events = []
        self.group = {}
        self.goal_odom = None
        self.arrival_sim = None
        self._samples = None
        self._truth = None
        self._stop = threading.Event()
        self._thread = None
        self._closed = False

    # ---- plumbing ---------------------------------------------------------
    def _store(self, name):
        def callback(message):
            with self.lock:
                self.data[name] = (message, time.monotonic())
        return callback

    def value(self, name, seconds=LIVENESS):
        with self.lock:
            entry = self.data.get(name)
        if entry is None:
            return None
        message, stamp = entry
        if name in LATCHED:
            return message
        if seconds is not None and time.monotonic() - stamp > seconds:
            return None
        return message

    def text(self, name, seconds=LIVENESS):
        message = self.value(name, seconds)
        return None if message is None else message.data

    def flag(self, name, seconds=LIVENESS):
        message = self.value(name, seconds)
        return None if message is None else bool(message.data)

    def sim_now(self):
        message = self.value('clock', 2 * LIVENESS)
        return None if message is None else message.clock.to_sec()

    def event(self, name, **detail):
        entry = dict(wall=round(time.time(), 3), sim=self.sim_now(), event=name)
        entry.update(detail)
        self.events.append(entry)
        rospy.loginfo('event %s %s', name, json.dumps(detail, sort_keys=True))
        return entry

    def wait(self, predicate, seconds, what):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.05)
        raise Timeout('%s (waited %.0f s)' % (what, seconds))

    def stable(self, predicate, seconds, hold, describe=None):
        """True once `predicate` has held continuously for `hold` seconds.

        `describe` is called every 15 s while the predicate is failing, so a gate
        that never opens says which condition is still unmet instead of sitting
        silent until the timeout.
        """
        deadline = time.monotonic() + seconds
        since = None
        last = 0.
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                since = since or time.monotonic()
                if time.monotonic() - since >= hold:
                    return True
            else:
                since = None
                if describe is not None and time.monotonic() - last > 15.:
                    last = time.monotonic()
                    rospy.loginfo('not ready yet: %s', json.dumps(describe(), sort_keys=True))
            time.sleep(0.1)
        return False

    def call(self, name, kind, *args, **kwargs):
        rospy.wait_for_service(name, timeout=20.)
        try:
            reply = rospy.ServiceProxy(name, kind)(*args, **kwargs)
        except rospy.ServiceException as exc:
            raise RuntimeError('%s failed: %s' % (name, exc))
        if not reply.success:
            raise RuntimeError('%s refused: %s' % (name, reply.message))
        rospy.loginfo('%s ok: %s', name, reply.message)
        return reply

    # ---- truth ------------------------------------------------------------
    def drone_world(self):
        return self.injector.model(DRONE)

    def drone_odom(self):
        message = self.value('odom')
        if message is None:
            return None
        p = message.pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def box_world(self):
        return self.injector.model(BOX)

    def watch_obstacle(self,centre):
        # Existing inflated floor cells do not prove the newly spawned box
        # was perceived. Observe only the volume above the floor's inflation.
        size=np.asarray(self.args.obstacle_size,float).copy()
        base=min(.6,size[2]/2)
        watched=np.asarray(centre,float)+np.array([0.,0.,base/2])
        size[2]-=base
        self.voxels.watch(BOX,self.transform.world_to_odom(watched),size)

    def map_age(self):
        """Simulation seconds since the newest grid-map delta, as flight_manager sees it.

        flight_manager cancels navigation outright while this exceeds
        max_map_age_s.  On this machine the simulated clock runs at roughly a
        quarter of real time, which stretches the perception pipeline's latency
        in simulated seconds, so this number is worth recording alongside every
        sample rather than assumed small.  Every sample carries it; the exact
        per-delta peak is accumulated by VoxelWatch and lands in group.json.
        """
        stamp = self.voxels.current_stamp()
        now = self.sim_now()
        if stamp is None or now is None:
            return None
        return round(now - stamp, 3)

    def clearance(self):
        box = self.box_world()
        drone = self.drone_world()
        if box is None or drone is None:
            return None
        return aabb_clearance(drone[0], box[0], np.asarray(self.args.obstacle_size) / 2.)

    # ---- sampling ---------------------------------------------------------
    def start_sampling(self):
        self._samples = open(os.path.join(self.output, 'samples.jsonl'), 'w')
        self._truth = open(os.path.join(self.output, 'ground_truth.jsonl'), 'w')
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def _sample_loop(self):
        tick = 0
        while not self._stop.is_set():
            try:
                self._sample(tick)
            except Exception as exc:                       # never kill the flight
                rospy.logwarn_throttle(5., 'sampler: %s', exc)
            tick += 1
            # 10 Hz wall is ~20 Hz simulation here.  The sampler competes with
            # the perception pipeline for the same four cores, and the pipeline
            # is already the reason the map goes stale, so it stays cheap.
            time.sleep(0.1)

    def _sample(self, tick):
        now = time.monotonic()
        odom = self.drone_odom()
        world = self.drone_world()
        box = self.box_world()
        row = dict(t=round(now, 3), sim=self.sim_now(),
                   flight_state=self.text('flight_state'),
                   navigation_stage=self.text('navigation_stage'),
                   wait_reason=self.text('wait_reason'),
                   terminal_stage=self.text('terminal_stage'),
                   flight_error=self.text('flight_error'),
                   odom=None if odom is None else [round(v, 4) for v in odom],
                   world=None if world is None else [round(v, 4) for v in world[0]],
                   box=None if box is None else [round(v, 4) for v in box[0]],
                   clearance=self.clearance(), map_age=self.map_age())
        self._truth.write(json.dumps(row) + '\n')
        if tick % 3 == 0:
            self._samples.write(json.dumps(row) + '\n')
        if tick % 25 == 0:
            self._truth.flush()
            self._samples.flush()

    def stop_sampling(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.)
        for handle in (self._samples, self._truth):
            if handle:
                handle.flush()
                handle.close()
        self._samples = self._truth = None

    # ---- flight -----------------------------------------------------------
    def ready(self):
        if getattr(self.args, 'reuse_airborne', False):
            self.wait(lambda: self.value('state') is not None and self.value('state').armed
                      and self.text('flight_state') == 'HOLD'
                      and self.text('flight_health') == 'HEALTHY'
                      and self.flag('map_alignment_ready') is True,
                      120., 'shared flight not healthy in HOLD')
            self.event('ready_airborne')
            return
        if getattr(self.args,'map_mode','MAPPING')=='PRIOR_NAV' and not rospy.has_param('/drone/simulation_spawn_request'):
            # Equivalent to the UI's explicit "start from map origin" choice.
            rospy.wait_for_service('/drone/map_session',timeout=60.)
            request=MapSessionRequest(action='origin',path=rospy.get_param('/drone/prebuilt_map_path'))
            proxy=rospy.ServiceProxy('/drone/map_session',MapSession)
            deadline=time.monotonic()+90.
            while time.monotonic()<deadline:
                reply=proxy(request)
                if reply.success:
                    self.event('initial_pose_origin_confirmed');break
                time.sleep(1.)
            else:raise Timeout('could not confirm initial map-origin pose')
        def predicate():
            if self.sim_now() is None:
                return False
            state = self.value('state')
            if state is None or not state.connected or state.armed:
                return False
            if self.drone_world() is None:
                return False
            return (self.text('flight_state') == 'READY'
                    and self.text('flight_health') == 'HEALTHY'
                    and self.flag('map_alignment_ready') is True
                    and self.text('map_mode') == getattr(self.args,'map_mode','MAPPING'))

        def pending():
            state = self.value('state')
            return dict(sim=self.sim_now(),
                        clock_ok=self.sim_now() is not None,
                        connected=None if state is None else bool(state.connected),
                        armed=None if state is None else bool(state.armed),
                        gazebo_model=self.drone_world() is not None,
                        flight_state=self.text('flight_state'),
                        flight_health=self.text('flight_health'),
                        map_alignment_ready=self.flag('map_alignment_ready'),
                        map_mode=self.text('map_mode'))

        if not self.stable(predicate, self.args.ready_timeout, SETTLE, describe=pending):
            raise Timeout('stack not READY within %.0f s: %s'
                          % (self.args.ready_timeout, json.dumps(pending(), sort_keys=True)))
        self.event('ready')

    def arm_and_takeoff(self):
        self.call('/drone/set_authorized', SetBool, True)
        self.call('/drone/arm', Trigger)

        def armed():
            # takeoff() demands phase ARMED, and the phase topic can trail the
            # mavros armed flag; wait for both so the call is never refused.
            state = self.value('state')
            return (state is not None and state.armed
                    and self.text('flight_state') == 'ARMED')

        self.wait(armed, PHASE_SCALE * 60., 'vehicle did not arm')
        self.call('/drone/takeoff', Takeoff, self.args.height)
        self.wait(lambda: self.text('flight_state') == 'HOLD', PHASE_SCALE * 60.,
                  'takeoff did not settle in HOLD')
        self.wait(lambda: self.drone_odom() is not None, 10., 'no odom after takeoff')

    def send_goal(self, offset):
        start_odom = self.drone_odom()
        if start_odom is None:
            raise Timeout('no odom at goal time')
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = float(start_odom[0] + offset[0])
        goal.pose.position.y = float(start_odom[1] + offset[1])
        goal.pose.position.z = float(start_odom[2] + offset[2])
        goal.pose.orientation.w = 1.0
        self.call('/drone/local_goal', LocalGoal, goal)
        self.goal_odom = np.array([goal.pose.position.x, goal.pose.position.y,
                                   goal.pose.position.z])
        return start_odom, goal

    def geometry(self, start_odom):
        """Route line in world frame plus the along/perp basis, from truth."""
        start_world = self.transform.odom_to_world(start_odom)
        goal_world = self.transform.odom_to_world(self.goal_odom)
        span = goal_world - start_world
        horizontal = np.array([span[0], span[1], 0.])
        span_length = float(np.linalg.norm(horizontal))
        if span_length < 1e-6:
            raise Timeout('degenerate route: goal on top of the start point')
        along = horizontal / span_length
        perp = np.array([-along[1], along[0], 0.])
        return start_world, goal_world, along, perp, span_length

    def resend_goal(self):
        """Re-issue the stored absolute goal after an infrastructure cancellation.

        flight_manager cancels navigation when its map-freshness deadline
        lapses.  On this machine the simulated clock runs at about a quarter of
        real time, which stretches the perception chain's latency in simulated
        seconds until that deadline is occasionally crossed.  The cancellation
        is unrelated to the experiment: the obstacle has not even been spawned
        yet.  An operator would simply re-issue the goal, so the harness does
        the same and records every occurrence.
        """
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'odom'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = [
            float(v) for v in self.goal_odom]
        goal.pose.orientation.w = 1.0
        # local_goal refuses while the map is stale, which is precisely the
        # condition that cancelled the previous attempt, so wait for the
        # perception chain to deliver a fresh delta before asking again.
        deadline = time.monotonic() + PHASE_SCALE * 30.
        while time.monotonic() < deadline and not rospy.is_shutdown():
            age = self.map_age()
            if age is not None and age <= 1.5:
                break
            time.sleep(0.1)
        try:
            self.call('/drone/local_goal', LocalGoal, goal)
        except RuntimeError as exc:
            # A refusal is not an infrastructure failure: the pre-injection
            # phase simply tries again on the next pass through the loop.
            rospy.logwarn('goal resend refused: %s', exc)
            self.event('goal_resend_refused', reason=str(exc))
            return False
        self.event('goal_resend', goal=[round(float(v), 3) for v in self.goal_odom],
                   flight_error=self.text('flight_error'))
        return True

    def await_terminal(self):
        """Wait for terminal COMPLETED, re-issuing the goal if HOLD pre-empts it.

        Same reasoning as resend_goal: a map-freshness cancellation is a
        property of this slow simulator, not a result of the experiment, and it
        is recorded as an event each time it happens.
        """
        deadline = time.monotonic() + PHASE_SCALE * 180.
        resends = 0
        cancelled_at = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.text('terminal_stage') == 'COMPLETED':
                return
            if self.text('flight_state') == 'HOLD':
                if cancelled_at is None:
                    cancelled_at = time.monotonic()
                elif time.monotonic() - cancelled_at > 3.:
                    cancelled_at = None
                    if resends >= self.args.goal_retries:
                        raise Timeout('goal never completed (%d resends): %s'
                                      % (resends, self.text('flight_error')))
                    rospy.logwarn('goal cancelled before completion; re-issuing (%d/%d)',
                                  resends + 1, self.args.goal_retries)
                    if self.resend_goal():
                        resends += 1
            else:
                cancelled_at = None
            time.sleep(0.1)
        raise Timeout('goal not completed')

    # ---- injection --------------------------------------------------------
    def inject(self, start_world, goal_world):
        """Spawn the box `obstacle_ahead` in front of the vehicle, toward the goal."""
        ahead = self.args.obstacle_ahead
        deadline = time.monotonic() + self.args.inject_timeout
        centre = None
        resends = 0
        cancelled_at = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            odom = self.drone_odom()
            world = self.drone_world()
            if odom is None or world is None:
                time.sleep(0.05)
                continue
            flying = (self.text('flight_state') == 'NAVIGATING'
                      and self.text('navigation_stage') == 'TRACKING')
            travelled = float(np.linalg.norm(world[0] - start_world))
            if flying and travelled >= 0.3:
                direction = goal_world - world[0]  # Placement is in Gazebo world.
                direction[2] = 0.
                norm = float(np.linalg.norm(direction))
                if norm < 1e-3:
                    time.sleep(0.05)
                    continue
                centre = world[0] + direction / norm * ahead
                # Keep the whole box inside the lower hall (x -8..8, y -6..6).
                centre[0] = float(np.clip(centre[0], -7.4, 7.4))
                centre[1] = float(np.clip(centre[1], -5.4, 5.4))
                centre[2] = float(self.args.obstacle_size[2]) / 2.
                break
            if self.text('flight_state') == 'HOLD':
                if cancelled_at is None:
                    cancelled_at = time.monotonic()
                elif time.monotonic() - cancelled_at > 3.:
                    cancelled_at = None
                    if resends >= self.args.goal_retries:
                        raise Timeout('navigation kept being cancelled (%d resends): %s'
                                      % (resends, self.text('flight_error')))
                    rospy.logwarn('navigation fell back to HOLD; re-issuing the goal (%d/%d)',
                                  resends + 1, self.args.goal_retries)
                    if self.resend_goal():
                        resends += 1
            else:
                cancelled_at = None
            time.sleep(0.05)
        if centre is None:
            raise Timeout('injection trigger never fired')

        distance = float(np.linalg.norm(self.drone_world()[0] - centre))
        late = distance < 1.5
        self.watch_obstacle(centre)
        self.injector.spawn(BOX, centre, self.args.obstacle_size)
        self.event('spawn', centre=[round(v, 3) for v in centre],
                   trigger_distance=round(distance, 3), late=bool(late))
        if not self.injector.wait_model(BOX, 5.):
            raise Timeout('spawned box never appeared in /gazebo/model_states')
        self.event('spawn_confirmed')
        return dict(centre=centre.tolist(), trigger_distance=distance, late=bool(late))

    def await_perception(self):
        hit = None
        deadline = time.monotonic() + self.args.perception_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            hits = self.voxels.hits(BOX)
            if hits:
                hit = hits[0]
                break
            time.sleep(0.1)
        if hit:
            self.event('perceived', voxel=[round(v, 3) for v in hit[2]],
                       count=len(self.voxels.hits(BOX)))
        return hit

    # ---- evaluation -------------------------------------------------------
    def _rows(self):
        rows = []
        with open(os.path.join(self.output, 'ground_truth.jsonl')) as handle:
            for line in handle:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    continue
        return rows

    def outcome(self, start_world, goal_world, along, perp, span_length, injection):
        """Score the group from the recorded truth only.

        Every window is clamped to the flight itself: samples taken while the box
        existed, up to and including the arrival sample.  The landing leg is
        excluded so a descent that happens to pass the box cannot decide a
        clearance or a route-return verdict.
        """
        rows = [r for r in self._rows() if r.get('world') and r.get('box')]
        result = dict(samples=len(rows))
        if not rows:
            result.update(code='F7_INFRASTRUCTURE', reason='no ground-truth samples with a box')
            return result

        arrival = next((i for i, r in enumerate(rows) if r.get('terminal_stage') == 'COMPLETED'),
                       len(rows) - 1)
        flight = rows[:arrival + 1]
        centre = np.asarray(injection['centre'], dtype=float)
        half = np.asarray(self.args.obstacle_size) / 2.
        along_box = float(np.dot(centre - start_world, along))

        points = np.asarray([r['world'] for r in flight], dtype=float)
        boxes = np.asarray([r['box'] for r in flight], dtype=float)
        nearest = np.clip(points, boxes - half, boxes + half)
        clearances = np.linalg.norm(points - nearest, axis=1) - BODY_RADIUS
        progress = (points - start_world) @ along
        deviation = (points - start_world) @ perp
        line_z = start_world[2] + np.clip(progress/max(span_length,1e-6),0.,1.)*(goal_world[2]-start_world[2])
        route_distance = np.sqrt(deviation**2+(points[:,2]-line_z)**2)

        # Route return: once past the box, the deviation has to come back to the
        # planned line and stay there.  Taking the maximum over the whole
        # post-box window would fail runs that are still recovering, so the
        # window is split at the point where the deviation first falls inside
        # the band; everything after it must stay inside.
        after = np.flatnonzero(progress > along_box + 1.0)
        recovery = None
        if after.size:
            suffix_max=np.maximum.accumulate(route_distance[::-1])[::-1]
            back = np.flatnonzero(suffix_max[after] <= 0.35)
            if back.size:
                recovery = int(after[back[0]])
        settled = None if recovery is None else float(route_distance[recovery:].max())

        stages = [r.get('navigation_stage') for r in flight]
        hits = self.voxels.hits(BOX)
        final = points[arrival]
        residual = self.transform.residual(final, np.asarray(flight[arrival]['odom'], dtype=float))

        result.update(
            samples=len(flight),
            min_clearance=round(float(clearances.min()), 4),
            max_deviation_from_line=round(float(route_distance.max()), 4),
            max_lateral_deviation_m=round(float(np.abs(deviation).max()), 4),
            deviation_after_box=None if settled is None else round(settled, 4),
            route_return_window=int(after.size),
            return_progress=None if recovery is None else round(float(progress[recovery]), 3),
            space_wait_seen=any(s == 'SPACE_WAIT' for s in stages),
            space_wait_samples=sum(1 for s in stages if s == 'SPACE_WAIT'),
            arrival_state=flight[arrival].get('flight_state'),
            final_goal_error=round(float(np.linalg.norm(final - goal_world)), 4),
            perception_hits=len(hits),
            frame_residual=round(float(residual), 4),
            goal_unchanged=bool(
                self.value('goal', None) is not None and self.goal_odom is not None
                and abs(self.value('goal').pose.position.x - self.goal_odom[0]) < 1e-6
                and abs(self.value('goal').pose.position.y - self.goal_odom[1]) < 1e-6
                and abs(self.value('goal').pose.position.z - self.goal_odom[2]) < 1e-6),
        )
        result['reached_completed'] = flight[arrival].get('terminal_stage') == 'COMPLETED'
        result['avoidance_manoeuvre'] = result['max_deviation_from_line'] >= 0.30
        # recovery is not None already means it re-entered the band and stayed.
        result['returned_to_route'] = recovery is not None

        if injection['late']:
            result['code'] = 'F7_INFRASTRUCTURE'
            result['reason'] = ('injected too late (%.2f m)' % injection['trigger_distance']
                                if injection['late']
                                else 'invalid initial frame alignment')
        elif result['min_clearance'] <= 0.:
            result['code'] = 'F3_COLLISION'
            result['reason'] = 'whole-airframe safety envelope reached the box (clearance %.3f m)' % result['min_clearance']
        elif not hits:
            result['code'] = 'F1_NO_PERCEPTION'
            result['reason'] = 'no box voxel ever reached /grid_map/voxel_delta'
        elif not result['space_wait_seen'] and not result['avoidance_manoeuvre']:
            result['code'] = 'F2_NO_REPLAN'
            result['reason'] = 'no SPACE_WAIT and no lateral deviation from the route line'
        elif not result['reached_completed'] or result['final_goal_error'] > 0.30:
            result['code'] = 'F4_GOAL_LOST'
            result['reason'] = ('did not complete (terminal %s)' % flight[arrival].get('terminal_stage')
                                if not result['reached_completed'] else
                                'final error %.2f m' % result['final_goal_error'])
        elif result['arrival_state'] in ('LANDING', 'FAILSAFE'):
            result['code'] = 'F6_HEALTH'
            result['reason'] = 'arrived in %s' % result['arrival_state']
        elif not result['goal_unchanged']:
            result['code'] = 'F4_GOAL_LOST'
            result['reason'] = 'the published goal changed during the flight'
        elif not result['returned_to_route']:
            result['code'] = 'F5_TIMEOUT' if result['route_return_window'] == 0 else 'F4_GOAL_LOST'
            result['reason'] = ('never passed the box, no post-box samples'
                                if result['route_return_window'] == 0 else
                                'never came back inside 0.35 m of the route line after the box '
                                '(closest %.2f m)' % float(route_distance[after].min()))
        else:
            result['code'] = 'SUCCESS'
            result['reason'] = 'perceived, avoided, returned to route, reached the original goal'
        return result

    # ---- modes ------------------------------------------------------------
    def run_group(self):
        self.group['mode'] = 'experiment'
        self.start_sampling()
        injection = None
        try:
            self.ready()
            if not getattr(self.args, 'reuse_airborne', False):
                self.arm_and_takeoff()
            start_odom, goal = self.send_goal((self.args.forward, self.args.lateral, getattr(self.args, "goal_offset_z", 0.)))
            start_world, goal_world, along, perp, span_length = self.geometry(start_odom)
            self.event('goal_issued', goal_odom=[round(v, 3) for v in self.goal_odom],
                       start_world=[round(v, 3) for v in start_world],
                       goal_world=[round(v, 3) for v in goal_world])

            injection = self.inject(start_world, goal_world)
            self.await_perception()
            deadline=time.monotonic()+self.args.arrive_timeout
            canceled_since=None
            while time.monotonic()<deadline:
                if self.text('terminal_stage')=='COMPLETED':break
                if self.text('flight_state') in ('HOLD','LANDING','FAILSAFE','READY','DESCENDING'):
                    canceled_since=canceled_since or time.monotonic()
                    if time.monotonic()-canceled_since>1.:
                        raise Timeout('navigation canceled after obstacle exposure: '+str(self.text('flight_error')))
                else:canceled_since=None
                time.sleep(.1)
            else:raise Timeout('goal not completed within trial deadline')
            self.arrival_sim = self.sim_now()
            self.event('arrived', sim=self.arrival_sim)
        except Timeout as exc:
            if injection is None:
                raise  # No obstacle exposure: readiness/harness failure.
            self.event('trial_timeout', error=str(exc))
            self.group['wait_error'] = str(exc)
        finally:
            # Freeze the evaluation at the end of navigation. Landing motion
            # cannot create a fake collision or a fake return-to-route result.
            if self._truth is not None:self._sample(0)
            self.stop_sampling()
            try:
                if getattr(self.args, 'keep_airborne', False) and self.text('flight_state') == 'HOLD' and self.text('flight_health') == 'HEALTHY':
                    self.event('hovering_between_groups')
                else:
                    self.land()
            except Exception as exc:
                rospy.logerr('landing: %s', exc)
        self.stop_sampling()
        result = self.outcome(start_world, goal_world, along, perp, span_length, injection)
        self.group.update(result, injection=injection, voxels=self.voxels.metadata(),
                          whole_airframe_envelope_radius_m=BODY_RADIUS)
        return self.group

    def run_feasibility(self):
        """Stage 1: prove the stack flies and the injection chain works."""
        self.group['mode'] = 'feasibility'
        steps = {}
        self.start_sampling()
        try:
            self.ready()
            steps['ready'] = True

            if not self.injector.available(30.):
                raise Timeout('gazebo spawn service absent; check rosservice list | grep gazebo')

            self.arm_and_takeoff()
            steps['takeoff'] = True
            sim_before = self.sim_now()

            self.send_goal((self.args.forward, 0., 0.))
            self.await_terminal()
            steps['goal_reached'] = True
            self.event('short_goal_done', sim_elapsed=round(self.sim_now() - sim_before, 2))

            # Parked well clear of the route, inside the lower hall.
            position=self.drone_world()[0]
            centre = np.array([position[0]+1.2,position[1]+2.,float(self.args.obstacle_size[2])/2.])
            self.watch_obstacle(centre)
            self.injector.spawn(BOX, centre, self.args.obstacle_size)
            steps['spawn'] = self.injector.wait_model(BOX, 5.)
            if not steps['spawn']:
                raise Timeout('box did not appear in /gazebo/model_states')
            steps['perceived'] = self.await_perception() is not None
            if not steps['perceived']:
                raise Timeout('box voxels never reached /grid_map/voxel_delta')
            self.injector.remove(BOX)
            steps['deleted'] = self.injector.wait_model(BOX, 10., present=False)
            if not steps['deleted']:
                raise Timeout('box did not disappear from /gazebo/model_states')
        finally:
            if self.injector.model(BOX) is not None:
                try:
                    self.injector.remove(BOX)
                except Exception:
                    pass
            try:
                self.land()
            except Exception as exc:
                rospy.logerr('landing: %s', exc)
        self.stop_sampling()
        self.group.update(steps=steps,
                          passed=all(steps.get(k) for k in ('ready', 'takeoff', 'goal_reached',
                                                            'spawn', 'perceived', 'deleted')),
                          voxels=self.voxels.metadata())
        return self.group

    def land(self):
        state = self.value('state')
        if state is not None and state.armed:
            self.call('/drone/land', Trigger)

            def disarmed():
                current = self.value('state')
                return current is not None and not current.armed

            self.wait(disarmed, PHASE_SCALE * 60., 'disarm after land')
        self.event('landed')

    def emergency(self):
        try:
            self.call('/drone/land', Trigger)
        except Exception as exc:
            rospy.logerr('emergency land: %s', exc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--feasibility', action='store_true')
    parser.add_argument('--map-mode',choices=['MAPPING','PRIOR_NAV'],default='MAPPING')
    parser.add_argument('--forward', type=float, default=5.5)
    parser.add_argument('--lateral', type=float, default=0.)
    parser.add_argument('--height', type=float, default=1.2)
    parser.add_argument('--obstacle-ahead', type=float, default=2.2)
    parser.add_argument('--obstacle-size', type=float, nargs=3, default=[0.30, 2.00, 1.80])
    parser.add_argument('--goal-retries', type=int, default=8,
                        help='re-issues of the same goal allowed while the vehicle is '
                             'still short of the injection point (map-freshness cancels)')
    parser.add_argument('--ready-timeout', type=float, default=READY_TIMEOUT)
    parser.add_argument('--inject-timeout', type=float, default=PHASE_SCALE * 120.)
    parser.add_argument('--perception-timeout', type=float, default=PHASE_SCALE * 30.)
    parser.add_argument('--arrive-timeout', type=float, default=PHASE_SCALE * 300.)
    args = parser.parse_args()

    rospy.init_node('dynamic_obstacle_experiment', anonymous=False)
    driver = Driver(args)
    started = time.monotonic()
    try:
        group = driver.run_feasibility() if args.feasibility else driver.run_group()
        group['ok'] = bool(group.get('passed')) if args.feasibility else group.get('code') == 'SUCCESS'
    except Exception as exc:
        rospy.logerr('group failed: %s', exc)
        driver.event('failed', error=str(exc))
        driver.emergency()
        driver.stop_sampling()
        group = dict(driver.group, ok=False, code='F7_INFRASTRUCTURE', reason=str(exc))
    group['events'] = driver.events
    group['wall_seconds'] = round(time.monotonic() - started, 1)
    with open(os.path.join(driver.output, 'group.json'), 'w') as handle:
        json.dump(group, handle, indent=2, sort_keys=True)
    print(json.dumps({k: group.get(k) for k in ('ok', 'mode', 'code', 'reason', 'wall_seconds')},
                     ensure_ascii=False))
    sys.exit(0 if group.get('ok') else 1)


if __name__ == '__main__':
    main()
