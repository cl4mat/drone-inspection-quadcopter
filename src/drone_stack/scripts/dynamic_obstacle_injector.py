#!/usr/bin/env python3
"""Runtime static-obstacle injection and voxel-perception evidence.

The obstacle in this experiment is a *static* box spawned on the live route
after the vehicle is already under way, so the reactive re-plan path in
flight_manager is what has to cope with it.  This module owns the three pieces
the driver and the orchestrator both need:

  box_sdf / Injector   spawn and delete the box through gazebo_ros directly.
                       No spawn_model subprocess: memory is the scarce resource
                       on this machine, and a second ROS node is not needed.
  VoxelWatch           decode /grid_map/voxel_delta addresses into odom-frame
                       voxel centres and report which of them fall inside a
                       watched box -- the evidence that the re-planner's own
                       map really did see the obstacle.
  MapTransform         /drone/map_transform odom <-> world, plus a residual
                       check against Gazebo ground truth.

The address->index->centre arithmetic is the same one voxel_map_client,
map_session.read_preview and reports/icp_registration_20261011 use; it is
repeated here rather than re-derived.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped
from rospy.numpy_msg import numpy_msg
from plan_env.msg import VoxelUpdate

# ego.launch:43 cloud_body_half_extent -- the body sphere the planner inflates by
BODY_RADIUS = 0.40  # Conservative whole-airframe envelope, including rotors.
# box() convention of generate_building_world.py:76-81
BOX_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="{name}_link">
      <collision name="{name}_collision">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>
      <visual name="{name}_visual">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} 1.0</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def box_sdf(name, size, colour=(0.92, 0.35, 0.16)):
    sx, sy, sz = (float(v) for v in size)
    return BOX_SDF.format(name=name, sx=sx, sy=sy, sz=sz,
                          r=colour[0], g=colour[1], b=colour[2])


def Rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def yaw_of(quaternion):
    return math.atan2(2. * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1. - 2. * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def aabb_clearance(point, centre, half, body_radius=BODY_RADIUS):
    """Signed distance from the body sphere to an axis-aligned box.

    Negative means the conservative whole-airframe envelope entered the box.
    It is a safety-envelope failure, not proof of a physical contact event.
    """
    point = np.asarray(point, dtype=float)
    centre = np.asarray(centre, dtype=float)
    half = np.asarray(half, dtype=float)
    nearest = np.clip(point, centre - half, centre + half)
    return float(np.linalg.norm(point - nearest) - body_radius)


def box_corners(centre, size):
    """Eight world-frame corners of an axis-aligned box, for logging/plots."""
    centre = np.asarray(centre, dtype=float)
    half = np.asarray(size, dtype=float) / 2.
    return (centre + np.array([[sx, sy, sz] for sx in (-half[0], half[0])
                               for sy in (-half[1], half[1])
                               for sz in (-half[2], half[2])]))


class Injector:
    """gazebo_ros spawn/delete plus /gazebo/model_states lookup."""

    def __init__(self):
        self._spawn = None
        self._delete = None
        self._lock = threading.Lock()
        self._names = []
        self._poses = []
        self._stamp = 0.
        self.spawn_service = '/gazebo/spawn_sdf_model'
        self.delete_service = '/gazebo/delete_model'
        self.subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                           self._receive, queue_size=1)

    # ---- services ---------------------------------------------------------
    def available(self, seconds=20.):
        """True when both gazebo services are up; caches the proxies."""
        if self._spawn is not None:
            return True
        try:
            rospy.wait_for_service(self.spawn_service, timeout=seconds)
            rospy.wait_for_service(self.delete_service, timeout=seconds)
        except rospy.ROSException:
            return False
        try:
            self._spawn = rospy.ServiceProxy(self.spawn_service, SpawnModel)
            self._delete = rospy.ServiceProxy(self.delete_service, DeleteModel)
        except rospy.ServiceException:
            self._spawn = self._delete = None
            return False
        return True

    def spawn(self, name, centre, size, frame='world', timeout=20.):
        if not self.available(timeout):
            raise RuntimeError('gazebo spawn service %s unavailable' % self.spawn_service)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in centre)
        pose.orientation.w = 1.0
        reply = self._spawn(model_name=name, model_xml=box_sdf(name, size),
                            robot_namespace='', initial_pose=pose, reference_frame=frame)
        if not reply.success:
            raise RuntimeError('spawn_sdf_model(%s) refused: %s' % (name, reply.status_message))
        return reply

    def remove(self, name, timeout=20.):
        if not self.available(timeout):
            raise RuntimeError('gazebo delete service %s unavailable' % self.delete_service)
        reply = self._delete(model_name=name)
        # Deleting an absent model is not an error for teardown purposes.
        return reply

    # ---- ground truth -----------------------------------------------------
    def _receive(self, message):
        names = list(message.name)
        poses = list(message.pose)
        with self._lock:
            self._names = names
            self._poses = poses
            self._stamp = time.monotonic()

    def fresh(self, seconds=5.):
        # Wall clock: the simulation runs well below real time, so a "recent"
        # reading here is a much wider wall window than it would be in real time.
        with self._lock:
            return bool(self._names) and (time.monotonic() - self._stamp) <= seconds

    def model(self, name):
        """(position, yaw) in world frame, or None when the model is absent."""
        with self._lock:
            if name not in self._names:
                return None
            pose = self._poses[self._names.index(name)]
        return (np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float),
                yaw_of(pose.orientation))

    def wait_model(self, name, seconds, present=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (self.model(name) is not None) == present:
                return True
            time.sleep(0.05)
        return False


class MapTransform:
    """odom <-> world from the latched /drone/map_transform PoseStamped.

    map_session publishes map->odom as world = Rz(yaw) @ p_odom + translation,
    the exact inverse of set_alignment's construction, so that same convention
    is used here rather than guessed at.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.translation = np.zeros(3)
        # map_session's transform maps odom to the birth-centred map, whereas
        # Gazebo's body centre rests at world Z=.17 m in the current world.
        self.world_from_map_translation = np.array([0.,0.,.17])
        self.yaw = 0.
        self.ready = False
        self.stamp = None
        self.subscriber = rospy.Subscriber('/drone/map_transform', PoseStamped,
                                           self._receive, queue_size=1)

    def _receive(self, message):
        p = message.pose.position
        with self._lock:
            self.translation = np.array([p.x, p.y, p.z], dtype=float)
            self.yaw = yaw_of(message.pose.orientation)
            self.ready = True
            self.stamp = time.monotonic()

    def _snapshot(self):
        with self._lock:
            return self.translation.copy(), self.yaw

    def odom_to_world(self, point):
        translation, yaw = self._snapshot()
        return Rz(yaw) @ np.asarray(point, dtype=float) + translation + self.world_from_map_translation

    def world_to_odom(self, point):
        translation, yaw = self._snapshot()
        return Rz(yaw).T @ (np.asarray(point, dtype=float) - self.world_from_map_translation - translation)

    def residual(self, drone_world, drone_odom):
        """Metres between Gazebo truth and where the transform puts the odom pose.

        ONLINE mode starts at identity and drifts with the LIO estimate, so this
        is a live health number: >0.3 m means the two frames disagree and the
        group must be discarded as an infrastructure failure.
        """
        return float(np.linalg.norm(np.asarray(drone_world, dtype=float)
                                    - self.odom_to_world(drone_odom)))


class VoxelWatch:
    """Which newly-occupied voxels land inside a watched box, in odom frame."""

    MAX_HITS = 20000

    def __init__(self):
        self._lock = threading.Lock()
        self._watched = {}
        self._hits = {}
        self._active = {}
        self._removed_counts = {}
        self.origin = None
        self.resolution = None
        self.shape = None
        self.epoch = 0
        self.revision = 0
        self.messages = 0
        self.added_total = 0
        # Acquisition time of the newest delta, in simulation seconds.  This is
        # the same number flight_manager stores as map_time, so the driver can
        # report the freshness the flight supervisor is actually acting on.
        self.last_stamp = None
        # Freshness statistics over *every* delta.  The driver's sampler is far
        # slower than the map publishes, so the peak age that flight_manager
        # actually compares against max_map_age_s is only visible here.
        self.age_count = 0
        self.age_sum = 0.
        self.age_max = 0.
        self.age_bins = [0] * 13                      # 0.5 s bins, then overflow
        self.gap_max = 0.
        self._previous_stamp = None
        self.subscriber = rospy.Subscriber('/grid_map/voxel_delta',
                                           numpy_msg(VoxelUpdate), self._receive,
                                           queue_size=50, buff_size=8 * 1024 * 1024,
                                           tcp_nodelay=True)

    def watch(self, key, centre, size, pad=0.):
        centre = np.asarray(centre, dtype=float)
        half = np.asarray(size, dtype=float) / 2. + pad
        with self._lock:
            self._watched[key] = (centre, half)
            self._hits[key] = []
            self._active[key] = set()
            self._removed_counts[key] = 0
        return key

    def release(self, key):
        with self._lock:
            self._watched.pop(key, None)

    def hits(self, key):
        with self._lock:
            return list(self._hits.get(key, ()))

    def active_count(self, key):
        with self._lock:
            return len(self._active.get(key, ()))

    def removed_count(self, key):
        with self._lock:
            return self._removed_counts.get(key, 0)

    def first_hit(self, key):
        seen = self.hits(key)
        return seen[0] if seen else None

    def current_stamp(self):
        """Acquisition stamp (sim seconds) of the newest delta, or None."""
        with self._lock:
            return self.last_stamp

    def metadata(self):
        with self._lock:
            return dict(origin=None if self.origin is None else self.origin.tolist(),
                        resolution=self.resolution,
                        shape=None if self.shape is None else list(self.shape),
                        epoch=self.epoch, revision=self.revision,
                        messages=self.messages, added=self.added_total,
                        stamp=self.last_stamp,
                        age=dict(count=self.age_count,
                                 mean=None if not self.age_count
                                 else round(self.age_sum / self.age_count, 3),
                                 max=None if not self.age_count else round(self.age_max, 3),
                                 max_gap=None if self._previous_stamp is None
                                 else round(self.gap_max, 3),
                                 bins=self.age_bins))

    def _receive(self, message):
        added = np.asarray(message.occupied_added)
        shape = tuple(int(v) for v in message.shape)
        origin = np.array([message.origin.x, message.origin.y, message.origin.z], dtype=float)
        resolution = float(message.resolution)
        stamp = time.monotonic()
        try:
            age = (rospy.Time.now() - message.header.stamp).to_sec()
        except rospy.ROSException:
            # /clock has not been seen yet; the delta itself is still decoded.
            age = None
        with self._lock:
            self.origin, self.resolution, self.shape = origin, resolution, shape
            self.epoch, self.revision = int(message.epoch), int(message.revision)
            self.last_stamp = float(message.header.stamp.to_sec())
            self.messages += 1
            self.added_total += int(added.size)
            if age is not None:
                self.age_count += 1
                self.age_sum += age
                self.age_max = max(self.age_max, age)
                self.age_bins[min(int(age / .5), len(self.age_bins) - 1)] += 1
            if self._previous_stamp is not None:
                self.gap_max = max(self.gap_max, self.last_stamp - self._previous_stamp)
            self._previous_stamp = self.last_stamp
            removed = set(int(v) for v in message.occupied_removed)
            for key, active in self._active.items():
                self._removed_counts[key] += len(active & removed)
                active.difference_update(removed)
                if message.full:
                    active.clear()
            if added.size == 0 or not self._watched:
                return
            if message.header.frame_id != 'odom' or resolution <= 0. or len(shape) != 3:
                return
            address = added.astype(np.int64)
            size = int(np.prod(shape))
            address = address[(address >= 0) & (address < size)]
            if address.size == 0:
                return
            nx, ny, nz = shape
            index = np.column_stack((address // (ny * nz),
                                     address // nz % ny,
                                     address % nz))
            centres = origin + (index + .5) * resolution
            for key, (box_centre, box_half) in self._watched.items():
                inside = np.all(np.abs(centres - box_centre) <= box_half, axis=1)
                if not inside.any():
                    continue
                self._active[key].update(int(v) for v in address[inside])
                bucket = self._hits[key]
                for point in centres[inside]:
                    if len(bucket) >= self.MAX_HITS:
                        break
                    bucket.append((stamp, float(message.header.stamp.to_sec()), point.tolist()))


def voxel_hits_in_box(hits, centre, size, pad=0.):
    """Filter a recorded hit list against a box; used when boxes move."""
    centre = np.asarray(centre, dtype=float)
    half = np.asarray(size, dtype=float) / 2. + pad
    return [h for h in hits if np.all(np.abs(np.asarray(h[2]) - centre) <= half)]
