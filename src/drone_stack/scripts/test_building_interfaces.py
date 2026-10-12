#!/usr/bin/env python3
"""No ROS node/master: mission ownership, startup and pause regression checks."""
import json
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State,ExtendedState
from drone_stack.srv import BuildingCommandRequest,ExecuteInspectionRouteRequest
from building_manager import BuildingManager
from flight_manager import FlightManager
from drone_ai_contract import validate_plan


class BuildingInterfaces(unittest.TestCase):
    def manager(self):
        n=BuildingManager.__new__(BuildingManager);n.lock=threading.RLock();n.state='READY';n.task={'map_path':'/tmp/map.dmap'}
        n.serial=0;n.index=0;n.progress=0.;n.owner='';n.record_path=None;n.ai={};n.ai_seen=0.
        n.exclusive=Mock();n.phase='READY';n.authorized=True;n.health='HEALTHY';n.fcu=State();n.extended=ExtendedState()
        n.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND;n.check_start=Mock();n.call_flight=Mock()
        n.activate_voxel=Mock();n.preview=Mock();n.record=Mock();n.save_progress=Mock();return n

    def test_start_does_not_race_first_heartbeat(self):
        n=self.manager()
        with patch.object(rospy.Time,'now',return_value=rospy.Time(70)),patch.object(rospy,'loginfo'):
            reply=n.command(BuildingCommandRequest(action='start'))
        self.assertTrue(reply.success);self.assertEqual(n.state,'ARMING');n.call_flight.assert_not_called()

    def test_pause_preserves_progress(self):
        n=self.manager();n.index=5;n.progress=3.2
        with patch.object(rospy.Time,'now',return_value=rospy.Time(70)),patch.object(rospy,'loginfo'):n.pause('blocked stair')
        self.assertEqual(n.state,'PAUSED');self.assertEqual(n.index,5);self.assertEqual(n.progress,3.2);n.call_flight.assert_called_with('hold')

    def test_late_ai_cancel_cannot_cancel_manual_takeover(self):
        n=self.manager();n.owner=''
        reply=n.command(BuildingCommandRequest(action='cancel',owner='ai:old|request'))
        self.assertFalse(reply.success);self.assertEqual(n.state,'READY')

    def test_stale_or_idle_building_cannot_refresh_execution(self):
        n=FlightManager.__new__(FlightManager);n.building_heartbeat=rospy.Time(70);n.inspection_heartbeat=rospy.Time(70)
        n.inspection_owner='building';n.inspection_task=123
        with patch.object(rospy.Time,'now',return_value=rospy.Time(71)):
            n.on_building_status(String(json.dumps(dict(stamp_ns=71*10**9,state='IDLE',task_id=123))))
            self.assertEqual(n.inspection_heartbeat,rospy.Time(70))
            n.on_building_status(String(json.dumps(dict(stamp_ns=71*10**9,state='INSPECTING',task_id=456))))
            self.assertEqual(n.inspection_heartbeat,rospy.Time(70))
            n.on_building_status(String(json.dumps(dict(stamp_ns=71*10**9,state='INSPECTING',task_id=123))))
            self.assertEqual(n.inspection_heartbeat,rospy.Time(70)) # duplicate time rejected

    def test_regional_and_building_heartbeat_separate(self):
        n=FlightManager.__new__(FlightManager);n.inspection_owner='building';n.inspection_heartbeat=rospy.Time(70)
        n.on_inspection_status(String(json.dumps(dict(stamp_ns=72*10**9))))
        self.assertEqual(n.inspection_heartbeat,rospy.Time(70))

    def test_regional_cannot_overwrite_building(self):
        n=FlightManager.__new__(FlightManager);n.lock=threading.RLock();n.building_reserved=True
        reply=n.execute_inspection_route(ExecuteInspectionRouteRequest(owner=''))
        self.assertFalse(reply.success)

    def test_building_ai_action_is_whitelisted(self):
        self.assertEqual(validate_plan(dict(steps=[dict(tool='inspect_building',arguments={})]),[])['steps'][0]['tool'],'inspect_building')


if __name__=='__main__':unittest.main()
