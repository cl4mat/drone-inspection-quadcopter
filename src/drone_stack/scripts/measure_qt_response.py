#!/usr/bin/env python3
"""Exercise the real Qt HOLD button and keep state/command timing evidence.

Launch Qt with DRONE_QT_TIMING_CSV=<output>/qt_timing.csv first. All reported
latencies are local CLOCK_MONOTONIC wall time, not ROS simulation time.
"""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from dynamic_obstacle_experiment import Driver


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--trials',type=int,default=5)
    parser.add_argument('--map-mode',choices=['MAPPING','PRIOR_NAV'],default='PRIOR_NAV')
    args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rospy.init_node('qt_response_measurement')
    args.height=1.2;args.ready_timeout=1200.;args.obstacle_size=[.3,2.,1.8]
    driver=Driver(args)
    command=rospy.Publisher('/drone/gui/timing_command',String,queue_size=1)
    results=[]
    try:
        driver.ready()
        driver.wait(lambda:command.get_num_connections()>0,60.,'instrumented Qt not connected')
        driver.arm_and_takeoff()
        for index in range(args.trials):
            origin=driver.drone_odom().copy()
            driver.send_goal((3. if index%2==0 else -3.,0.,0.))
            driver.wait(lambda:driver.text('flight_state')=='NAVIGATING'
                        and driver.text('navigation_stage')=='TRACKING'
                        and np.linalg.norm(driver.drone_odom()-origin)>.15,
                        240.,'navigation did not move before Qt HOLD trial')
            # Wait for the ordinary GUI refresh to enable the real button.
            time.sleep(.8)
            requested=time.monotonic()
            command.publish(String(data='hold'))
            driver.wait(lambda:driver.text('flight_state')=='HOLD',20.,'Qt HOLD command not accepted')
            row=dict(trial=index+1,request_monotonic_s=requested,
                     observer_hold_monotonic_s=time.monotonic(),sim_s=driver.sim_now())
            driver.event('qt_hold_trial',**row)
            results.append(row)
            (out/'command_trials.json').write_text(json.dumps(results,indent=2))
            time.sleep(3.)
    finally:
        driver.land()
        time.sleep(3.)
    command.publish(String(data='finish'))
    driver.wait(lambda:(out/'shot_qt_online.png').exists(),20.,'Qt finish/screenshot not acknowledged')
    data=list(csv.DictReader((out/'qt_timing.csv').open()))
    def stats(kind,column,flying=False):
        values=np.array([float(r[column]) for r in data if r['kind']==kind and r[column] and (not flying or r.get('armed')=='1')],float)
        return dict(n=len(values),mean=float(values.mean()),p95=float(np.percentile(values,95)),
                    max=float(values.max())) if len(values) else dict(n=0)
    summary=dict(trials=len(results),health_receive_ms=stats('health','receive_ms'),
                 health_display_ms=stats('health','display_ms'),
                 airborne_health_display_ms=stats('health','display_ms',True),
                 hold_service_ack_ms=stats('hold','service_ack_ms'),
                 hold_state_receipt_ms=stats('hold','hold_state_ms'),
                 hold_display_ms=stats('hold','display_ms'),
                 clock='Same-host monotonic wall time; not simulation time',
                 state_scope='Real flight-health snapshot publication to Qt callback/widget refresh',
                 hold_scope='Real Qt HOLD button slot to service ACK and HOLD phase callback/widget refresh; not physical stop time',
                 successful_hold_rows=sum(r['kind']=='hold' and r['success']=='1' for r in data))
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
