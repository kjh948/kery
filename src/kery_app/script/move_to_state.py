#!/usr/bin/env python

import roslib
import rospy
import argparse

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from smach import StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
'''
Frame:map, Position(1.896, 0.822, 0.000), Orientation(0.000, 0.000, -0.105, 0.995) = Angle: -0.209
Frame:map, Position(0.539, 0.147, 0.000), Orientation(0.000, 0.000, -0.768, 0.641) = Angle: -1.750
Frame:map, Position(0.034, 1.404, 0.000), Orientation(0.000, 0.000, 0.704, 0.710) = Angle: 1.563
'''
WAYPOINTS = {
    'P1': (1.9, 0.8, 'P2'),
    'P2': (0.5, 0.15, 'P3'),
    'P3': (0.0, 1.4, 'P1'),
}

def main():
    rospy.init_node('smach_example_state_machine')

    sm = StateMachine(['exit'])
    with sm:
        for key, (x, y, next_point) in WAYPOINTS.items():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.

            StateMachine.add(key, SimpleActionState('move_base',
                                                MoveBaseAction,
                                                goal=goal),
                              transitions={'succeeded': next_point, 'aborted': 'exit', 'preempted': 'exit'})

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
