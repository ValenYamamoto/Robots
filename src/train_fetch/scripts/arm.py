#!/usr/bin/python3
import rospy

import actionlib
import datetime
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState


class ArmController:
        _joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint'] 

        CONTACT_MADE = False

        def __init__(self):
                self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
                #rospy.init_node( 'ArmTest', anonymous=False )
                rospy.Subscriber( "/box_contact_topic", ContactsState, self.contactMade )


        def contactMade( self, data ):
                states = data.states
                for state in states:
                        if "ground_plane" not in state.collision1_name and "ground_plane" not in state.collision2_name:
                                self.CONTACT_MADE = True
                                break
        
        def send_arm_goal(self, joint_points, previous=None ):
                duration = 1
                if previous is not None:
                        diff = [ abs(x-y) for x, y in zip( joint_points, previous ) ]
                        if max( diff ) > 1:
                                duration = 3
                #rospy.loginfo( "Waiting for action server" )
                self.client.wait_for_server()
                #rospy.loginfo( "Connected to action server" )
                time_limit = 2+duration-1

                traj = JointTrajectory()
                traj.joint_names = self._joint_names
                traj.points.append( JointTrajectoryPoint() )
                traj.points[0].positions = joint_points
                traj.points[0].velocities =  [0.0] * len(joint_points)
                traj.points[0].accelerations = [0.0] * len(joint_points)
                traj.points[0].time_from_start = rospy.Duration( 2 )

                traj_pos_goal = FollowJointTrajectoryGoal() 
                traj_pos_goal.trajectory = traj
                if previous is not None:
                        pos_tol = [ 0.1*abs(x-y) for x, y in zip( joint_points, previous ) ]
                        tolerances = []
                        for i, name in enumerate( self._joint_names ):
                                j = JointTolerance()
                                j.name = name
                                j.position = pos_tol[i]
                                j.velocity = 5
                                j.acceleration = 5
                                tolerances.append( j )
                        traj_pos_goal.goal_tolerance = tolerances 

                traj_pos_goal.goal_time_tolerance = rospy.Duration( duration )
                #rospy.loginfo( "Sent goal successfully" )
                #rospy.loginfo( "State:" + self.status_string( self.client.get_state() ) )
                self.client.send_goal( traj_pos_goal )
                time_start = rospy.Time.now()
                #print( "TIME LIMIT:", time_limit )

                self.CONTACT_MADE = False
                while self.client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                        if (rospy.Time.now() - time_start).to_sec() > time_limit:
                                self.client.cancel_all_goals()
                                return 3
                        if self.CONTACT_MADE == True:
                                self.client.cancel_all_goals()
                                self.CONTACT_MADE = False
                                return 1

                if self.client.get_state() == GoalStatus.ABORTED:
                        return 2
                elif self.client.get_state() == GoalStatus.SUCCEEDED:
                        return 0


        def status_string( self, i ):
                status = {GoalStatus.LOST: "LOST",
                         GoalStatus.RECALLING: "RECALLING",
                         GoalStatus.PENDING: "PENDING",
                         GoalStatus.PREEMPTING: "PREEMPTING",
                         GoalStatus.PREEMPTED: "PREEMPTED",
                         GoalStatus.ACTIVE: "ACTIVE",
                         GoalStatus.SUCCEEDED: "SUCCEEDED",
                         GoalStatus.ABORTED: "ABORTED",
                         GoalStatus.REJECTED: "REJECTED",
                         GoalStatus.RECALLED: "RECALLED"
                         }
                return status[i]



if __name__ == "__main__":
        ArmController() 