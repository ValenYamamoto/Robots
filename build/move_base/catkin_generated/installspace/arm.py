import rospy


# ROS builtins
import actionlib
from control_msgs.msg import FollowJointTrajectoryGoal, \
                             FollowJointTrajectoryAction, \
                             FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

class ArmController:
	self._joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']	
	def __init__(self):
		self.client = actionlib.SimpleActionClient('fibonacci', FollowJointTrajectoryAction)
		rospy.init_node( 'ArmTest', anonymous=False )
		while( True ):
			self.loop()
	
	def loop():
		keyboard_cmd = input().split()
		shoulder_pan_joint = float( keyboard_cmd[0] )
		shoulder_lift_joint = float( keyboard_cmd[1] )
		upperarm_roll_joint = float( keyboard_cmd[2] )
		elbow_flex_joint = float( keyboard_cmd[3] )
		forearm_roll_joint = float( keyboard_cmd[4] )
		wrist_flex_joint = float( keyboard_cmd[5] )
		wrist_roll_joint = float( keyboard_cmd[6] )

		arm_goal = [shoulder_pan_joint,
		           shoulder_lift_joint,
			   upperarm_roll_joint,
			   elbow_flex_joint,
			   forearm_roll_joint,
			   wrist_flex_joint,
			   wrist_roll_joint]
		self.send_arm_goal()

	def send_arm_goal(self, joint_points ):
		self.client.wait_for_server()

		traj = JointTrajectory()
		traj.joint_names = arm_joint_names
		traj.points.append( JointTrajectoryPoint() )
		traj.points[0].positions = joint_points
		traj.points[0].positions = rospy.Duration( 10 )

		traj_pos_goal = FollowJointTrajectoryGoal() 
		traj_pos_goal.traj = traj

		traj_pos_goal.goal_time_tolerance = rospy.Duration( 0 )
		self.client.send_goal( traj_pos_goal )
		self.client.wait_for_result()



if __name__ == "__main__":
	ArmController()	
