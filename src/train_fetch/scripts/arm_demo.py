#!/usr/bin/python3
import rospy


import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from std_srvs.srv import Empty

def delete_model( model_name='contact_box' ):
	delete_model_prox = rospy.ServiceProxy( 'gazebo/delete_model', DeleteModel )
	delete_model_prox( model_name )

model_path = '/home/simulator/Robots/src/train_fetch/models/'

def spawn_model( model_name='contact_box', pose=(0.75, 0, 0) ):
	initial_pose = Pose()
	initial_pose.position.x = pose[0]
	initial_pose.position.y = pose[1]
	initial_pose.position.z = pose[2]
	model_filename = model_path + model_name + '/model.sdf'
	model_xml = ''

	with open( model_filename ) as xml_file:
		model_xml = xml_file.read().replace( '\n', '' )

	spawn_model_prox = rospy.ServiceProxy( 'gazebo/spawn_sdf_model', SpawnModel )
	spawn_model_prox( 'contact_box', model_xml, '', initial_pose, 'world' )


CONTACT_MADE = 0
def printContact( data ):
	states = data.states
	for state in states:
		if "ground_plane" not in state.collision1_name and "ground_plane" not in state.collision2_name:
			CONTACT_MADE = 1
			"""
			print( "Info:", state.info )
			print( "collision1_name:", state.collision1_name )
			print( "collision2_name:", state.collision2_name )
			print( "wrenches:", state.wrenches )
			print( "total_wrench:", state.total_wrench )
			print( "contact_positions:", state.contact_positions )
			print( "contact_normals:", state.contact_normals )
			print( "depths:", state.depths )
			"""

INITIAL_ARM_POSTION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
class ArmController:
	_joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']	
	def __init__(self):
		self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.init_node( 'ArmTest', anonymous=False )
		rospy.Subscriber( "/box_contact_topic", ContactsState, self.printContact )
		while( True ):
			self.loop()
	
		self.CONTACT_MADE = 0

	def printContact( self, data ):
		states = data.states
		for state in states:
			if "ground_plane" not in state.collision1_name and "ground_plane" not in state.collision2_name:
				self.CONTACT_MADE = 1
				return

	def loop( self ):
		shoulder_pan_joint = float( input("shoulder pan joint: "))
		shoulder_lift_joint = float( input("shoulder lift joint: ") )
		upperarm_roll_joint = float( input("upperarm roll joint: ") )
		elbow_flex_joint = float( input("elbow flex joint: ") )
		forearm_roll_joint = float( input("forearm roll joint: ") )
		wrist_flex_joint = float( input("wrist flex joint: ") )
		wrist_roll_joint = float( input("wrist roll joint: ") )

		arm_goal = [shoulder_pan_joint,
		           shoulder_lift_joint,
			   upperarm_roll_joint,
			   elbow_flex_joint,
			   forearm_roll_joint,
			   wrist_flex_joint,
			   wrist_roll_joint]
		result_code = self.send_arm_goal( arm_goal )
		if result_code == 0:
			pass
		elif result_code == 1:
			delete_model()
			self.send_arm_goal(INTIAL_ARM_POSITION)
			reset_simulation = rospy.ServiceProxy( '/gazebo/reset_world', Empty )
			reset_simulation()
			spawn_model()
		elif result_code == 2:
			print( "ABORTED!!" )


	def send_arm_goal(self, joint_points ):
		rospy.loginfo( "Waiting for action server" )
		self.client.wait_for_server()
		rospy.loginfo( "Connected to action server" )

		traj = JointTrajectory()
		traj.joint_names = self._joint_names
		traj.points.append( JointTrajectoryPoint() )
		traj.points[0].positions = joint_points
		traj.points[0].velocities =  [0.0] * len(joint_points)
		traj.points[0].accelerations = [0.0] * len(joint_points)
		traj.points[0].time_from_start = rospy.Duration( 4 )

		traj_pos_goal = FollowJointTrajectoryGoal() 
		traj_pos_goal.trajectory = traj

		traj_pos_goal.goal_time_tolerance = rospy.Duration( 0 )
		rospy.loginfo( "Sent goal successfully" )
		self.client.send_goal( traj_pos_goal )

		self.CONTACT_MADE = 0
		while self.client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
			if self.CONTACT_MADE == 1:
				self.client.cancel_all_goals()
				self.CONTACT_MADE = 0
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
