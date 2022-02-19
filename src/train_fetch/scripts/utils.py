#!/usr/bin/python3
import rospy

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from std_srvs.srv import Empty
from datetime import datetime

import os

import numpy as np

model_path = '/home/valen/py3_ws/src/train_fetch/models/'

joint_names = ['shoulder_pan_joint',
                    'shoulder_lift_joint',
                    'upperarm_roll_joint',
                    'elbow_flex_joint',
                    'forearm_roll_joint',
                    'wrist_flex_joint',
                    'wrist_roll_joint'] 

joint_names = {name:index for index, name in enumerate( joint_names ) }
def delete_model( model_name='contact_box' ):
        os.system( "gz model -m contact_box -d" )

def move_model( pose=(0.75, 0, 0) ):
        os.system( f"gz model -m contact_box -x {pose[0]} -y {pose[1]} -z {pose[2]} -R 0 -P 0 -Y 0" )

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

def reset_world():
        reset_simulation = rospy.ServiceProxy( '/gazebo/reset_world', Empty )
        reset_simulation()

def get_gripper_pos( data ):
        pos = []
        for name, pose in zip( data.name, data.pose ):
                if 'gripper_finger_link' in name: 
                        pos.append( (pose.position.x,pose.position.y,pose.position.z) )
                        if len(pos) == 2:
                                break
        return tuple( (pos[0][i]+pos[1][i])/2 for i in range( 3 ) )

def get_box_position( data ):
        pos = 0 
        for name, pose in zip( data.name, data.pose ):
                if 'contact_box' in name: 
                        pos =  (pose.position.x,pose.position.y,pose.position.z)
                        break
        if pos == 0:
                return 7
        return not any( i>1 for i in pos )

def get_joint_data( data ):
        answer = np.zeros( 7 )
        for name, position in zip( data.name, data.position ):
                if name in joint_names:
                        answer[joint_names[name]] = position
        return answer

def set_arm( arm_controller, initial_pos ):
        delete_model()
        arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
        reset_world()
        arm_controller.send_arm_goal( initial_pos )
        spawn_model()

def get_logfilename():
        now = datetime.now()
        dt_string = now.strftime("%m%d-%H%M")
        name = "/home/valen/py3_ws/src/train_fetch/logs/" + dt_string+"-pytorch.log"
        return name
        
def get_logfile(name):
        return open( name, 'a' )



