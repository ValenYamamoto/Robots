#!/usr/bin/python3
import rospy

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from datetime import datetime

import os
import yaml

import numpy as np

model_path = '/home/simulator/Robots/src/train_fetch/models/'

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

def move_model( proxy, pose=(0.75, 0, 0.25) ):
        #os.system( f"gz model -m contact_box -x {pose[0]} -y {pose[1]} -z {pose[2]} -R 0 -P 0 -Y 0" )
        msg = move_model_msg( "contact_box", pose=pose )
        return proxy( msg )

def move_model_msg( name, pose=(0, 0, 0.5), orientation=(0, 0, 0, 0)):
        state_msg = ModelState()
        state_msg.model_name = name 
        state_msg.pose.position.x = pose[0];
        state_msg.pose.position.y = pose[1];
        state_msg.pose.position.z = pose[2];
        state_msg.pose.orientation.x = orientation[0];
        state_msg.pose.orientation.y = orientation[1];
        state_msg.pose.orientation.z = orientation[2];
        state_msg.pose.orientation.w = orientation[3];
        return state_msg

def spawn_model( model_name='contact_box_ns1', pose=(0.75, 0, 0.25), namespace="/ns1/" ):
        initial_pose = Pose()
        initial_pose.position.x = pose[0]
        initial_pose.position.y = pose[1]
        initial_pose.position.z = pose[2]
        model_filename = model_path + model_name + '/model.sdf'
        model_xml = ''

        with open( model_filename ) as xml_file:
                model_xml = xml_file.read().replace( '\n', '' )

        spawn_model_prox = rospy.ServiceProxy( namespace + 'gazebo/spawn_sdf_model', SpawnModel )
        spawn_model_prox( 'contact_box', model_xml, '', initial_pose, 'world' )

def reset_world( proxy ):
        """
        reset_simulation = rospy.ServiceProxy( '/gazebo/reset_world', Empty )
        reset_simulation()
        """
        msg = move_model_msg( "fetch" )
        proxy( msg )
        #os.system( f"gz model -m fetch -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0" )

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

def reset_arm( arm_controller, initial_pos, move_proxy ):
        move_model( move_proxy, (2, 0, 0.25 ) )
        arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0], ignore_contact=True )
        arm_controller.send_arm_goal( initial_pos, ignore_contact=True )
        reset_world( move_proxy )
        move_model( move_proxy )

def get_logfilename():
        now = datetime.now()
        dt_string = now.strftime("%m%d-%H%M")
        name = "/home/simulator/Robots/src/train_fetch/logs/" + dt_string+"-pytorch.log"
        return name
        
def get_logfile(name):
        return open( name, 'a' )

def read_yaml( filename ):
        with open( filename ) as f:
                params_dict = yaml.safe_load( f )
                params_dict['params']['initial_std_dev']  = eval( params_dict['params']['initial_std_dev'] )
                params_dict['params']['actor_lr']  = float( params_dict['params']['actor_lr'] )
                params_dict['params']['critic_lr']  = float( params_dict['params']['critic_lr'] )
        return params_dict


def distance_from_goal( current, goal, tol=1e-1 ):
        d = distance(current, goal )
        return d, abs( d ) < tol

def distance( x, y ):
        return sum( (a-b)**2 for a, b in zip( x, y ) ) ** 0.5
