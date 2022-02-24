#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
import argparse


model_path = '/home/valen/py3_ws/src/train_fetch/models/'

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


if __name__ == '__main__':
        """
        parser = argparse.ArgumentParser()
        parser.add_argument( '--pos', type=str, default="(0.75, 0, 0)" )
        args.parser.parse_args()
        pose = eval( args.pos )
        """
        spawn_model( model_name='contact_box', pose=(5, 0, 0) )
