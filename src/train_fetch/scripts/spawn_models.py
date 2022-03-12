#!/usr/bin/python3

import rospy
import utils

rospy.init_node( "Spawn_models" )
utils.spawn_model( model_name='contact_box', pose=(0.75, 0, 0), namespace="ns1" ):

