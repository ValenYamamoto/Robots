#!/usr/bin/python3

import rospy
import utils


def delete_model( model_name='contact_box' ):
	delete_model_prox = rospy.ServiceProxy( 'gazebo/delete_model', DeleteModel )
	delete_model_prox( model_name )

if __name__ == '__main__':
	utils.delete_model()
