#!/usr/bin/python3

import sys
import pandas as pd

def get_dataframe( f ):
	data_dict = dict()
	for line in f:
		if line.startswith( "name" ):
			data_dict['name'] = line.split( ":" )[1][2:-1].split(', ') 
		if line.startswith( "position" ):
			data_dict['position'] = eval( line.split( ":" )[1] )
		if line.startswith( "velocity" ):
			data_dict['velocity'] = eval( line.split( ":" )[1] )
		if line.startswith( "effort" ):
			data_dict['effort'] = eval( line.split( ":" )[1] )
	return pd.DataFrame( data_dict )


if __name__ == '__main__':
	filename = sys.argv[1]

	with open( filename ) as f:
		df = get_dataframe( f )

	print( df )
			
