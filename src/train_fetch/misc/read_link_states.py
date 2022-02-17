#!/usr/bin/python3

import sys
import pandas as pd

def get_dict( f ):
	data_dict = dict()
	for line in f:
		if line.startswith( "name" ):
			data_dict['name'] = eval( line[5:] )

		if line.startswith( "pose" ):
			data_dict['position'] = []
			for l in f:
				if 'position' in l:
					x = float( next(f).split()[1] )
					y = float( next(f).split()[1] )
					z = float( next(f).split()[1] )
					data_dict['position'].append( (x,y,z) )

			return data_dict

					

def get_dataframe( data_dict ):
	return pd.DataFrame( data_dict )

if __name__ == "__main__":
	filename = sys.argv[1]

	with open( filename ) as f:
		data_dict = get_dict( f )
	
	print( get_dataframe( data_dict ) )
