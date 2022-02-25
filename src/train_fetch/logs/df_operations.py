import argparse
import pandas


def read_pickle( filename ):
    return pandas.read_pickle( filename )

def combine_df( df1, df2, stop_index=None ):
    df1 = df1.drop( columns=["epoch"] )
    df2 = df2.drop( columns=["epoch"] )
    df1 = df1 if stop_index == None else df1[:stop_index]
    combine = pandas.concat( [df1, df2], ignore_index=True )
    combine["epoch"] = list( range( len( combine ) ) )
    return combine
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument( 'filename', type=str )
    parser.add_argument( '-p', '--print', action='store_true' )
    args = parser.parse_args()
    filename = args.filename

    df = pandas.read_pickle( filename )
    if args.print:
        print( df )
