import argparse
import pandas

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument( 'filename', type=str )
    parser.add_argument( '-p', '--print', action='store_true' )
    args = parser.parse_args()
    filename = args.filename

    df = pandas.read_pickle( filename )
    if args.print:
        print( df )
