import numpy as np

a, b, c, d, e, f = [], [], [], [], [], []


with open( "arm_positions") as fil:
    for line in fil:
        x, y, z = eval( line )
        a.append( x )
        b.append(y )
        c.append( z )
        line = next( fil)
        x, y, z = eval( line )
        d.append( x )
        e.append(y )
        f.append( z )
print( "INITIAL" )
print( np.mean( a ) )
print( np.std( a ) )
print( np.mean( b ) )
print( np.std( b ) )
print( np.mean( c ) )
print( np.std( c ) )
print( "GOAL" )
print( np.mean( d ) )
print( np.std( d ) )
print( np.mean( e ) )
print( np.std( e ) )
print( np.mean( f ) )
print( np.std( f ) )
