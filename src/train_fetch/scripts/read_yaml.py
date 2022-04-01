import utils

params_dict = utils.read_yaml( "params.yaml" )
print( params_dict )
for key in params_dict.keys():
    for k, v in params_dict[ key ].items():
        print( k, type( v ) )
        if type( v ) == list:
            for i in v:
                print( type( i ) )

