import pandas
import seaborn as sns
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import df_operations

def create_bin_array( arr ):
    bins = {
        1: (-51, -49),
        2: (-3.5, -1.8),
        3: (-1.8, -1.2),
        4: (-1.2, -1),
        5: (-1, -0.6),
        6: (-0.6, -0.2),
        7: (-0.2, 0.2),
        8: (0.2, 0.6),
        9: (0.6, 1.1),
        10: (99, 101),
    }
    binned = []
    for x in arr.ravel():
        for value, (a, b) in bins.items():
            if a <= x < b:
                binned.append( value )
    return binned
    
def create_histogram( df, start, stop ):
    labels = ["(-51, -49)",
        "(-3.5, -1.8)",
        "(-1.8, -1.2)",
        "(-1.2, -1)",
        "(-1, -0.6)",
        "(-0.6, -0.2)",
        "(-0.2, 0.2)",
        "(0.2, 0.6)",
        "(0.6, 1.1)",
        "(99, 101)",]
    arr = df[start:stop].to_numpy()
    binned = create_bin_array( arr )
    n, bins, patches = plt.hist( binned, bins=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] )
    for rect, label in zip(patches, labels):
        height = rect.get_height()
        plt.text(rect.get_x() + rect.get_width() / 2, height+0.01, label,
            ha='center', va='bottom')