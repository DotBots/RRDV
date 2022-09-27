import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import matplotlib.dates as mdates

from dist_to_line import minDistance

# Configuration
ANGLE = 20 # degrees
ANGLE = np.cos(np.radians(ANGLE))

ROBOT_SIZE = 100 # mm radius

# Initialize video input
file_path = 'Experiments/220824 - Third experiment/1 - transmit all/1_transmit_all-raw.csv'
# file_path = 'Experiments/220824 - Third experiment/4 - transmit pairs/4_transmit_pairs-raw.csv'
file_name = file_path.split('/')[-1]
print(f"file to analyze = {file_name}")

## read the data
df = pd.read_csv(file_path, index_col=0)
# print(df.tail())

# Create a new dataframe for the distances
df_dist = pd.DataFrame()

# Calculate distances between robots
robot_combination = [[1,2], [1,3], [1,4], [1,5], [2,3], [2,4], [2,5], [3,4], [3,5], [4,5]] 
df_dist['timestamp'] = df['timestamp']

# calculate the distance between robots
for a, b in robot_combination:
    df_dist[f'd{a}_{b}'] = ((df[f'r{b}_x'] - df[f'r{a}_x'])**2 + (df[f'r{b}_y'] - df[f'r{a}_y'])**2)**(1/2)

# robot angle
df_vec = pd.DataFrame()
# calculate the  unit vector between robots
for a, b in robot_combination:
    df_vec[f'v{a}{b}_x'] = (df[f'r{b}_x'] - df[f'r{a}_x']) / df_dist[f'd{a}_{b}']
    df_vec[f'v{a}{b}_y'] = (df[f'r{b}_y'] - df[f'r{a}_y']) / df_dist[f'd{a}_{b}']
# Calculate individual robot unit vectors
for a in range(1,5 +1):
    df_vec[f'v{a}_x'] = np.cos(np.radians(df[f'r{a}_a']))
    df_vec[f'v{a}_y'] = np.sin(np.radians(df[f'r{a}_a']))

# robot angle
df_angle = pd.DataFrame()
# are they looking at each other? Calculate the dot product between the robot direction and the connecting vector
for a, b in robot_combination:
    df_angle[f'a{a}{b}'] = [np.dot([xa, ya], [xab, yab]) for xa,ya,xab,yab in zip(df_vec[f'v{a}_x'], df_vec[f'v{a}_y'], df_vec[f'v{a}{b}_x'], df_vec[f'v{a}{b}_y'])  ]
    df_angle[f'a{b}{a}'] = [np.dot([xb, yb], [xab, yab]) for xb,yb,xab,yab in zip(df_vec[f'v{b}_x'], df_vec[f'v{b}_y'], df_vec[f'v{a}{b}_x'], df_vec[f'v{a}{b}_y'])  ]

# robot angle
df_look = pd.DataFrame()
# are they looking at each other?
for a, b in robot_combination:
    print(f'[{a},{b}]')

    other_bots = [1,2,3,4,5]  # Use only the robots that got detected by ARUCO
    other_bots.remove(a)
    other_bots.remove(b)

    # Check that the robots are looking at each other
    df_look[f'look{a}{b}'] = ((df_angle[f'a{a}{b}'] > ANGLE) & (df_angle[f'a{b}{a}'] < -ANGLE))

    # Check that no other robot is too close, go line by line, because the minDistance function doesn't like vectorization.
    for idx, row in df_look.iterrows():
        for bot in other_bots:
            row[f'look{a}{b}'] &= (minDistance([df[f"r{a}_x"].at[idx], df[f"r{a}_y"].at[idx]], [df[f"r{b}_x"].at[idx], df[f"r{b}_y"].at[idx]], [df[f"r{bot}_x"].at[idx], df[f"r{bot}_y"].at[idx]]) > ROBOT_SIZE)

## assemble the results in a final dataframe
df_result = pd.DataFrame()
for a, b in robot_combination:
    df_result[f'd_{a}{b}'] = df_dist[f'd{a}_{b}']
    df_result[f'look_{a}{b}'] = df_look[f'look{a}{b}']
# Add the process to the original file
df_result = pd.concat([df,df_result],axis=1)
df_result.to_csv(file_path.replace('-raw', '-processed'))

# print(df_angle.tail())
# print(df_look.tail())
# print(df_result.tail())

print(len(df_result.loc[df_result['look_25'] == True, 'timestamp']))
print(df_result.loc[df_result['look_25'] == True, 'timestamp'])
print(df_result)


##############################################################################################
####                                     PLOT                                             #### 
##############################################################################################

# Open Trifuns experiment data
# trifun_data_path = 'Trifun data/rrdv_results/filter_val_1_cm/rp_1_filter_distance_1_cm.csv'
trifun_data_path = 'Trifun data/rrdv_results/filter_val_1_cm/ra_1_filter_distance_1_cm.csv'
df_trifun = pd.read_csv(trifun_data_path, index_col=0)



print("Plot...")
# We create the Matplotlib figure object
fig, axs = plt.subplots(nrows= 1, ncols=1)
# Assign the first on to the balance plot
ax_pnl = axs


alpha = 0.6
## Plot the Balance 
look_12_data = df_result.loc[df_result['look_12'] == True, 'timestamp']
trifun_12_data = df_trifun.loc[df_trifun['Pair'] == 'robot_1_2', 'Timestamp [ms]']
ax_pnl.scatter(look_12_data,   [10]*len(look_12_data), label="robot 1-2", marker='s', c="xkcd:rose", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_12_data, [9.8]*len(trifun_12_data), c="xkcd:rose", lw=1, alpha=alpha)

look_13_data = df_result.loc[df_result['look_13'] == True, 'timestamp']
trifun_13_data = df_trifun.loc[df_trifun['Pair'] == 'robot_1_3', 'Timestamp [ms]']
ax_pnl.scatter(look_13_data, [9]*len(look_13_data), label="robot 1-3", marker='s', c="xkcd:pastel orange", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_13_data, [8.8]*len(trifun_13_data), c="xkcd:pastel orange", lw=1, alpha=alpha)

look_14_data = df_result.loc[df_result['look_14'] == True, 'timestamp']
trifun_14_data = df_trifun.loc[df_trifun['Pair'] == 'robot_1_4', 'Timestamp [ms]']
ax_pnl.scatter(look_14_data, [8]*len(look_14_data), label="robot 1-4", marker='s', c="xkcd:dull yellow", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_14_data, [7.8]*len(trifun_14_data), c="xkcd:dull yellow", lw=1, alpha=alpha)

look_15_data = df_result.loc[df_result['look_15'] == True, 'timestamp']
trifun_15_data = df_trifun.loc[df_trifun['Pair'] == 'robot_1_5', 'Timestamp [ms]']
ax_pnl.scatter(look_15_data, [7]*len(look_15_data), label="robot 1-5", marker='s', c="xkcd:dark lime", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_15_data, [6.8]*len(trifun_15_data), c="xkcd:dark lime", lw=1, alpha=alpha)

look_23_data = df_result.loc[df_result['look_23'] == True, 'timestamp']
trifun_23_data = df_trifun.loc[df_trifun['Pair'] == 'robot_2_3', 'Timestamp [ms]']
ax_pnl.scatter(look_23_data, [6]*len(look_23_data), label="robot 2-3", marker='s', c="xkcd:dark sea green", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_23_data, [5.8]*len(trifun_23_data), c="xkcd:dark sea green", lw=1, alpha=alpha)

look_24_data = df_result.loc[df_result['look_24'] == True, 'timestamp']
trifun_24_data = df_trifun.loc[df_trifun['Pair'] == 'robot_2_4', 'Timestamp [ms]']
ax_pnl.scatter(look_24_data, [5]*len(look_24_data), label="robot 2-4", marker='s', c="xkcd:mid blue", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_24_data, [4.8]*len(trifun_24_data), c="xkcd:mid blue", lw=1, alpha=alpha)

look_25_data = df_result.loc[df_result['look_25'] == True, 'timestamp']
trifun_25_data = df_trifun.loc[df_trifun['Pair'] == 'robot_2_5', 'Timestamp [ms]']
ax_pnl.scatter(look_25_data, [4]*len(look_25_data), label="robot 2-5", marker='s', c="xkcd:robin's egg", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_25_data, [3.8]*len(trifun_25_data), c="xkcd:robin's egg", lw=1, alpha=alpha)

look_34_data = df_result.loc[df_result['look_34'] == True, 'timestamp']
trifun_34_data = df_trifun.loc[df_trifun['Pair'] == 'robot_3_4', 'Timestamp [ms]']
ax_pnl.scatter(look_34_data, [3]*len(look_34_data), label="robot 3-4", marker='s', c="xkcd:carolina blue", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_34_data, [2.8]*len(trifun_34_data), c="xkcd:carolina blue", lw=1, alpha=alpha)

look_35_data = df_result.loc[df_result['look_35'] == True, 'timestamp']
trifun_35_data = df_trifun.loc[df_trifun['Pair'] == 'robot_3_5', 'Timestamp [ms]']
ax_pnl.scatter(look_35_data, [2]*len(look_35_data), label="robot 3-5", marker='s', c="xkcd:soft purple", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_35_data, [1.8]*len(trifun_35_data), c="xkcd:soft purple", lw=1, alpha=alpha)

look_45_data = df_result.loc[df_result['look_45'] == True, 'timestamp']
trifun_45_data = df_trifun.loc[df_trifun['Pair'] == 'robot_4_5', 'Timestamp [ms]']
ax_pnl.scatter(look_45_data, [1]*len(look_45_data), label="robot 4-5", marker='s', c="xkcd:perrywinkle", lw=1, alpha=alpha)
ax_pnl.scatter(trifun_45_data, [0.8]*len(trifun_45_data), c="xkcd:perrywinkle", lw=1, alpha=alpha)


# ax_pnl.set(xlim=(data['timestamp'].iloc[start], data['timestamp'].iloc[end1]))
ax_pnl.set_xlabel('Time [s]')
ax_pnl.set_ylabel('Incidence event')
# ax_pnl.set_title('Experiment 3 - Transmit Pairs - filter distance: 1cm - filter angle: 20deg')
ax_pnl.set_title('Experiment 1 - Transmit All - filter distance: 1cm - filter angle: 20deg')
ax_pnl.legend()
ax_pnl.grid()
plt.show()
