#!/usr/bin/env python3

from queue import Queue
from spot_ros.spotmicro.spot_gym_env import spotGymEnv
import pickle
import numpy as np
import rospy
from itertools import cycle

rospy.init_node('curve', anonymous=True)

HEIGHT_FIELD = False
CONTACTS = True
MAX_EPISODES = 1_000_000
MAX_STEPS = 200
MAX_BUFFER = 1_000_000   
MAX_TRAIN_ITER = 200

env = spotGymEnv(render=True,
                    on_rack=False,
                    height_field=HEIGHT_FIELD,
                    draw_foot_path=False,
                    contacts=CONTACTS,)


state = env.reset()
hip_lim=[-0.548, 0.548]
shoulder_lim=[-2.17, 0.97]
leg_lim=[-0.1, 2.59]

# with open(r'/home/ros/custom_ai/src/spot_ros_v2/scripts/Left_curve_a_is_5.pkl', 'rb') as file:
with open(r'/home/ros/custom_ai/src/spot_ros_v2/scripts/curve_quad_seq_1432.pkl', 'rb') as file:
      
    # Call load method to deserialze
    seq1_data = pickle.load(file=file)

with open(r'/home/ros/custom_ai/src/spot_ros_v2/scripts/curve_quad_seq_3214.pkl', 'rb') as file:
      
    # Call load method to deserialze
    seq2_data = pickle.load(file=file)

# for i in range(10000):
k = 0
# k2 = 10


seq1_pool = cycle(seq1_data)
seq2_pool = cycle(seq2_data)
# pool = cycle(lst)

for _ep in range(MAX_EPISODES+1):
    
    for time_step, seq1, seq2 in zip(range(MAX_STEPS+1), seq1_pool, seq2_pool):
        action = np.array([0,seq1[1],seq1[2],0,seq2[1],seq2[2],0,seq2[1],seq2[2],0,seq1[1],seq1[2]])
        state, reward, done, info = env.step(action)
        # break
        if done:
            break
    if not rospy.is_shutdown():
        continue


    # break
# break



