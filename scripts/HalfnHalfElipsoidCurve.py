#!/usr/bin/env python3

from fileinput import filename
from defer import return_value
from spot_ros.Kinematics.LegKinematics import LegIK
import pickle
import numpy as np
import math
import matplotlib.pyplot as plt


ANIMATION = False
ANIMATION = True

class GenerateJointsViaCurve(object):
    def __init__(self, k = 0.00032, a = 3.7, b = 2.5, m = 0.5,) -> None:
        self.k = k
        self.a = a   #
        self.b = b   # ideal value for this spot is 0.045
        self.m = m   # ideal value for this spot is 0.01

        self.max_span = 0.1324  #max span of the curve in the pdf along the x axis
        self.storage = []
        self.x_max_limit = self.max_span/2
        self.half_step_per_sec = 20
        self.positive_x_values = []
        self.negative_x_values = []
        self.x_resolution = self.max_span/self.half_step_per_sec
        self.positive_start = 0
        self.negative_start = 0
        
        

            # self.x_values.append()

        # self.final_curve_x_values = []
        # self.final_curve_x_values.extend(self.positive_x_values)
        # self.final_curve_x_values.extend(self.negative_x_values)
        self.left_leg_joint = LegIK(legtype='LEFT', shoulder_length=0.055, elbow_length=0.1065, wrist_length=0.145)
        self.right_leg_joint = LegIK(legtype='RIGHT', shoulder_length=0.055, elbow_length=0.1065, wrist_length=0.145)

        # print(self.positive_x_values)
    
    def divide_x(self):
        for _ in range(int(self.half_step_per_sec/2)):
            self.positive_x_values.append(self.positive_start)
            self.negative_x_values.append(self.negative_start)
            self.positive_start += self.x_resolution
            self.negative_start -= self.x_resolution
        
        return self.positive_x_values, self.negative_x_values

    def generate_xyz_from_eqn(self, x_positive_val: list, x_negative_val: list, y):
        """
        graph is below
           2   |   1
        ---------------
           3   |   4
        
        :param: 
        :return:x:float
        """
        final_curve = []
        quad_1 = []
        quad_4 = []
        quad_3 = []
        quad_2 = []

        rev_x_negative_val = x_negative_val[::-1]
        rev_x_positive_val = x_positive_val[::-1]

        for px, nx, rpx, rnx in zip(x_positive_val, x_negative_val, rev_x_positive_val, rev_x_negative_val):
            first_quad_z = (self.b/self.a)*math.sqrt((self.k*(self.a**2) - px**2))
            fourth_quad_z = -(self.m/self.a)*math.sqrt((self.k*(self.a**2) - rpx**2)) # Reversed curve to be re -reversed
            third_quad_z = -(self.m/self.a)*math.sqrt((self.k*(self.a**2) - nx**2))
            second_quad_z = (self.b/self.a)*math.sqrt((self.k*(self.a**2) - rnx**2)) # Reversed curve to be re -reversed
            quad_1.append(np.array([px, y, first_quad_z])) 
            quad_4.append(np.array([rpx, y, fourth_quad_z])) 
            quad_3.append(np.array([nx, y, third_quad_z])) 
            quad_2.append(np.array([rnx, y, second_quad_z])) 
            
        # quad_4 = quad_4[::-1]
        # quad_2 = quad_2[::-1]

        # Default curve 
        # final_curve.extend(quad_1)
        # final_curve.extend(quad_4)
        # final_curve.extend(quad_3)
        # final_curve.extend(quad_2)


        final_curve.extend(quad_3)
        final_curve.extend(quad_2)
        final_curve.extend(quad_1)
        final_curve.extend(quad_4)


        return final_curve


    def rectifiy_my_curve(self, is_left: bool, right_y: float, left_y: float, curve: list, curve_zero_wrt_hip = 0.126):
        """
        :param: 
        x y z: value with respect to same legs hip link

        :return:
         numpy array of 3 values for joint angles
        J1, J2, J3
        """
        # if is_left:
        #     pass
        # else:
        for idx, xyz in enumerate(curve):
            xyz[1] = left_y if is_left else right_y
            xyz[2] = -(xyz[2] + curve_zero_wrt_hip)
            curve[idx] = xyz

        return curve
        # pass

    def generate_joint_from_xyz(self, is_left:bool, curve: list):
        """
        :param:
        is_left: leg on left side of spot or not 
        x y z: value with respect to same legs hip link

        :return:
         numpy array of 3 values for joint angles
        J1, J2, J3
        """
        curve_defined_by_joints = []
        if is_left:
            for k in curve:    
                joint_array = self.left_leg_joint.solve([k[0], k[1], k[2]])
                curve_defined_by_joints.append(joint_array)
            
        else:
            for k in curve:
                joint_array = self.right_leg_joint.solve([k[0], k[1], k[2]])
                curve_defined_by_joints.append(joint_array)

        
        return curve_defined_by_joints

    def save(self, file_name, data):
        """
        saves file in pickle and txt format
        """
        with open(file_name, 'wb') as file:
            # A new file will be created
            pickle.dump(data, file)
        
        with open(file_name+'.txt', 'w') as fp:
            for item in data:
                # write each item on a new line
                fp.write("%s\n" % item)
            print('Done')

        print("| Curve Saved! |")




if __name__ == "__main__":
    make = GenerateJointsViaCurve(b=2.5)
    px, nx = make.divide_x()


    val = make.generate_xyz_from_eqn(y=0.063, x_positive_val=px, x_negative_val=nx)
    rectified_curve = make.rectifiy_my_curve(is_left=True, right_y=0.063, left_y=-0.063, curve=val)

    ##########################
    rectified_curve = rectified_curve[::-1]
    ##########################

    joint_defined_curve = make.generate_joint_from_xyz(is_left=True, curve = rectified_curve)
    make.save('curve_quad_seq_3214.pkl', joint_defined_curve)


    # val = make.generate_xyz_from_eqn(y=-0.063, x_positive_val=px, x_negative_val=nx)
    # rectified_curve = make.rectifiy_my_curve(is_left=False, right_y=0.063, left_y=-0.063, curve=val)

    # ##########################
    # rectified_curve = rectified_curve[::-1]
    # ##########################

    # joint_defined_curve = make.generate_joint_from_xyz(is_left=False, curve = rectified_curve)
    # make.save('right_curve_quad_seq_1432.pkl', joint_defined_curve)



    if ANIMATION:
        x = []
        y = []

        for i in rectified_curve:
            x.append(i[0])
            y.append(i[2])
        
        plt.plot(x, y, color='green', linestyle='dashed', linewidth = 3,
            marker='o', markerfacecolor='blue', markersize=12)
        # print(val[-1])
        # naming the x axis
        plt.xlabel('x - axis')
        # naming the y axis
        plt.ylabel('y - axis')
        
        # giving a title to my graph
        plt.title('Some cool customizations!')
        
        # function to show the plot
        plt.show()
    
