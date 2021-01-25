#!/usr/bin/env python3
# license removed for brevity
import os
import sys
sys.path.append(os.getcwd())
import rospy
import datetime
import Global_path.Global_path as GPP
import Decision_Making.DM as DM
import numpy as np
from std_msgs.msg import *
from save_interface.msg import array, Detect2Dm, Object, debug_raw_array
from hellocm_msgs.msg import TrafficLight_Out, VehicleInfo_Out,Singallight_In

def Lidar_processing(Lidar_from_ROS):

    left_idx ,front_idx ,right_idx = [],[],[]
    No_Object = 0

    test = np.array([])
    for i in Lidar_from_ROS:
        test = np.append(test,[i.x, i.y, i.w, i.h, i.rel_v_x, i.rel_v_y])

    test = np.reshape(test,(10,6))

    No_Object = 0
    x_range_1 = 4.5
    x_range_2 = 1.5

    Lidar_info = test

    Lidar_info_x = Lidar_info[:,0]
    No_object_idx = np.where(Lidar_info_x==9999)[0] # if x==9999 -> No Object
    Lidar_info = np.delete(Lidar_info,No_object_idx,axis=0)

    if Lidar_info.size == 0:
        No_Object = 1
    else:
        Object_x = Lidar_info[:,0]
        Object_y = Lidar_info[:,1]
        left_idx =  np.where( (Object_x <= -1 * x_range_2) & (Object_x >= -1 * x_range_1))[0]
        front_idx = np.where( (Object_x < x_range_2) & (Object_x > -1 * x_range_2) & (Object_y > 0))[0]
        right_idx = np.where( (Object_x >= x_range_2) & (Object_x <= x_range_1))[0]

        if np.size(left_idx) >= 1:
            pass
        else:
            left_idx = []

        if np.size(front_idx) >= 1:
            pass
        else:
            front_idx = []

        if np.size(right_idx) >= 1:
            pass
        else:
            right_idx = []

    return Lidar_info, left_idx ,front_idx ,right_idx , No_Object

class main:
    def __init__(self):
        self.Ego_x = 0
        self.Ego_y = 0
        self.Ego_heading = 0
        self.Current_lane = 0
        self.End_lane = 0
        self.Lidar_from_ROS = 0
        self.light_state = 0
        self.Ego_vx = 0
        self.Ego_vy = 0

        rospy.Subscriber("/Ego_location", Float64MultiArray, self.callback_Localization)
        rospy.Subscriber('/Detect2DM', Detect2Dm, self.callback_Lidar)
        rospy.Subscriber('/TrfLight_Out', TrafficLight_Out, self.callback_Traffic_Light)
        rospy.Subscriber('/VehicleInfo_Out', VehicleInfo_Out, self.callback_Ego_velocity)


    def callback_Localization(self, msg):
        self.Ego_x = msg.data[0]
        self.Ego_y = msg.data[1]
        self.Ego_heading = msg.data[2]
        self.Current_lane = msg.data[3]
        self.End_lane = msg.data[6]

    def callback_Lidar(self, msg):
        self.Lidar_from_ROS = msg.objects

    def callback_Traffic_Light(self, msg):
        self.light_state = msg.light_state

    def callback_Ego_velocity(self, msg):
        self.Ego_vx = msg.vx
        self.Ego_vy = msg.vy

if __name__ == '__main__':
    
    rospy.init_node('dm', anonymous=True)
    ros = main()

    while not rospy.is_shutdown(): 

        Ego_x = ros.Ego_x
        Ego_y = ros.Ego_y
        Ego_heading = ros.Ego_heading
        Current_lane = int(ros.Current_lane)
        End_lane = int(ros.End_lane)
        Lidar_from_ROS = ros.Lidar_from_ROS
        Ego_vx = ros.Ego_vx
        Ego_vy = ros.Ego_vy
        safe_dis = Ego_vy - 15
              
        if Ego_x == 0 or End_lane == 0 or np.size(Lidar_from_ROS) != 10:
            pass

        else:
            
            A = GPP.Pathplanning(Current_lane, End_lane, Ego_x, Ego_y, Ego_heading, path_lane_matching, L_M_R, SaveArrays, path_cost, lane,IC_path,IC_lane_matching,IC,lane_to_IC,IC_to_lane)
            ##GP
            #LC_req, pos_lane, LC_ing = A.~~~
            
            #Lidar data processing
            Lidar_info, Object_left_idx,Object_current_idx,Object_right_idx, No_Object = Lidar_processing(Lidar_from_ROS)

            #DM Start
            B = DM.decision_making(Lidar_info, Object_left_idx,Object_front_idx,Object_right_idx, No_Object,Current_lane, End_lane, Ego_x, Ego_y, Ego_heading,Ego_vx,Ego_vy,safe_dis)

            AEB = B.AEBrake(Ego_vy)

            if AEB == 1:
                print('AEB Signal: ', AEB)

            else:
                LC2GP, Ovt  = B.DM_main(LC_req, pos_lane, LC_ing)












            if LC2GP == -1:
                signal_light = 1

            elif LC2GP == 1
                signal_light = -1

            # elif straight_GP == 1 or straight_Obs == 1:
            #     if current_lane_idx == 0:
            #         signal_light = 1
            #     elif len(lanes_in_path) - 1 == current_lane_idx:
            #         signal_light = -1
            #     else:
            #         signal_light = 0
          
            if AEB == 1:
                signal_light = 0

            pub_sig = Singallight_In()
            pub_sig.Singallight = signal_light
            pub_sig.hazard = AEB
            
            pub_siglight = rospy.Publisher('Signallight', Singallight_In, queue_size=10)
            pub_Ovt = rospy.Publisher('Ovt', Float64MultiArray, queue_size=10)            
            # pub_local_path_y = rospy.Publisher('local_path_y', Float64MultiArray, queue_size=10)
            # pub_local_path_s = rospy.Publisher('local_path_s', Float64MultiArray, queue_size=10)
            # pub_local_path_curvature = rospy.Publisher('local_path_curvature', Float64MultiArray, queue_size=10)
            pub_AEB_signal = rospy.Publisher('AEB_signal', Int8, queue_size=10)
            
            rospy.init_node('dm', anonymous=True)
            rate = rospy.Rate(10) # 10hz
            
            send_Ovt = Float64MultiArray()
            send_Ovt.data = Ovt

            # send_local_path_y = Float64MultiArray()
            # send_local_path_y.data = Final_y

            # send_local_path_s = Float64MultiArray()
            # send_local_path_s.data = local_dis

            # send_local_path_curvature = Float64MultiArray()
            # send_local_path_curvature.data = local_curvature

            send_AEB_signal = Int8()
            send_AEB_signal.data = AEB

            # # rospy.loginfo(data_to_send)
            # step += 1
            # print("Pub: Localization Step {} Time {}".format(step, datetime.datetime.now()))
            pub_siglight.publish(pub_sig)
            pub_Ovt.publish(send_Ovt)
            # pub_local_path_y.publish(send_local_path_y)
            # pub_local_path_s.publish(send_local_path_s)
            # pub_local_path_curvature.publish(send_local_path_curvature)
            pub_AEB_signal.publish(send_AEB_signal)

            rate.sleep()