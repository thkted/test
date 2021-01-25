#!/usr/bin/env python3
# license removed for brevity
import numpy as np

class decision_making:
    def __init__(self,Lidar_info, Object_left_idx,Object_front_idx,Object_right_idx, No_Object,Current_lane, End_lane, Ego_x, Ego_y, Ego_heading,Ego_vx,Ego_vy,safe_dis):
        
        self.Lidar_info = Lidar_info
        self.Object_left_idx = Object_left_idx
        self.Object_front_idx = Object_front_idx
        self.Object_right_idx = Object_right_idx
        self.No_Object = No_Object
        self.Current_lane = Current_lane
        self.End_lane = End_lane
        self.Ego_x = Ego_x
        self.Ego_y = Ego_y
        self.Ego_heading = Ego_heading
        self.Ego_vx = Ego_vx
        self.Ego_vy = Ego_vy
        self.safe_dis = safe_dis

    def AEBrake(self):

        TTC = 0
        AEB = 0
        if self.No_Object == 1:
            AEB = 0

        else:
            if np.size(self.Object_front_idx) >= 1:
                front = self.Lidar_info[self.Object_front_idx]
                front = front.astype('float64')
                front_dis = np.sqrt(np.square(front[:,0] - self.Ego_x) + np.square(front[:,1] - self.Ego_y))
                TTC = front_dis / front[:,-1]

                if np.size( np.where( (front_dis <= self.safe_dis) & (front_dis > 1) )[0] ) >= 1: # 1 < y < safe_dis
                    if np.size( np.where(front[:,-1] == 0)[0] ) >= 1: #relv == 0 (not convergence)
                        if np.size( np.where(front_dis < 5)[0] ) >= 1:
                            AEB = 1
                    else:
                        if np.size( np.where(TTC < 2)[0] ) >= 1:
                            AEB = 1


        return AEB

    def LC_decision(self,LC_req, pos_lane,LC_ing):
        LC2GP = 0, Ovt = 0

        front = self.Lidar_info[self.Object_front_idx]
        front = front.astype('float64')
        left = self.Lidar_info[self.Object_left_idx]
        left = left.astype('float64')
        right = self.Lidar_info[self.Object_right_idx]
        right = right.astype('float64')

        front_dis = np.sqrt(np.square(front[:,0] - self.Ego_x) + np.square(front[:,1] - self.Ego_y))        

        if np.size(self.Object_front_idx) >= 1:
            
            if pos_lane != 0:
                ## overtaking dm
                if np.size( np.where( (front[:,-1] < -3) & (front_dis > self.safe_dis) )[0]) >= 1:
                    Ovt = 1

                elif np.size( np.where( (front[:,-1] < -3) & (front_dis < self.safe_dis) )[0]) >= 1:
                    if pos_lane == -1:
                        if np.size( np.where( (left[:,1] < -10) & (left[:,1] > 30) )[0]) >= 1:
                            Ovt = 1
                            LC2GP = -1

                    else:
                        if np.size( np.where( (right[:,1] < -10) & (right[:,1] > 30) )[0]) >= 1: 
                            Ovt = 1
                            LC2GP = 1

                #GP LC request
                elif LC_req != 0:
                    if LC_req == -1 & (pos_lane == -1 or pos_lane == 2):
                        if np.size( np.where( (left[:,1] < -10) & (left[:,1] > 25) )[0]) >= 1:
                            LC2GP = - 1

                    elif LC_req == 1 & (pos_lane == 1 or pos_lane == 2):
                        if np.size( np.where( (right[:,1] < -10) & (right[:,1] > 25) )[0]) >= 1: 
                            LC2GP = 1

        else:
            if LC_req == -1 & (pos_lane == -1 or pos_lane == 2):
                if np.size( np.where( (left[:,1] < -10) & (left[:,1] > 25) )[0]) >= 1:
                    LC2GP = - 1

            elif LC_req == 1 & (pos_lane == 1 or pos_lane == 2):
                 if np.size( np.where( (right[:,1] < -10) & (right[:,1] > 25) )[0]) >= 1: 
                    LC2GP = 1

        if LC_ing == 1:
            LC2GP = 0


        return LC2GP, Ovt

    def DM_main(self,LC_req, pos_lane, LC_ing):
        
        AEB = AEBrake()
        LC2GP, Ovt = LC_decision(LC_req,pos_lane,LC_ing)

        return LC2GP, Ovt



if __name__ == '__main__':

    Current_lane = 10
    End_lane = 70

    A = GPP.Pathplanning(Current_lane,End_lane)

    B = decision_making()

    Ego_x = 1000
    Ego_y = 1200
    Ego_vy = 30
    Ego_heading = 1


    Start_path, End_path = A.Start_End_Path()
    Final_Global_path, GP, GP_cost = A.Global_path(Start_path, End_path)
    Next_path,nNext_path = A.local_path(GP, GP_cost, Start_path)
    local_x, local_y, local_dis = A.local_x_y_dis( Next_path, nNext_path,Ego_heading)
    L_M_R_info = A.lane_info()

    # Lidar_from_ROS=np.random.randint(-5,5,size=60)
    Lidar_from_ROS = np.zeros((10,6))

    Lidar_info,left_idx,current_idx,right_idx = B.Lidar_information(Lidar_from_ROS)

    AEB = B.AEBrake(Lidar_info, current_idx,Ego_vy)

    LC_L_GP, LC_R_GP, straight_GP,lanes_in_path, possible_lanes_no_zero, current_lane_idx = B.lane_change_for_GP(path_lane_matching, GP_cost, L_M_R_info, Current_lane, GP)

    LC_L_Obs,LC_R_Obs,straight_Obs = B.lane_change_for_Obstacles(Lidar_info, left_idx, current_idx,right_idx,Ego_vy,Current_lane,GP, possible_lanes_no_zero,current_lane_idx)

    Final_LC_L,Final_LC_R,straight_GP,straight_Obs = B.LC_decision(LC_L_GP, LC_R_GP, straight_GP,LC_L_Obs, LC_R_Obs, straight_Obs)

    Final_local_x, Final_local_y = B.local_path(Final_LC_L, Final_LC_R,Ego_x,Ego_y,local_x,local_y)

