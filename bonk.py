# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 01:29:27 2021

@author: maxim
"""
import cv2
import time
import math
import json
import numpy as np
from os.path import exists
from collections import deque
from utils.mouseFunctions import getMousePos, isLeftMousePressed, isRightMousePressed
from utils.directKeys import PressKey, ReleaseKey, D_W, D_A, D_Z, D_S, D_Q, D_D, D_X
from utils.grabScreen import grabScreen
from utils.getKeys import getKeys


class Bonk():
    def __init__(self):
        
       if exists("setup_bonk.json"):
           with open("setup_bonk.json") as f:
               setup_variables = json.load(f)
           
           print("Loaded : ", setup_variables)
           self.game_window_coords = setup_variables['game_window_coords']
           self.hsv_ball_color = np.array(setup_variables['hsv_ball_color'])
          
       else:    
           self.game_window_coords = [180,384,760,780]
           self.hsv_ball_color = np.array([7, 110, 255])
       
       self.tracking_mode = 'mouse' #'numpad' or 'mouse'
       #control_keys :     [Up, Down, Left, Right, heavy]
       self.control_keys = [D_W, D_S, D_A, D_D, D_X]
       self.track_offscreen = True
       self.maximize_speed = False #does not work well on all jumps
       
       self.preview_width = self.game_window_coords[2] - self.game_window_coords[0]
       self.preview_height = self.game_window_coords[3] - self.game_window_coords[1]
       self.preview_frame = 0 * np.zeros((self.preview_height, self.preview_width, 3), np.uint8)
        
       self.avg_len = 5
       self.position_list = deque(maxlen=self.avg_len)
       self.velocity_list = deque(maxlen=self.avg_len)
       self.acceleration_list = deque(maxlen=self.avg_len)
       self.error_list = deque(maxlen=self.avg_len)
       self.target_distance = 1
       
       self.ball_radius = 10
       self.ball_pos = (-1,-1)
       self.target_pos = (-1,-1)
       self.target_list = [(0,0) for i in range(10)] 
       self.ball_velocity = (0,0)
       self.ball_direction = (1,1)
       self.ball_acceleration = (0,0)
       self.gravity_strength = 0.1
       self.keys_strength = 0.1
       self.last_controls = [0,0,0,0,0]
       self.controls = [0,0,0,0,0]
       self.controls_accel = (0,0)
       
       self.future_trajectory = []
       self.low_future_trajectory = []
       self.high_future_trajectory = []
       
       self.frame_rate = 30
       self.last_time = time.time()
       
        

    def getBall(self):
        frame = grabScreen(self.game_window_coords)
        BGR_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        HSV_frame = cv2.cvtColor(BGR_frame, cv2.COLOR_BGR2HSV)
      
        mask_range = np.array([5,5,5])
        mask_lower = np.clip(self.hsv_ball_color - mask_range, 0, 255)
        mask_upper =  np.clip(self.hsv_ball_color + mask_range, 0, 255)
        mask = cv2.inRange(HSV_frame, np.array(mask_lower, np.uint8) , np.array(mask_upper, np.uint8))  
        
        M = cv2.moments(mask)
        if M["m00"] != 0:
            self.is_ball = True
        else:
            self.is_ball = False
            
        if self.is_ball:
            ball_x = round(M["m10"] / M["m00"], 2)
            ball_y = round(M["m01"] / M["m00"], 2)
            
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            new_radius = cv2.minEnclosingCircle(c)[1]
            
            self.ball_radius = self.ball_radius * 0.9 + new_radius * 0.1
            self.ball_pos = (ball_x, ball_y)
            
        self.preview_frame = BGR_frame
        
        return self.is_ball, self.ball_pos, self.ball_radius  
    
    def drawArrow(self, pos, x, y, color):
        cv2.arrowedLine(self.preview_frame, 
                        (int(pos[0]), int(pos[1])),
                        (int(pos[0]+x), int(pos[1]+y)),
                        color, 4)
    
    
    def showPreview(self, show_movements=True, show_trajectory=True, show_controls=True):
        control_arrow_length = 20
        velocity_arrow_length = 0.2
        accel_arrow_length = 0
        
        nb_trajectory_points = 30
        traj_point_size = max(2, int(self.ball_radius/3))
        
        yellow = (0,255,255)
        green = (20,255,50)
        blue = (200,40,60) 
        pink = (255,0,255) 
        red = (0,0,255) 
        
        Pos = self.ball_pos
        Vit = self.ball_velocity
        Acc = self.ball_acceleration
        
        if show_movements:
            self.drawArrow(Pos, velocity_arrow_length*Vit[0], velocity_arrow_length*Vit[1], green)
            self.drawArrow(Pos, accel_arrow_length*Acc[0], accel_arrow_length*Acc[1], blue)
        
        if show_controls and self.controls[:4] != [0,0,0,0]:
            if self.controls[0] == 1:
                self.drawArrow(Pos, 0, -control_arrow_length, yellow)
            if self.controls[1] == 1:
                self.drawArrow(Pos, 0, control_arrow_length, yellow)
            if self.controls[2] == 1:
                self.drawArrow(Pos, -control_arrow_length, 0, yellow)
            if self.controls[3] == 1:
                self.drawArrow(Pos, control_arrow_length, 0, yellow)
                
        if show_trajectory and len(self.future_trajectory) >= nb_trajectory_points:
            for i in range(0, nb_trajectory_points, 2):
                cv2.circle(self.preview_frame, self.future_trajectory[i], traj_point_size, pink, -1)
                cv2.circle(self.preview_frame, self.low_future_trajectory[i], traj_point_size, blue, -1)
                cv2.circle(self.preview_frame, self.high_future_trajectory[i], traj_point_size, red, -1)
                
        if abs(self.target_attainability) <= 1:
            cv2.circle(self.preview_frame, self.target_pos, 4, yellow, -1)
        else:
            cv2.circle(self.preview_frame, self.target_pos, 4, red, -1)
        
        
        cv2.imshow("preview window", self.preview_frame)
        cv2.waitKey(10)             
                
        
    def getFuturePos(self, current_pos, time, key_accel=(0,0)):
        x, y = current_pos
        vx, vy = self.ball_velocity
        Gy = self.gravity_strength
        ax, ay = key_accel
        t = time
        
        new_x = int(x + vx*t + (1/2)*ax*t*t)
        new_y = int(y + vy*t + (1/2)*(Gy+ay)*t*t)
    
        return (new_x, new_y)
        
    def getErrorFromTarget(self, target_pos):      
        x, y = self.ball_pos
        vx, vy = self.ball_velocity
        Gy = self.gravity_strength
        t_x, t_y = target_pos
        
        time_x_reached, time_y_reached = -1, -1
        error_x, error_y = -1, -1
        
        time_x_reached = (t_x - x) / (vx + 1e-2)
        time_x_reached = max(0, time_x_reached)
        error_y = t_y - self.getFuturePos(self.ball_pos, time_x_reached)[1]
        
        try:        
            time_y_reached = (-1*vy + math.sqrt(vy*vy - 2*Gy*(y-t_y)))/Gy
        except ValueError:
            pass
        
        time_y_reached = max(0, time_y_reached)
        error_x = t_x - self.getFuturePos(self.ball_pos, time_y_reached)[0]
        
        self.target_distance = self.getDist(self.ball_pos, self.target_pos)
        
        self.error_list.append((error_x, error_y))
        
        return (error_x, error_y)
    
    def getControls(self, error):
        error_x, error_y = error[1], error[0]
        kp = 1
        kd = 0
        threshold = 0
        
        D_error_x, D_error_y = 0, 0
        if len(self.error_list) == self.avg_len:
            D_error_x = self.error_list[-1][0] - self.error_list[-2][0]
            D_error_y = self.error_list[-1][1] - self.error_list[-2][1]
        
        PD_error_x = kp * error_x + kd * D_error_x
        PD_error_y = kp * error_y + kd * D_error_y
        
        controls = [0,0,0,0,0]
        
        if abs(PD_error_y) > threshold:
            if PD_error_y >= 0:
                controls[3] = 1
            else:
                controls[2] = 1
            
        if abs(PD_error_x) > threshold:
            if PD_error_x >= 0:
                controls[1] = 1
            else:
                controls[0] = 1
                    
                    
        if self.maximize_speed and abs(self.target_attainability) < 0.5:
            controls = [0,0,0,0,0]
            target_direction = np.sign(np.subtract(self.target_pos, self.ball_pos))
            
            if target_direction[0] == 1:
                controls[3] = 1
                
            elif target_direction[0] == -1:
                controls[2] = 1
                
            if target_direction[1] == 1:
                controls[0] = 1
                
            elif target_direction[1] == -1:
                controls[1] = 1
        
        return controls
    
    def pressControlIndex(self, i):
        PressKey(self.control_keys[i])
    
    def releaseControlIndex(self, i):
        ReleaseKey(self.control_keys[i])    
        
    def applyControls(self, controls):
        self.controls = controls
        for i in range(len(self.controls)):
            if self.last_controls[i] == 0 and self.controls[i] > 0:
                self.pressControlIndex(i)
                
            elif self.last_controls[i] > 0 and self.controls[i] == 0:
                self.releaseControlIndex(i)
        
        self.controls_accel = self.getControlsAccel(self.controls)
        self.last_controls = self.controls
        
    
    def getControlsAccel(self, controls):
        accel_x = 0
        accel_y = 0
        if self.controls[0] > 0:
            accel_y = -self.keys_strength
            
        elif self.controls[1] > 0:
            accel_y = self.keys_strength
            
        if self.controls[2] > 0:
            accel_x = -self.keys_strength

        elif self.controls[3] > 0:
           accel_x = self.keys_strength
        
        self.controls_accel = (accel_x, accel_y)
        return self.controls_accel
    
    
    def updateMovementInfo(self, pos):
        self.position_list.append(pos)
        if len(self.position_list) == self.avg_len:
            avg_velocity = (np.array(self.position_list[-1]) - np.array(self.position_list[-self.avg_len])) / (self.avg_len-1) * self.frame_rate
            self.velocity_list.append(tuple(avg_velocity))
        
        if len(self.velocity_list) == self.avg_len:
            avg_accel = (np.array(self.velocity_list[-1]) - np.array(self.velocity_list[-self.avg_len])) / (self.avg_len-1) * self.frame_rate
            self.acceleration_list.append(tuple(avg_accel))
            
            self.ball_velocity = self.velocity_list[-1]
            self.ball_acceleration = self.acceleration_list[-1]
            
            self.gravity_strength = 25.46 * self.ball_radius
            self.keys_strength = 15.65 * self.ball_radius
            self.ball_direction = tuple(np.sign(self.ball_velocity))
            
        self.updateTrajInfo()
        
        
    def tickPhysics(self):
        self.ball_acceleration = np.add((0, self.gravity_strength), np.multiply(self.controls_accel, ( 1, 1)))
        self.ball_velocity = tuple(np.add(self.ball_velocity, np.multiply(self.ball_acceleration, 0.65/self.frame_rate)))
        self.ball_pos = tuple(np.add(self.ball_pos, np.multiply(self.ball_velocity, 1/self.frame_rate)))
        
        self.position_list.append(self.ball_pos)
        self.velocity_list.append(self.ball_velocity)
        self.acceleration_list.append(self.ball_acceleration)
        
        self.updateTrajInfo()
        
            
    def updateTrajInfo(self):
        time_interval = 0.1
        nb_traj_points = int(self.frame_rate * 4)
        self.future_trajectory = []
        self.low_future_trajectory = []
        self.high_future_trajectory = []
        
        neutral_traj_accel = (0, 0)
        low_traj_accel = (-1*self.ball_direction[0]*self.keys_strength, self.keys_strength)
        high_traj_accel = (self.ball_direction[0]*self.keys_strength, -1*self.keys_strength) 
        
        for i in range(nb_traj_points):
            self.future_trajectory.append(self.getFuturePos(self.ball_pos, i*time_interval, neutral_traj_accel))
            self.low_future_trajectory.append(self.getFuturePos(self.ball_pos, i*time_interval, low_traj_accel))
            self.high_future_trajectory.append(self.getFuturePos(self.ball_pos, i*time_interval, high_traj_accel))
        
    
    def getClosestPointToTarget(self, trajectory, target):
        closest_point = trajectory[0]
        closest_distance = 4000
        for point in trajectory:
            dist = self.getDist(target, point)
            if dist < closest_distance:
                closest_distance = dist
                closest_point = point
                
        return closest_point
    
    
    def getAngleDiff(self, origin, point1, point2):
        vec1 = np.subtract(point1, origin)
        vec2 = np.subtract(point2, origin)
        
        dot = np.inner(vec1, vec2)
        norm = np.linalg.norm(vec1) * np.linalg.norm(vec2)
        
        angle = np.arccos(np.clip(dot/norm, -1.0, 1.0))
        
        return angle
    
    
    def getDist(self, point1, point2):
        return np.linalg.norm(np.subtract(point1, point2))
    
    
    def getTargetAttainability(self, target_pos):
        "return 0 if the base trajectory is on target, 1 if the high trajectory is on target and -1 if its the low trajectory"
        base_point = self.getClosestPointToTarget(self.future_trajectory, target_pos)
        high_point = self.getClosestPointToTarget(self.high_future_trajectory, target_pos)
        low_point = self.getClosestPointToTarget(self.low_future_trajectory, target_pos)
        
        angle_base_target = self.getAngleDiff(self.ball_pos, base_point, target_pos)
        angle_base_high = self.getAngleDiff(self.ball_pos, base_point, high_point)
        angle_base_low = self.getAngleDiff(self.ball_pos, base_point, low_point)
        
        if angle_base_target >= 0:
            attainability = angle_base_target / (angle_base_high + 1e-5)
            
        else:
            attainability = -1 * angle_base_target / (angle_base_low + 1e-5)
            
        attainability = np.clip((attainability - 0.5) * 2, -2 , 2)  
            
        self.target_attainability = attainability
        
        return attainability
        
        
    def updateFramerate(self):
        new_time = time.time()
        current_FPS = 1/(new_time-self.last_time)
        
        self.frame_rate = 0.8 * self.frame_rate + 0.2 * current_FPS
        self.last_time = new_time
        
    
    def getNumpadKey(self):
        numpad_key = -1
        keys = getKeys()
        for key in keys:
            if 48 <= ord(key) <= 57:
                numpad_key = ord(key) - 48
        
        return numpad_key
        
    def updateTargetNumpad(self):
        follow_target = False 
        numpad_key = self.getNumpadKey()
        if numpad_key != -1:
            if isLeftMousePressed():
                self.target_list[numpad_key] = self.getRelativeMousePos()
                
            else:
                self.target_pos = self.target_list[numpad_key]
                follow_target = True
            
        return follow_target

    def getRelativeMousePos(self):
        return tuple(np.subtract(getMousePos(), (self.game_window_coords[0], self.game_window_coords[1])))
        
    def updateTargetMouse(self):
        if isLeftMousePressed():
            self.target_pos = self.getRelativeMousePos()
            return True
        else:
            return False
        
        
    def mainLoop(self):
        while True:
            is_ball, pos, radius = bonk.getBall()
            if is_ball:
                bonk.updateMovementInfo(pos)
                
            elif bonk.track_offscreen:
                bonk.tickPhysics()
                is_ball = True
            
            if is_ball:
                #print("fps : ", bonk.frame_rate)
                bonk.getTargetAttainability(bonk.target_pos)
                bonk.showPreview()
                
                if self.tracking_mode == 'mouse':
                    follow_target = bonk.updateTargetMouse()
                elif self.tracking_mode == 'numpad':
                    follow_target = bonk.updateTargetNumpad()
                
                if follow_target:
                    traj_error = bonk.getErrorFromTarget(bonk.target_pos)
                    controls = bonk.getControls(traj_error)
                    bonk.applyControls(controls)
                
                else:
                    bonk.applyControls([0,0,0,0,0])
                
            else:
                time.sleep(1/bonk.frame_rate)
            
            keys = getKeys()
            if 'P' in keys:
                cv2.destroyAllWindows()
                break
            
            bonk.updateFramerate()
        
    

bonk = Bonk()

bonk.mainLoop()


        
        
         

       
