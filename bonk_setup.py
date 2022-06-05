# -*- coding: utf-8 -*-
"""
Created on Wed Jun  1 21:50:15 2022

@author: maxim
"""

import json
import cv2
import time
from utils.grabScreen import grabScreen
from utils.getKeys import getKeys, waitForKey
from utils.mouseFunctions import getMousePos


time.sleep(0.5)
print("put mouse on top left corner of the game window and press P")
waitForKey("P")
top_left = getMousePos()

time.sleep(1)
print("put mouse on down right corner of the game window and press P")
waitForKey("P")
down_right = getMousePos()

time.sleep(1)
print("put mouse on the bonk you want to track color and press P")
waitForKey("P")
mouseX, mouseY = getMousePos()
rgb = grabScreen([mouseX - 2, mouseY - 2,mouseX + 2, mouseY + 2])
hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
hsv_ball_color = [int(hsv[2][2][i]) for i in range(3)] 


setup_variables = {"game_window_coords" : [top_left[0], top_left[1], down_right[0], down_right[1]], 
            "hsv_ball_color" : hsv_ball_color}


print(setup_variables)

with open("setup_bonk.json", "w") as f:
    json.dump(setup_variables, f)