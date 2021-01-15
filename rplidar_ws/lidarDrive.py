#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive)


Options:
    -h --help          Show this screen.
"""
import os
import time
import numpy as np
import cv2
import threading

from picamera.array import PiRGBArray
from picamera import PiCamera
from docopt import docopt
from lidarInfo import *

import donkeycar as dk
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

from donkeycar.parts.camera import PiCamera

class MyCVController:
    '''
    CV based controller
    '''
    throttle = 0.2
    def run(self, cam_img):

        steering = 0
        throttle = 0
        left_ang_avg = 0
        left_dis_avg = 0
        right_ang_avg = 0
        right_dis_avg = 0
        cent_ang_avg = 0
        cent_dis_avg = 0
        final_ang_avg = 0;
        final_dis_avg = 0;
        recording = False
        frame = cam_img
        data = getScan()
        length = len(data[0])
       # print (data)
        data_counter = 1
        
        # Here we split the 180 degree zone in front of the car into 3 segments: right, left, and center
        
        for i in range (len(data[0])):
            if data[0][i] >= 90.0 and data[0][i] < 270.0:
                continue
            elif data[1][i] == 0:
                data[1][i] = 10000

            if data[1][i] != 0 and data[0][i] >= 30.0 and data[0][i] < 90.0:
                #left_ang_avg = left_ang_avg + data[0][i]
                left_ang_avg = 60
                left_dis_avg = left_dis_avg + data[1][i]
                data_counter = data_counter + 1
            elif data[1][i] != 0 and data[0][i] >= 270.0 and data[0][i] < 330.0:
                #right_ang_avg = right_ang_avg + data[0][i]
                right_ang_avg = 300
                right_dis_avg = right_dis_avg + data[1][i]
                data_counter = data_counter + 1
            elif data[1][i] != 0 and ((data[0][i] < 30.0 and data[0][i] > 0) or (data[0][i]$
               #cent_ang_avg = cent_ang_avg + data[0][i]
                cent_ang_avg = 0
                cent_dis_avg = cent_dis_avg + data[1][i]
                data_counter = data_counter + 1
            else:
                continue
                
        # Average all the data points scanned by the Lidar in each of the 3 segments

        #left_ang_avg = left_ang_avg / length
        right_dis_avg = right_dis_avg / data_counter
        #right_ang_avg = right_ang_avg / length
        left_dis_avg = left_dis_avg / data_counter
        #cent_ang_avg = cent_ang_avg / length
        cent_dis_avg = cent_dis_avg / data_counter
        
        
        # Radian/Cartesian conversions
        right_y = math.sin(math.pi/3) * right_dis_avg
        right_x = math.cos(math.pi/3) * right_dis_avg
        left_y = math.sin(math.pi/3) * left_dis_avg
        left_x = -1 * math.cos(math.pi/3) * left_dis_avg
        center_y = cent_dis_avg

        sum_x = round(left_x + right_x,2)
        sum_y = round(center_y - (left_y + right_y)/2,2)
        if sum_y < 100:
            sum_y = 100
        sum_angle = math.atan2(sum_x,sum_y) * 180.0 / math.pi
#       print (left_x, left_y, right_x, right_y, center_y, sum_x, sum_y, sum_angle)  --For debugging
        print (left_dis_avg, cent_dis_avg, right_dis_avg)

        if (sum_angle > -90 and sum_angle < 0):
            steering = (((sum_angle + 90) * 1) / 90) + 0
        else:
            steering = (((sum_angle - 0) * 1) / 90) - 1
        print (steering)

        if ((right_dis_avg < 600 and left_dis_avg < 600 and cent_dis_avg < 370) or (cent_di$
            throttle = 0
        else:
            throttle = 0.21
        print (throttle)
        return steering ,throttle, recording

# ----------Default code below, no need to edit below this line----------

def drive(cfg):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    # Initialize car
    V = dk.vehicle.Vehicle()

    # Camera
    camera = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    V.add(camera, outputs=['cam/image_array'], threaded=True)

    # Controller
    V.add(MyCVController(),
          inputs=['cam/image_array'],
          outputs=['steering', 'throttle', 'recording'])

    # Sombrero
    if cfg.HAVE_SOMBRERO:
        from donkeycar.parts.sombrero import Sombrero
        s = Sombrero()

    # Drive train setup

    from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PC$
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)
