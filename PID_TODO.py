#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import math
import numpy as np
from crazyflie_driver.msg import GenericLogData

# PID controller gains
Kp = 0.3
Ki = 0.001
Kd = 0.1

duration_test = 10.0/50.0
freq = 50
duration = 10
count_coils = 4
class PIDController:
    def __init__(self, setpoint):
        self.setpoint = setpoint
        self.error = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.dt = 1#  (1.0/(freq-1)) * (duration/count_coils)
        self.memory =[0 for i in range(5)]
    def update(self, feedback_value, goal_value):
        output = 0 
        self.setpoint = goal_value
        self.error = self.setpoint - feedback_value

        # Compute the integral and derivative terms
        #self.integral += self.last_error*self.dt #TODO: Add code below
        self.integral = np.sum(self.memory)*self.dt
        #self.integral = np.clip(self.integral,-0.1, 0.1)
        #self.integral = np.clip(self.integral,-1.5, 1.5)
        print('i',self.integral)
        self.derivative = (self.error-self.last_error) / self.dt #TODO: Add code below
        
        print('d',self.integral)
        self.last_error = self.error #TODO: Add code below
        print('err',self.error)
        self.memory = self.memory[1:] + [self.last_error]
        # Compute the PID output
        output =  np.clip(Kp*self.error + Kd*self.derivative + Ki*self.integral , -0.03, 0.03) #TODO: Add code below
        print('o', output)
        return output

def circle_trajectory(freq, duration,r):
    x = list()
    y = list()
    t = list()
    for i in range(freq):
        t0 = 2 * math.pi * (1.0/(freq-1)) * duration * i
        x.append(r * math.cos(t0/duration))
        y.append(r * math.sin(t0/duration))
        t.append((1.0/(freq-1)) * duration * i)
    return (t,x,y)

def coil_trajectory(z0, freq, duration,count_coils):
    # Generate waypoints for a spiral trajectory
    #TODO: Add code below
    x = list()
    y = list()
    z = list() 
    k = 0
    for j in range (count_coils): 
        for i in range(freq-1):
            t0 = 2 * math.pi * (1.0/(freq-1)) * (duration) * i 
            x.append(0.8 * math.cos(t0/(duration)))
            y.append(0.8 * math.sin(t0/(duration)))
            z.append(z0 + k)
            k = k+0.05
    
    return (x,y,z)

def square_trajectory(length,height):
    # Generate waypoints for a square trajectory
    x = [0, length, length, 0, 0]
    y = [0, 0, length, length, 0]
    z = [height] * 5  # Maintain a constant height of 1.5 meters
    return (x, y, z)

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf1 = crazyflie.Crazyflie("cf1", "/cf1")

    cf1.setParam("commander/enHighLevel", 1)

    cf1.takeoff(targetHeight = 1.5, duration = 2.0)

    time.sleep(5.0)
    cf1.goTo([0.8,0,1.5], 0.0, duration = 3.0, relative = False)
    time.sleep(3.0)

    # (t , x , y,) = circle_trajectory(50 , 10 , 0.8)
    #(x , y, z) = square_trajectory(1 , 1.5)
    (x , y, z) = coil_trajectory(1.5,50 , 10,4)
    # Initialize the PID controller for x, y, and z
    pid_x = PIDController(x[0])
    pid_y = PIDController(y[0])
    pid_z = PIDController(z[0])

    for i in range(0, len(x)):
        # Get the current position of the drone
        current_pos = rospy.wait_for_message('/cf1/local_position', GenericLogData)
        while True:
            current_pos = rospy.wait_for_message('/cf1/local_position', GenericLogData)

            # Compute the PID output for x, y, and z
            output_x = pid_x.update(current_pos.values[0], x[i])
            output_y = pid_y.update(current_pos.values[1], y[i])
            output_z = pid_z.update(current_pos.values[2], z[i])

            # Set the goal position and duration for the goTo() function
            goal_pos = [x[i] + output_x, y[i] + output_y, z[i] + output_z]
            cf1.goTo(goal = goal_pos, yaw=0.0, duration = duration_test , relative = False)
            
            time.sleep(1*duration_test)
            if ((np.abs(current_pos.values[0]-x[i])<0.05) and (np.abs(current_pos.values[1]-y[i])<0.05)):
                break


    cf1.land(targetHeight = 0.0, duration = 7.0)

    time.sleep(3.0)
    cf1.stop()
 
