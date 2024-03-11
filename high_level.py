#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory
import math
import numpy as np

duration_test = 3*20.0/50.0

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
    theta = np.linspace(-count_coils * np.pi, count_coils * np.pi, freq)

    z = np.linspace(z0, z0+1, freq)
    r = 0.5

    x = r * np.cos(theta)
    y = r * np.sin(theta)
    t = 1.0/(freq-1) * duration
    return (t,x,y,z)

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

    cf1.goTo([0,0,1.5], 0.0, duration = 3.0, relative = False)
    time.sleep(3.0)


    # (t , x , y,) = circle_trajectory(50 , 10 , 0.8)
    #(t , x , y, z) = coil_trajectory(1.5, 50 , 20 , 3)

    (x , y, z) = square_trajectory(1 , 1.5)

    for i in range(len(x)):
          cf1.goTo(goal = [x[i], y[i], z[i]], yaw=0.0, duration = duration_test, relative = False)
          time.sleep(duration_test*2/3)


    cf1.land(targetHeight = 0.0, duration = 2.0)

    time.sleep(3.0)
    cf1.stop()
 
