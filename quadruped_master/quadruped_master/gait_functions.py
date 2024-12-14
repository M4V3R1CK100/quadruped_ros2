#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from math import pi, cos, sin, acos, atan2
from rclpy.node import Node
import time

def FK(theta1,theta2, base_on_dummy):
    #t1 = theta1*180/np.pi
    #t2 = theta2*180/np.pi
    if(base_on_dummy):
        theta1 = -theta1
        theta2 = -theta2
    l1 = 0.4
    l2 = 0.4

    x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
    z = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)
    return (x,z)

def IK(pos_x, pos_z, base_on_dummy):
    l1 = 0.4
    l2 = 0.4
    distancia = np.sqrt(pos_x**2 + pos_z**2)
    if distancia > (l1 + l2) or distancia < abs(l1 - l2):
        print("El punto objetivo está fuera del alcance del robot.")
    else:
        cosbeta = ( pos_x ** 2 + pos_z ** 2 - l1 ** 2 - l2 ** 2 ) / ( 2.0 * l1 * l2 )
        beta1 = acos( cosbeta )
        beta2 = -beta1
        A  , B = l1 + l2 * cosbeta, l2 * sin( beta1 )
        if(base_on_dummy):
            alpha1 = - atan2( pos_z * A + pos_x * B, pos_x * A - pos_z * B )
        else:
            alpha1 = atan2( pos_z * A - pos_x * B, pos_x * A + pos_z * B )
        #print("beta1:alpha1, beta2:alpha2")
        #print("(",alpha1,beta1,"),(",alpha2,beta2, ")")
    return alpha1,beta1

def angles_new_arm(theta1, theta2):
    beta = theta2
    alpha = 3.14 - (beta + theta1)
    return (alpha,beta)

def angles_buddy_arm(alpha, beta):
    theta2 = beta
    theta1 = 3.14 -(beta+alpha)
    return (theta1, theta2)

def plan_circle( center_x : float , center_y : float , r : float , theta_o : float  , theta_f : float , sentido_x : bool, sentido_y : bool, steeps):
    pos = []
    if (sentido_x and sentido_y):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (not(sentido_x) and sentido_y):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y + r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    elif (sentido_x and not(sentido_y)):
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x + r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))
    else:
        for theta in range(theta_o, theta_f + 1, steeps):
            position_z = center_y - r*math.sin(theta*math.pi/180)
            position_x = center_x - r*math.cos(theta*math.pi/180)
            pos.append((position_x,position_z))

    return pos

def gait(foot_number, length, current_pos): #Front foot? True or False
    positions_plan = []
    steps = 10
    length = length/2
    n = 2*foot_number
    joint_position_state = current_pos
    print(joint_position_state)
    #1rst leg gait
    first_position = FK(current_pos[n-1], current_pos[n],True)
    #print("first position: ",first_position)
    if length>=0:
        if foot_number>2: #Para las patas traseras
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, True, steps)
        else: #Patas delanteras
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180, False, True, steps)
    else:
        if foot_number>2: #Para las patas traseras
            posd = plan_circle(first_position[0]-length, first_position[1], length, 0,180, True, False, steps)
        else: #Patas delanteras
            print(first_position[0]+length, first_position[1])
            posd = plan_circle(first_position[0]+length, first_position[1], length, 0,180,  False, False, steps)

    for p in posd:
        #print("Position x,z: ",p)
        j = IK(p[0], p[1],True)
        #print("Joints: ",j)
        joint_position_state[n-1]=j[0]
        joint_position_state[n]=j[1]
        positions_plan.append(joint_position_state)

    return positions_plan

def dummy_traslation(x_trasl, z_trasl, angle ,current_pos, current_rot):
    print("current position: ", current_pos)

    #falta revisar la rotacion

    d = 0.22 #Distancia del dummy entra pata y pata
    angle = - np.pi*angle/180
    z_rotation = (d)*sin(angle)
    x_rotation = -(d)*(cos(angle)) + d

    trans_x = x_trasl*cos(current_rot) - z_trasl*sin(current_rot)
    trans_z = x_trasl*sin(current_rot) + z_trasl*cos(current_rot)

    positions_plan = []
    joint_position_goal = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0, 0.0]
    pre_position = [(0,0),(0,0),(0,0),(0,0)]
    final_position = [(0,0),(0,0),(0,0),(0,0)]

    for i in range(4):
        
        new_x_trasl = trans_x

        new_x_rot = x_rotation
        new_z_rot = z_rotation

        n = 2*i +1      #+1 es porque queremos evitar el 1er joint que es la camara
        base_new_arm = angles_new_arm(current_pos[n],current_pos[n+1])
        point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)

        pre_position[i] = point_new_arm 

        if i >= 2:
            new_x_trasl = -trans_x
        else:
            new_x_rot = -x_rotation
            new_z_rot = -z_rotation

        new_point = (point_new_arm[0]+ new_x_trasl+ new_x_rot, point_new_arm[1]+ trans_z + new_z_rot)
        final_position[i] = new_point

    steps = 10

    for step in range(steps+1):
        #print("interpolacion")

        for a in range(4):
            m = 2*a +1      #+1 es porque queremos evitar el 1er joint que es la camara
            x_interp = pre_position[a][0] + (final_position[a][0] - pre_position[a][0]) * (step / steps)
            z_interp = pre_position[a][1] + (final_position[a][1] - pre_position[a][1]) * (step / steps)

            new_angles = IK(x_interp,z_interp,False)
            angles = angles_buddy_arm(new_angles[0],new_angles[1])
            joint_position_goal[m] = angles[0]
            joint_position_goal[m+1] = angles[1]

        positions_plan.append(joint_position_goal.copy())
    
    return positions_plan

if __name__ == "__main__":
    # Coloca aquí cualquier código de prueba o de ejecución que no desees que se ejecute al importar.
    print("Este mensaje solo aparece cuando ejecutas gait_functions directamente.")

