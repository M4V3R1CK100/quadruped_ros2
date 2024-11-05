#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from math import pi, cos, sin, acos, atan2
from rclpy.node import Node
import time

global current_position
current_position = [0,0,0,0,0,0,0,0]

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

# def send_joints(list_joints):
#     global current_position
#     joints_states.position = list_joints
#     pub.publish(joints_states)
#     current_position = list_joints


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
    n = 2*foot_number -1
    joint_position_state = current_pos
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
        joint_position_state[n]=j[1]
        joint_position_state[n-1]=j[0]
        positions_plan.append(joint_position_state)
    
    return positions_plan

def dummy_traslation(x_trasl,z_trasl,angle ,current_pos):
    #falta revisar la rotacion
    d = 0.22 #Distancia del dummy entra pata y pata
    angle = - np.pi*angle/180
    z_rotation = (d)*sin(angle)
    x_rotation = -(d)*(cos(angle)) + d

    positions_plan = []
    joint_position_goal = [0,0,0,0,0,0,0,0]
    pre_position = [(0,0),(0,0),(0,0),(0,0)]
    final_position = [(0,0),(0,0),(0,0),(0,0)]
    for i in range(4):
        n = 2*i 
        base_new_arm = angles_new_arm(current_pos[n],current_pos[n+1])
        point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
        pre_position[i] = point_new_arm 

        if i >= 2:
            print("back")
            x_trasl = -x_trasl
        else:
            print("frontal")
            x_rotation = -x_rotation
            z_rotation = -z_rotation

            

        new_point = (point_new_arm[0]+x_trasl+x_rotation, point_new_arm[1]+z_trasl+z_rotation)
        final_position[i] = new_point

    steps = 10

    for step in range(steps+1):
        interpolated_position = [(0,0),(0,0),(0,0),(0,0)]
        #print("interpolacion")

        for a in range(4):
            m = 2*a
            x_interp = pre_position[a][0] + (final_position[a][0] - pre_position[a][0]) * (step / steps)
            z_interp = pre_position[a][1] + (final_position[a][1] - pre_position[a][1]) * (step / steps)
            #print(x_interp,z_interp)

            new_angles = IK(x_interp,z_interp,False)
            angles = angles_buddy_arm(new_angles[0],new_angles[1])
            joint_position_goal[m] = angles[0]
            joint_position_goal[m+1] = angles[1]
        
        positions_plan.append(joint_position_goal)
    
    return positions_plan

def dummy_rotation(angle,currrent_pos):

    print("Rotation", angle)
    d = 0.22 #Distancia del dummy entra pata y pata
    angle = - np.pi*angle/180
    z = (d)*sin(angle)
    x = -(d)*(cos(angle)) + d
    #z = 0.22
    #x = 0.22
    #FRONT
    #print("Dummy rotation")
    #first_position = FK(currrent_pos[0], currrent_pos[1],True)
    #print("first position: ", first_position)
    base_new_arm = angles_new_arm(currrent_pos[0],currrent_pos[1])
    point_new_arm = FK(base_new_arm[0],base_new_arm[1],False)
    new_point = (point_new_arm[0]-x, point_new_arm[1]-z)
    print("first position new arm: ",point_new_arm)
    print("new point: ",new_point)
    new_angles = IK(new_point[0],new_point[1],False)
    angles = angles_buddy_arm(new_angles[0],new_angles[1])
    #print("Angles new arm: ",new_angles )
    #print("Angles buddy arm: ",angles )

    #BACK
    base_new_arm_b = angles_new_arm(currrent_pos[0],currrent_pos[1])
    point_new_arm_b = FK(base_new_arm_b[0],base_new_arm_b[1],False)
    new_point_b = (point_new_arm_b[0]+x, point_new_arm_b[1]+z)
    print("first position new arm: ",point_new_arm_b)
    print("new point: ",new_point_b)
    new_angles_b = IK(new_point_b[0],new_point_b[1],False)
    angles_b = angles_buddy_arm(new_angles_b[0],new_angles_b[1])
    #print("Angles buddy arm: ",angles_b )

    joint_position_state=[angles[0],angles[1] ,angles[0],angles[1],angles_b[0],angles_b[1],angles_b[0],angles_b[1]] # stand up principal
    
    return joint_position_state

# def movement(speed):
#     #speed variara de -5 a 5
#     min_speed = 0
#     max_speed = 5
#     delay_min = 0.1  # en s
#     delay_max = 0.5  # en s
#     length = 0.15

#     if speed<0:
#         length = -length

#     # Suponiendo que `velocidad` es el valor actual de la velocidad
#     time_delay = delay_max - ((abs(speed) - min_speed) / (max_speed - min_speed)) * (delay_max - delay_min)
#     print(time_delay)
#     print(speed)
    
    
#     while speed!=0:
#         print("Traslation")
#         dummy_traslation(-length/2,0,time_delay)
#         rospy.sleep(0.05)
#         print("Front Gait")
#         if speed>0: #Cuando Avanza
#             gait(1,length,time_delay)
#             gait(2,length,time_delay)
            
#             dummy_traslation(4*length/2,0, time_delay)
#             rospy.sleep(1)

#             gait(3,length,time_delay)
#             gait(4,length,time_delay)
        
#         if speed<0: #Cuando retrocede
#             gait(4,length,time_delay)
#             gait(3,length,time_delay)
            
#             dummy_traslation(4*length/2,0, time_delay)
#             rospy.sleep(1)

#             gait(2,length,time_delay)
#             gait(1,length,time_delay)

#         dummy_traslation(-length/2,0, time_delay)

#     pass

if __name__ == "__main__":
    # Coloca aquí cualquier código de prueba o de ejecución que no desees que se ejecute al importar.
    print("Este mensaje solo aparece cuando ejecutas gait_functions directamente.")

