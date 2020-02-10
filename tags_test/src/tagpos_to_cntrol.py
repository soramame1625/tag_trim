#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Vector3
from apriltag_ros.msg import AprilTagDetectionArray
from data_prepare.msg import msg_Controldata_IBIS

"""
THR_MAX = 1100
THR_MIN = 1600
"""

rospy.init_node('ttc')
pub = rospy.Publisher('data_IBIS1',msg_Controldata_IBIS,queue_size=10)
msg = msg_Controldata_IBIS()

class tag_instance():
    def __init__(self):
        self.id = 0
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        #def setdata


class PID():
    def __init__(self):
        self.t_Kp = 0.002
        self.t_Ki = 0.0025
        self.t_Kd = 1
        
        self.r_Kp = 10
        self.r_Ki = 0
        self.r_Kd = 1

        self.p_Kp = 10
        self.p_Ki = 0
        self.p_Kd = 1

        self.y_Kp = 2
        self.y_Ki = 0#.05
        self.y_Kd = 1

        self.Ml = 0
        self.Dl1 = 0
        self.Dl2 = 0 
        self.out_l = 0
        self.tag1_l = 0
        self.gap_l = 0
        self.gap_l2 = 0
        self.pM = 0
        self.iM = 0
        self.dM = 0

        self.fg =0
        #self.t_Ki =
        #self.t_Kd 

class r_pid():
    def __init__(self):
        self.t_Kp = 2
        self.Ml = 0
        self.Dl1 = 0
        self.Dl2 = 0 
        self.roll_l = 0
        self.tag1_l = 0

        self.fg = 0

tag_0 = tag_instance()
tag_1 = tag_instance()
t_pid = PID()
r_pid = PID()
p_pid = PID()
y_pid = PID()
#tag_data = tag_instance()
def tagpos_callback(data):
    if len(data.detections) >= 2:
        if data.detections[0].id[0] == 0:
            tag_0.id = 0
            tag_1.id = 1
        elif data.detections[1].id[0] == 0:
            tag_0.id = 1
            tag_1.id = 0

        #tagdata = tag_instance(data)
        tag_0.pos_x = data.detections[tag_0.id].pose.pose.pose.position.x * 100
        tag_0.pos_y = data.detections[tag_0.id].pose.pose.pose.position.y * -100
        tag_0.pos_z = data.detections[tag_0.id].pose.pose.pose.position.z * 100
        tag_1.pos_x = data.detections[tag_1.id].pose.pose.pose.position.x * 100
        tag_1.pos_y = data.detections[tag_1.id].pose.pose.pose.position.y * -100
        tag_1.pos_z = data.detections[tag_1.id].pose.pose.pose.position.z * 100

        e0 = tf.transformations.euler_from_quaternion((data.detections[tag_0.id].pose.pose.pose.orientation.x, data.detections[tag_0.id].pose.pose.pose.orientation.y, data.detections[tag_0.id].pose.pose.pose.orientation.z, data.detections[tag_0.id].pose.pose.pose.orientation.w))
        e1 = tf.transformations.euler_from_quaternion((data.detections[tag_1.id].pose.pose.pose.orientation.x, data.detections[tag_1.id].pose.pose.pose.orientation.y, data.detections[tag_1.id].pose.pose.pose.orientation.z, data.detections[tag_1.id].pose.pose.pose.orientation.w))
        
        tag_0.ori_y = e0[1] * 100
        #tag_1.ori_x = e[0] * 100
        tag_1.ori_y = e1[1] * 100
        #tag_1.ori_z = e[2] * 100

        #print "tag0.ori_y = " + str(tag_0.ori_y)
        #print "tag1.ori_y = " + str(tag_1.ori_y)

        t_pid.fg = 1
        r_pid.fg = 1
    else:
        print('no such tag')

        if t_pid.fg == 3:
            t_pid.fg = 3

        else: t_pid.fg = 2


        if r_pid.fg == 3:
            r_pid.fg = 3

        else: r_pid.fg = 2

    #---MID = 1520---#
    t_MAX = 1433
    t_MIN = 1536

    r_MAX = 1320
    r_MIN = 1720
    p_MAX = 1320
    p_MIN = 1720
    y_MAX = 1420
    y_MIN = 1620
    if t_pid.fg == 1:
        t_target = tag_0.pos_y - 0
        t_gap = t_target - tag_1.pos_y
        t_pid.pM = t_pid.t_Kp * (t_gap - t_pid.gap_l)
        t_pid.iM = t_pid.t_Ki * t_gap
        t_pid.dM = t_pid.t_Kd * ((t_gap - t_pid.gap_l) - (t_pid.gap_l - t_pid.gap_l2))
        msg.prop_thr = t_pid.out_l - t_pid.pM - t_pid.iM - t_pid.dM

        t_pid.gap_l2 = t_pid.gap_l
        t_pid.gap_l = t_gap
        #print "t_gap = " + str(t_gap)
        #print "t_pM = " + str(t_pid.pM)

        r_target = tag_0.pos_x + 60
        r_gap = r_target - tag_1.pos_x
        r_pid.pM = r_pid.r_Kp * (r_gap - r_pid.gap_l)
        r_pid.iM = r_pid.r_Ki * r_gap
        r_pid.dM = r_pid.r_Kd * ((r_gap - r_pid.gap_l) - (r_pid.gap_l - r_pid.gap_l2))
        msg.prop_roll = r_pid.out_l - r_pid.pM - r_pid.iM - r_pid.dM

        r_pid.gap_l2 = r_pid.gap_l
        r_pid.gap_l = r_gap

        #print tag_1.pos_x
    
        #print "r_gap = " + str(r_gap)
        #print "r_pM = " + str(r_pid.pM)

        p_target = tag_0.pos_z -50
        p_gap = p_target - tag_1.pos_z
        p_pid.pM = p_pid.p_Kp * (p_gap - p_pid.gap_l)
        p_pid.iM = p_pid.p_Ki * p_gap
        p_pid.dM = p_pid.p_Kd * ((p_gap - p_pid.gap_l) - (p_pid.gap_l - p_pid.gap_l2))
        msg.prop_pitch = p_pid.out_l + p_pid.pM - p_pid.iM - p_pid.dM

        p_pid.gap_l2 = p_pid.gap_l
        p_pid.gap_l = p_gap

        print tag_1.pos_z

        print "p_gap = " + str(p_gap)
        print "p_pM = " + str(p_pid.pM)

        y_gap = (tag_1.ori_y - tag_0.ori_y) 
        y_pid.pM = y_pid.y_Kp * (y_gap - y_pid.gap_l)
        y_pid.iM = y_pid.y_Ki * y_gap
        y_pid.dM = y_pid.y_Kd * ((y_gap - y_pid.gap_l) - (y_pid.gap_l - y_pid.gap_l2))
        msg.prop_yaw = y_pid.out_l - y_pid.pM - y_pid.iM - y_pid.dM

        y_pid.gap_l2 = y_pid.gap_l
        y_pid.gap_l = y_gap

        #print tag_0.ori_y
        #print tag_1.ori_y'

        print "y_gap = " + str(y_gap)
        print "y_pM = " + str(y_pid.pM)


        t_pid.out_l = msg.prop_thr
        r_pid.out_l = msg.prop_roll
        p_pid.out_l = msg.prop_pitch
        y_pid.out_l = msg.prop_yaw

        if msg.prop_thr <= t_MAX:
            msg.prop_thr = t_MAX

        elif msg.prop_thr >= t_MIN:
            msg.prop_thr = t_MIN

        if msg.prop_roll <= r_MAX:
            msg.prop_roll = r_MAX

        elif msg.prop_roll >= r_MIN:
            msg.prop_roll = r_MIN

        if msg.prop_pitch <= p_MAX:
            msg.prop_pitch = p_MAX

        elif msg.prop_pitch > p_MIN:
            msg.prop_pitch = p_MIN

        if msg.prop_yaw <= y_MAX:
            msg.prop_yaw = y_MAX

        elif msg.prop_yaw >= y_MIN:
            msg.prop_yaw = y_MIN

        #print (t_gap)

    if t_pid.fg == 2:
        if t_pid.out_l == t_MAX:
            msg.prop_thr = t_MIN

        elif t_pid.out_l == t_MIN:
            msg.prop_thr = t_MAX

        elif t_pid.pM < 0:
            msg.prop_thr = t_MIN

        elif t_pid.pM > 0:
            msg.prop_thr = t_MAX

        #if 

        t_pid.fg = 3
        t_pid.out_l = 1494
        r_pid.out_l = 1520
        p_pid.out_l = 1520     
        y_pid.out_l = 1520

    
    if t_pid.fg == 3:
        print ('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')


    #msg.prop_roll = 1520
    #msg.prop_pitch = 1520
    #msg.prop_yaw = 1520


    pub.publish(msg)
    print "thr = " + str(msg.prop_thr) + " roll = " + str(msg.prop_roll) + " pitch = " + str(msg.prop_pitch) + " yaw = " + str(msg.prop_yaw)
    #print (t_pid.fg)    


    r_pid.roll_l = msg.prop_roll
    #t_pid.out_l = msg.prop_thr

    
    t_pid.tag1_y_l = tag_1.pos_y
    tag1_y_l = tag_1.pos_y

    #pub.publish(msg)
    #rospy.loginfo(msg.prop_thr)
    #print (pid.fg)
    #print (terget)



sub = rospy.Subscriber('tag_detections',AprilTagDetectionArray, tagpos_callback)
rospy.spin()
