#!/usr/bin/env python

#=========================================================================================== 
#                                                                                          |
#     ODOM pubbing -- cmdvel callback -- Left/Right [CAN do moving turn]                   |
#                                                                                          |
#     > getRCnENC suppressed in odom pub                                                   |
#     > rate set to 17 for testing                                                         |
#     > frame changed from bbl to bbl2 so that the other tf pubber can send bbl            |
#===========================================================================================
import serial
import time
import io
import rospy
import tf
import math as m
import sys
from math import sin, cos, pi
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped

#########################################################################################
#########################################################################################
######################### Roboteq Controller Serial Connection ##########################
#########################################################################################
#########################################################################################

# begin the connection to the roboteq controller
try:

    device = sys.argv[1]
    ser = serial.Serial(
        port=device,
        baudrate=115200, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    )

except:

    raise IOError

# reset the connection if need be
if (ser.isOpen()):

    ser.close()

ser.open()

# serial readline based on \r 
slash_r = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),  
                               newline = '\r',
                               line_buffering = True)

slash_r.write(unicode("ID\r"))

#########################################################################################



#########################################################################################
#########################################################################################
##################################### Initialize ########################################
#########################################################################################
#########################################################################################
x = 0.0
y = 0.0
th = 0.0


def initialize():

    global odom_pub
    global odom_broadcaster
    global r
    global current_time
    global last_time
    global enc_last
    global pub_time
    global test_wheels

    rospy.init_node('base_odom_publisher_node')
    rospy.loginfo('status: odometry_publisher crearted')

    rospy.Subscriber('/cmd_vel', Twist, move_callback)
    rospy.Subscriber('/e_stop', Bool, estop_callback)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.loginfo('status: odom_pub initiated')
    # r = rospy.Rate(17.0)
    r = rospy.Rate(20.0)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    pub_time = rospy.get_time()
    
    getEncoders()
    enc_last = test_wheels

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Debug mode #######################################
#########################################################################################
#########################################################################################

debug = 0
encoder_debug = 0
rc_debug = 0
mode_debug = 0
speed_debug = 1
reader_debug = 0
getdata_debug = 1
test_debug = 0
rc_enc_debug = 0
motor_command_debug = 0
callback_debug = 0

def debug_printer(debug,variable,variable_value):

    if debug == 1:

        print '======',variable,':',variable_value,'======'
        # msg = "======" + str(variable) + ":" + str(variable_value)
        # rospy.loginfo(msg)

    return 

# debug_printer(debug,'DEBUG MODE', 'ACTIVATED')

#########################################################################################



#########################################################################################
#########################################################################################
############################### Get Data and Clean message ##############################
#########################################################################################
#########################################################################################

def getdata3(num_of_reads):

    global debug
    global getdata_debug

    output = []
    count = 0

    while True:
        line = slash_r.readline()
        if (line != '') and (line != '+\r') and (line != '\r'):
            output.append(line)
            count += 1
        if count >= num_of_reads:
            break 

    debug_printer(debug,'getdata3 - funtion output',output)
    debug_printer(getdata_debug,'getdata3 - funtion output',output)

    return output

def clean_messages(message):
    '''
    Encoder output looks like this - "C=123456\r"
    This function reads output like that and returns "123456"
    '''
    clean_message = ''
    start = 0
    try:
        for i in range(0,len(message)):
            if (message[i]== '='):
                start = 1
            if (message[i]== '\r'):
                clean_message = int(clean_message)
                return clean_message
            if (start == 1):
                clean_message += message[i+1]
    except:
        return clean_message


#########################################################################################



#########################################################################################
#########################################################################################
####################################### reader ##########################################
#########################################################################################
#########################################################################################

#=========================================================================================== 
# --> Arguments - 1. query_id                                                              |                                                               |
#                                                                                          |
#   1. query_id    - Roboteq has about 20 or so values that can be queried but the current |
#                    version of function can get pulse input(pulse-PI), absolute encoder   | 
#                    value(enc_abs - C), relative encoder rpm(enc_rel_rpm - RS) and battery|
#                    voltage                                                               |
#                                                                                          |
# --> Output - The function returns what is read from the serial port for the command sent |
#===========================================================================================

def reader(query_id):

    global debug
    global reader_debug

    num_of_reads = 0

    if query_id == 'battery':
        command = '?V 2\r'
        num_of_reads = 1
    elif query_id == 'enc_abs':
        command = '?C 1_?C 2\r'
        num_of_reads = 2
    else:
        print 'error: query_id unknown'
        return False

    debug_printer(debug,'reader - command',command)
    debug_printer(reader_debug,'reader - command',command)

    time.sleep(0.0015)
    ser.write(command)  
    time.sleep(0.0015)

    output = getdata3(num_of_reads)

    return output


#########################################################################################




#########################################################################################
#########################################################################################
###################################### Pulse 2 Speed ####################################
#########################################################################################
#########################################################################################

def pulse2vel(pulses):

    global min_forward_pulse1 
    global min_backward_pulse1
    global min_forward_pulse2 
    global min_backward_pulse2

    signs = [0,0]

    if (pulses[0] > min_forward_pulse1):

        speed = pulses[0] - min_forward_pulse1
        signs[0] = 1
        signs[1] = -1
        left  = speed * signs[0]
        right = speed * signs[1] 

    elif (pulses[0] < min_backward_pulse1):

        speed =  -(pulses[0] - min_backward_pulse1)
        signs[0] = -1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    elif (pulses[1] > min_forward_pulse2):

        speed = pulses[1] - min_forward_pulse2
        signs[0] = 1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    elif (pulses[1] < min_backward_pulse1):

        speed =  -(pulses[1] - min_backward_pulse2)
        signs[0] = -1
        signs[1] = -1
        left  = speed * signs[0]
        right = speed * signs[1]

    else:

        speed = 0
        signs[0] = 1
        signs[1] = 1
        left  = speed * signs[0]
        right = speed * signs[1]

    return left,right

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Get Encoders #####################################
#########################################################################################
#########################################################################################

def getEncoders():

    global debug
    global encoder_debug
    global test_debug
    global test_wheels

    leftWheel = [0,0]
    rightWheel = [0,0]

    encoder_absolute = reader('enc_abs')

    encoder_values = [clean_messages(encoder_absolute[0]), clean_messages(encoder_absolute[1])]
    
    leftWheel = [encoder_values[0], encoder_values[0]]
    rightWheel = [encoder_values[1], encoder_values[1]]

    test_wheels = [leftWheel,rightWheel]

    debug_printer(debug,'getEncoders - encoder_values',encoder_values)
    debug_printer(encoder_debug,'getEncoders - encoder_values',encoder_values)

    return leftWheel, rightWheel

#########################################################################################



#########################################################################################
#########################################################################################
###################################### Move Command #####################################
#########################################################################################
#########################################################################################

def move_command(left,right):

    global command_merger
    global motor_command_debug

    debug_printer(motor_command_debug,"motor_command - left,right",[left,right])

    try:
        cmd = '!G 1 ' + str(left) + '_' + '!G 2 ' + str(right) + '\r'
        ser.write(cmd)
    except Exception as e:
        print e
        print( "error: move_command" )
    return 

#########################################################################################



#########################################################################################
#########################################################################################
###################################### data2vel #########################################
#########################################################################################
#########################################################################################

def data2vel(data):

    global mps_to_movecmd
    global rps_to_movecmd

    left  =   (data.linear.x * mps_to_movecmd) - (data.angular.z * rps_to_movecmd)
    right = - (data.linear.x * mps_to_movecmd) - (data.angular.z * rps_to_movecmd)
    
    return left,right

#########################################################################################



#########################################################################################
#########################################################################################
######################################## TEST ###########################################
#########################################################################################
#########################################################################################

test_error = 0

def TEST():

    global test_error

    for i in range(30):
        try:
            encabs = getEncoders()
            error = 0            
            time.sleep(0.01)
            move_command(0,0)
            time.sleep(0.01)

            battery = reader('battery')
            battery_output = clean_messages(battery[0])
            print("="*70)
            print(battery_output/10.0)

        except Exception as e:
            print e
            print( "error: TEST" )
            error = 1

    if error == 1:
        test_error = 1

    return 

#########################################################################################



#########################################################################################
#########################################################################################
#################################### odom publisher #####################################
#########################################################################################
#########################################################################################

vx = 0.0
vy = 0.0
vth = 0.0

encoder_ppr = 2048                                                      # encoder is connected to the shaft. Gear ratio between shaft and wheel is ~81
wheel_diameter = 16                                                     # in inches
in_to_m = 0.0254
 
enc_countperrev_left_front  = 165881.1                                  # Gear ratio between shaft and wheel is ~81 (2048 x 81 = 165888)
enc_countperrev_left_rear   = 165884.3                                 
enc_countperrev_right_front = 165883.9                                 
enc_countperrev_right_rear  = 165885.5                                 

DistancePerCount_left_front  = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_left_front               # (PI*D) / ppr
DistancePerCount_left_rear   = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_left_rear            
DistancePerCount_right_front = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_right_front            
DistancePerCount_right_front = (m.pi * wheel_diameter * in_to_m) / enc_countperrev_right_rear              

lengthBetweenTwoWheels = 26.0 * in_to_m
factor = 280.0/360.0 # 0.69 - 250/360
odom_old = Odometry()

# distance between origin and the center of the robot
origin_x = 0.76/2 + 0.025
origin_y = 0.55/2 - 0.025

def odom_publisher():

    global odom_pub
    global current_time
    global last_time
    global enc_last

    global test_wheels

    global DistancePerCount_left_front
    global DistancePerCount_left_rear
    global DistancePerCount_right_front
    global DistancePerCount_right_front

    global lengthBetweenTwoWheels
    global factor
    
    global x
    global y
    global th

    global odom

    # global pose_pub
    global odom_old

    current_time = rospy.Time.now()

    getEncoders()
    enc_now = test_wheels

    deltaLeft1  = enc_now[0][0]  - enc_last[0][0]
    deltaLeft2  = enc_now[0][1]  - enc_last[0][1]
    deltaRight1 = enc_now[1][0]  - enc_last[1][0]
    deltaRight2 = enc_now[1][1]  - enc_last[1][1]

    time_elapsed = current_time.to_sec() - last_time.to_sec()

    v_left1  = (deltaLeft1  * DistancePerCount_left_front)  / time_elapsed
    v_left2  = (deltaLeft2  * DistancePerCount_left_rear)   / time_elapsed
    v_right1 = (deltaRight1 * DistancePerCount_right_front) / time_elapsed
    v_right2 = (deltaRight2 * DistancePerCount_right_front) / time_elapsed

    v_left  = (v_left1  + v_left2)  / 2 
    v_right = -(v_right1 + v_right2) / 2 

    vx  = ((v_right + v_left) / 2);
    vy  = 0;
    vth = ((v_right - v_left) / lengthBetweenTwoWheels);

    # compute odometry in a typical way given the velocities of the robot
    dt       = (current_time - last_time).to_sec()
    delta_x  = (vx * cos(th) - vy * sin(th)) * dt
    delta_y  = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    # x -= (origin_x*cos(th) + origin_y*sin(th))
    # y -= (origin_x*sin(th) - origin_y*cos(th))

    x  += delta_x
    y  += delta_y
    th += factor*delta_th

    # x += (origin_x*cos(th) + origin_y*sin(th))
    # y += (origin_x*sin(th) - origin_y*cos(th))

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th, axes='rxyz')
    # odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0, axes='rxyz')

    # next, we'll publish the odometry message over ROS
    odom                 = Odometry()
    odom.header.stamp    = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist    = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # set the covariance
    odom.pose.covariance  = [1e-3,0,0,0,0,0, 0,1e-3,0,0,0,0, 0,0,1e-6,0,0,0, 0,0,0,1e-6,0,0, 0,0,0,0,1e-6,0, 0,0,0,0,0,1e-3] ;                                                    
    odom.twist.covariance = [1e-3,0,0,0,0,0, 0,1e-3,0,0,0,0, 0,0,1e-6,0,0,0, 0,0,0,1e-6,0,0, 0,0,0,0,1e-6,0, 0,0,0,0,0,1e-3] ;
 
    # publish the message
    odom_pub.publish(odom)

    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       odom.child_frame_id,
       # "base_footprint"
       odom.header.frame_id
    )

    # odom_broadcaster.sendTransform(
    #    (0, 0, 0.),
    #    odom_quat,
    #    current_time,
    #    odom.child_frame_id,
    #    # "base_footprint"
    #    odom.header.frame_id
    # )

#########################################################################################



#########################################################################################
#########################################################################################
#################################### Move Callback ######################################
#########################################################################################
#########################################################################################

EM_STOP = 0
RC_MODE = 0

# new speed to movecommand
mps_to_movecmd = 20.0/0.1033401

# new ang_vel to movecommand
rps_to_movecmd = 20.0/0.20862

# max linear velocity
v_max = 0.5

# max angular velocity
w_max = 1.0

estop = False

def sign(a):
    return (a/abs(a))

def estop_callback(data):
    global estop
    estop = data.data
    if estop:
        ser.write('!EX\r')
    else:
        ser.write('!MG\r')

def move_callback(data):

    global debug
    global mode_debug
    global speed_debug
    global EM_STOP
    global RC_MODE
    global pub_time
    # global test_pulsevals
    global v_max
    global w_max
    global callback_debug


    pulses = [0, 0]
    signs = [1,-1]

    debug_printer(debug,"RC_MODE",RC_MODE)
    debug_printer(debug,"EM_STOP",EM_STOP)
    debug_printer(mode_debug,"RC_MODE",RC_MODE)
    debug_printer(mode_debug,"EM_STOP",EM_STOP)

    debug_printer(callback_debug,"cmdvel data received - [ vx , wz ]",[data.linear.x,data.angular.z])

    pub_time = rospy.get_time()

    # if (pulses[0] > 1500) and (RC_MODE != 1):  #estop activated

    #     ser.write('!EX\r')
    #     EM_STOP = 1

    # elif (EM_STOP) and (pulses[1] > 1500):

    #     ser.write('!MG\r')
    #     EM_STOP = 0
    #     RC_MODE = 1

    # elif (RC_MODE):

    #     [left,right] = pulse2vel(pulses)
    #     move_command(left,right)
    if not estop:

        if (abs(data.linear.x) < v_max) and (abs(data.angular.z) < w_max):

            [left,right] = data2vel(data)
            print "cmd command-------------"

        else:

            left  = 0
            right = 0

            print "cmd failed------------- vx=", data.linear.x, 'wz=', data.angular.z

        debug_printer(debug,"left",left)
        debug_printer(debug,"right",right)
        debug_printer(speed_debug,"left",left)
        debug_printer(speed_debug,"right",right)


        move_command(left,right)

#########################################################################################



#########################################################################################
#########################################################################################
######################################### Main ##########################################
#########################################################################################
#########################################################################################

if __name__ == "__main__":
    TEST()
    initialize()
    while (not rospy.is_shutdown()) and (test_error == 0):

        try:
            getEncoders()
            odom_publisher()
            enc_last = test_wheels
            last_time = current_time
            r.sleep()
            time_now = rospy.get_time()
            # print "time diff:", time_now - pub_time

            cmd_vel_timeout = 0.6 # seconds

            if (time_now - pub_time > cmd_vel_timeout):
                # ser.write('!MG\r') # Emergency Stop Release
                move_command(0,0)

        except KeyboardInterrupt:
            left = 0
            right = 0
            move_command(left,right)
            ser.close()

    left = 0
    right = 0
    move_command(left,right)
    ser.close()

#########################################################################################
