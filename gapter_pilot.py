#!/usr/bin/env python

#Needed Libraries for the code
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped


#Functions to services requests

def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e


def joystick():

    class Joystick(object):
        def __init__(self):
            self.axes = 12*[0.,]    #Defines 2 objects, axes and buttons
            self.buttons = 12*[0.,]
            rospy.Subscriber("/joy", Joy, self.callback)    #Subscription to joy topic for reading Joy type messages

        def callback(self, msg):
            self.axes = msg.axes
            self.buttons = msg.buttons

    class Movement():
        def __init__(self): #Parameters to update the position using a rate
            self.forward_rate = rospy.get_param('~forward_rate', 0.5)
            self.backward_rate = rospy.get_param('~backward_rate', 0.5)
            self.rigth_rate = rospy.get_param('~rigth_rate', 0.5) 
            self.left_rate = rospy.get_param('~left_rate', 0.5)
            self.up_rate = rospy.get_param('~up_rate', 0.5)
            self.down_rate = rospy.get_param('~down_rate', 0.5)

            #Objects to assign the drone position
            self.linearX = 0
            self.linearY = 0
            self.linearZ = 0


    #Publish in the topic /mavros/setpoint_velocity/cmd_vel
    joy_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    
    #Objects instance
    joystick = Joystick()
    move = Movement()
    joy = TwistStamped()

    while(joystick.buttons[9] == 0):    #joystick.buttons[9] = Start button
        move.linearX = 0
        move.linearY = 0
        move.linearZ = 0
        
        if joystick.axes[0] > 0.5:  #Limit and axes configuration defined by the user
            move.linearX = joystick.axes[0] * move.forward_rate     #Product between the new position and the move rate
        elif joystick.axes[0] < -0.5:
            move.linearX = joystick.axes[0] * move.backward_rate

        if joystick.axes[1] > 0.5:    
            move.linearY = joystick.axes[1] * move.left_rate
        elif joystick.axes[1] < -0.5:
            move.linearY = joystick.axes[1] * move.rigth_rate

        if joystick.axes[2] > 0.5:
            move.linearZ = joystick.axes[2] * move.up_rate
        elif joystick.axes[2] < -0.5:
            move.linearZ = joystick.axes[2] * move.down_rate  

        print('x: %f - y: %f - z: %f\n' % (move.linearX, move.linearY, move.linearZ))
        joy.twist.linear.x = move.linearX   #Assignment of the desired point
        joy.twist.linear.y = move.linearY
        joy.twist.linear.z = move.linearZ
        joy_pub.publish(joy)    #Publish de twistStamped type message
        rospy.sleep(0.5)

def myLoop():
    rate = rospy.Rate(100)  #Defines the update frecuency for the loop

    while not rospy.is_shutdown():  #Options to select
        print "Press s to takeoff\n"
        print "Press j to use joystick\n"
        print "Press start to leave joystick\n"
        print "Press f to land\n"

        x = raw_input("Enter your input: ");    #Input to select the option

        if(x == 's' or x == 'S'):   #Minus/mayus key choice
            setGuidedMode() #Guided mode function for the drone
            setArm()    #Arming motors function for the drone
            setTakeoffMode()    #Takeoff function

        elif(x == 'j' or x == 'J'):
            joystick()  #Function to operate the drone with a joystick
    
        elif(x == 'f' or x == 'F'):
            setLandMode()   #Function to land
            setDisarm() #Function to disarming motors

        else: 
            print "Exit"
    rate.sleep(0.5)    #ROS will sleep for the specified period

if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)    #Init node called gapter_pilot_node
    myLoop()    #Inconditional loop function