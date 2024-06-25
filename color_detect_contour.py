import cv2
from picamera2 import Picamera2
import time
import numpy as np

picam2 = Picamera2()
dispW=640
dispH=360
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
fps=0
pos=(10,20)
font=cv2.FONT_HERSHEY_SIMPLEX
height=0.5
weight=1
myColor=(0,0,255)

def onTrack1(val):
    global hueLow
    hueLow = val
    # print('Hue Low',hueLow)
def onTrack2(val):
    global hueHigh
    hueHigh = val
    # print('Hue High',hueHigh)
def onTrack3(val):
    global satLow
    satLow = val
    # print('Hue Low',satLow)
def onTrack4(val):
    global satHigh
    satHigh = val
    # print('Hue Low',satHigh)
def onTrack5(val):
    global valLow
    valLow = val
    # print('Hue Low',valLow)
def onTrack6(val):
    global valHigh
    valHigh = val
    # print('Hue Low',valHigh)

cv2.namedWindow('myTracker')

cv2.createTrackbar('Hue Low','myTracker',150,179,onTrack1)
cv2.createTrackbar('Hue High','myTracker',179,179,onTrack2) #red
cv2.createTrackbar('Sat Low','myTracker',100,255,onTrack3)
cv2.createTrackbar('Sat High','myTracker',255,255,onTrack4)
cv2.createTrackbar('Val Low','myTracker',100,255,onTrack5)
cv2.createTrackbar('Val High','myTracker',255,255,onTrack6)



#dynamixel

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
    
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 6                 # Dynamixel ID : 1
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 0                 # Value for enabling the torque
TORQUE_DISABLE              = 1                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 400           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 600           # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

center_x = int(dispW/2)

while True:
    tStart=time.time()
    frame= picam2.capture_array()
    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)
    lowerBound = np.array([hueLow,satLow,valLow])
    upperBound = np.array([hueHigh,satHigh,valHigh])
    frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    myMask = cv2.inRange(frameHSV,lowerBound,upperBound)
    myMaskSmall = cv2.resize(myMask,(int(dispW/2),int(dispH/2)))
    myObject = cv2.bitwise_and(frame,frame, mask = myMask)
    myObjectSmall = cv2.resize(myObject,(int(dispW/2),int(dispH/2)))

    contours, junk = cv2.findContours(myMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours)>0:
        contours = sorted(contours, key = lambda x:cv2.contourArea(x),reverse=True)
        # cv2.drawContours(frame,contours, 0,(255,0,0),3)
        contour = contours[0]
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),3)
        center_x = int((x+x+w)/2)
        center_y = int((y+y+h)/2)
        center = (x,y)
        print(f"center : ({center_x},{center_y})")
        cv2.circle(frame, (center_x,center_y), 1, (0, 255, 0), -1)

    cv2.imshow("Camera", frame)
    cv2.imshow('my Mask', myMaskSmall)
    cv2.imshow('My Object', myObjectSmall)

    if cv2.waitKey(1)==ord('q'):
        break
    tEnd=time.time()
    loopTime=tEnd-tStart
    fps=.9*fps + .1*(1/loopTime)

    #dynamixel

    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    print(dxl_present_position)
    # Write goal position
    if center_x >= dispW*2/5 and center_x <= dispW*3/5:
        pass

    elif center_x < dispW*2/5:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_present_position + 10)

    elif center_x > dispW*3/5:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_present_position - 10)



cv2.destroyAllWindows()