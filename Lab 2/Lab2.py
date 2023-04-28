"""Lab2 controller."""


#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor
import numpy as np
# create the Robot instance.
robot = Robot()

# define max speed
MAX_SPEED = 6.28

# get the time step of the current world.
TIMESTEP = 32 # ms
delta_t = TIMESTEP/1000.0 #s

# set state for avoiding obstacle
avoid = 0
runtime = 0
totalS = 0
action = 0
rejoin = 0

# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]


# initz line sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(TIMESTEP)

# initz proximity sensors
ps = []
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIMESTEP)

# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(TIMESTEP)

oldEncoderValues = []

# initz motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Robot Localization functions

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    #Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2   
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi


#------------ Main Loop ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
while True:

    # start simulation step
    robot.step(TIMESTEP)

    # indicate new info batch
    print('.')
    print('.')
    print('.')
    print('.')
    print('.')

    # define seconds since start simulation, every step is 0.032 seconds
    totalS = round(totalS + 0.032, 4)
    print(totalS)
# Data gathering --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    # Read the distance sensors
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # process distance sensor data
    FLsensor = round(psValues[7])
    FRsensor = round(psValues[0])
    Lsensor = round(psValues[5])
    Rsensor = round(psValues[2])

    # print distance data
    print('sensorFL '+str(FLsensor))
    print('sensorFR '+str(FRsensor))
    print('sensorL '+str(Lsensor))
    print('sensorR '+str(Rsensor))

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Read the line sensors:
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process line sensor data here.
    line_right = round(gsValues[0])
    line_center = round(gsValues[1])
    line_left = round(gsValues[2])

    # print line data
    print('line_right: '+ str(line_right))
    print('line_left: '+ str(line_left))
    print('line_center: '+ str(line_center))

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Read the encoders
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
        
    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   

#-- Localization ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    
    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)
    
    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    # update old encoder values for the next cycle
    oldEncoderValues = encoderValues

    print(f'Sim time: {totalS}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')


#-- Obstacle avoidance --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    #check for obstacle / enter object avoidance state
    if FLsensor > 100 or FRsensor > 100 or avoid == 1:
        
        if rejoin == 0:

            # set ending variables
            end = totalS - 2
            end2 = totalS - 4

            # turn right when obstacle is detected
            if avoid == 0 or end < start:
                # set avoid state to 1

                print('enter avoid state')

                # start timer
                if avoid == 0:
                    start = totalS
                    avoid = 1

                leftMotor.setVelocity(MAX_SPEED/2)
                rightMotor.setVelocity(MAX_SPEED/20)

            # set action trigger to 0
            action = 0
            # turn away from obstacle
            if Lsensor > 150 and action == 0  and end > start:
                leftMotor.setVelocity(MAX_SPEED/5)
                rightMotor.setVelocity(MAX_SPEED/20)
                action = 1
                print('turn away object (Lsens)')

            # turn away from obstacle
            if FLsensor > 90 and action == 0  and end > start:
                leftMotor.setVelocity(MAX_SPEED/5)
                rightMotor.setVelocity(MAX_SPEED/20)
                action = 1
                print('turn away object (FLsens)')

            # turn towards obstacle
            if Lsensor < 150 and action == 0 and end > start:
                leftMotor.setVelocity(MAX_SPEED/20)
                rightMotor.setVelocity(MAX_SPEED/6)
                action = 1 
                print('turn toward object (Lsens)')

            # start to rejoin
            line_check = line_center + line_left + line_right
            if (line_check < 1100 or rejoin == 1) and end2 > start:
                leftMotor.setVelocity(MAX_SPEED/5)
                rightMotor.setVelocity(MAX_SPEED/20)
                print('rejoining')
                rejoin = 1
                start2 = totalS
                print(start2)
            
        elif rejoin == 1:

            # set ending variables
            end = totalS - 2
            end2 = totalS - 4
            leftMotor.setVelocity(MAX_SPEED/5)
            rightMotor.setVelocity(MAX_SPEED/20)
            print('rejoining')

            if end2 > start2:
                avoid = 0
                rejoin = 0
                print('ending avoid obstacle')



#-- Line Following --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    if avoid == 0:

        # fullspeed ahead
        fullspeed = 0
        if line_left < 500 and line_right < 500:
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED)
            fullspeed = 1
            print('fullspeed')
        # turn Right
        right = 0
        if line_right > 500 and line_left < 500:
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED/2)
            right = 1
            print('right')
        # turn Left
        left = 0
        if line_right < 500 and line_left > 500:
            leftMotor.setVelocity(MAX_SPEED/2)
            rightMotor.setVelocity(MAX_SPEED)
            left = 1
            print('left')
        # if there is no line turn right (most probable as it is a right turning oval)
        if fullspeed == 0 and right == 0 and left == 0: 
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(2)
            print('houston, we have a problem')