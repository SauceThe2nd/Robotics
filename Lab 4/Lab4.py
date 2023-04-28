"""Lab4 controller."""

from controller import Robot, DistanceSensor, Motor
import numpy as np

#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = robot.getBasicTimeStep()/1000.0    # [s]

# states
states = ['forward', 'turn_right', 'turn_left']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 3

goalc = 0 #goal counter
xcom = 0 #xgoal completed
setgoal = 0
steps = 0
distancesteps = 0
rotatesteps = 0
srotate = 0

# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = 0    # position in x [m]
y = 0    # position in y [m]
phi = 0  # orientation [rad]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.052    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]

#-------------------------------------------------------
# Initialize devices

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)


# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

######################################################################
#----- functions ----------------------------------------------------------------
def PositionError(xgoal, ygoal, fx, fy, fphi):
    # Position error:
    x_err = xgoal - fx
    y_err = ygoal - fy
    dist_err = np.sqrt(x_err**2 + y_err**2)

    # Orientation error
    phi_d = np.arctan2(y_err,x_err)
    phi_err = phi_d - fphi

    # Limit the error to (-pi, pi):
    phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))

    return x_err, y_err, phi_err_correct, dist_err

#######################################################################
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


#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Update sensor readings
    steps += 1

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
        
    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   


    #######################################################################
    # Using the equations for the robot kinematics based on speed
    
    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    
    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)
    
    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    #######################################################################
    
    # update old encoder values for the next cycle
    oldEncoderValues = encoderValues
    
    #set goals
    if goalc == 0:
        xg = 0.8
        yg = 0

    elif goalc == 1:
        xg = 0
        yg = 0.9

    elif goalc == 2:
        xg = 1.6
        yg = 0

    elif goalc == 3:
        xg = 0
        yg = 1
    
    elif goalc == 4:
        xg = 0.8
        yg = 0

    elif goalc == 5:
        xg = 0
        yg = 0.45

    xerr, yerr, phierr, disterr = PositionError(xg, yg, x, y, phi)

    if xg > yg:
        error = xerr
        current = x
        goal = xg
    else: 
        error = yerr
        current = y
        goal = yg

    if goalc > 5:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print('good job!')
        setgoal = 1
        srotate = 1
        break

    if setgoal == 0:
        distancesteps = (goal / 0.004) + steps
        setgoal = 1


    if distancesteps >= steps:
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
        

    elif steps > distancesteps:
        if srotate == 0:
            if goalc == 0:
                rotatesteps = steps + 44
            
            elif goalc == 1:
                rotatesteps = steps + 42

            elif goalc == 2:
                rotatesteps = steps + 44

            elif goalc == 3:
                rotatesteps = steps + 44

            elif goalc == 4:
                rotatesteps = steps + 43

            srotate = 1

        if rotatesteps > steps:
            leftSpeed = MAX_SPEED/4
            rightSpeed = -MAX_SPEED/4
        
        else:
            goalc += 1
            srotate = 0
            setgoal = 0

    # Set motor speeds with the values defined by the state-machine
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Repeat all steps while the simulation is running.

