"""Lab 5 controller."""

from controller import Robot, DistanceSensor, Motor
import numpy as np

#-------------------------------------------------------
# Initialize variables

TIME_STEP = 64
MAX_SPEED = 6.28
counter = 0

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = timestep/1000.0    # [s]

# Robot pose
# ********************** ADJUST VALUES OF ROBOT POSE *************************
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]
# ****************************************************************************

# Robot initial velocity and acceleration in (x,z) coordinates
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]
is_saturated = False    # Indicates when any of the wheels is saturated (max speed)

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# Physical parameters of the robot for the kinematics model
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]

#-------------------------------------------------------
# Initialize devices

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

#-------------------------------------------------------
# Functions

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

    return dist_err

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

def get_cartesian_speeds(u, w, phi, a):
    """Computes cartesian speeds of the robot"""
    dx = u * np.cos(phi) + a * w * np.sin(phi)
    dy = u * np.sin(phi) - a * w * np.cos(phi)
    dphi = w

    return dx, dy, dphi

def get_robot_pose(x_old, y_old, phi_old, dx, dy, dphi, delta_t):
    """Updates robot pose"""
    phi = phi_old + dphi * delta_t
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    x = x_old + dx * delta_t
    y = y_old + dy * delta_t

    return x, y, phi

def traj_tracking_controller(dxd, dyd, xd, yd, x, y, phi, a):
    """Updates references speeds for the robot to follow a trajectory"""
    # Controller gains:
    KX = 4
    KY = 2

    # Position error:
    x_err = xd - x
    y_err = yd - y
    
    # If error is smaller than some value, make it null:
    if (abs(x_err) < 0.001) and (abs(y_err) < 0.001):
        x_err = 0
        y_err = 0
        
    # Controller equation - matrix format:
    #C = np.matrix([[np.cos(phi), np.sin(phi)],
    #               [-1/a*np.sin(phi), 1/a*np.cos(phi)]])
    #[u_ref, w_ref] = C * np.matrix([[dxd + kx*x_err],[dyd + ky*y_err]])

    # Controller equations - non-matrix format:
    u_ref = np.cos(phi)*(dxd + KX*x_err) + np.sin(phi)*(dyd + KY*y_err)
    w_ref = -(1/a)*np.sin(phi)*(dxd + KX*x_err) + (1/a)*np.cos(phi)*(dyd + KY*y_err)
    
    return u_ref, w_ref
    
def wheel_speed_commands(u_ref, w_ref, d, r):
    """Converts reference speeds to wheel speed commands"""
    global is_saturated
    leftSpeed = float((2 * u_ref - d * w_ref) / (2 * r))
    rightSpeed = float((2 * u_ref + d * w_ref) / (2 * r))
    
    # Limits the maximum speed of one wheel to MAX_SPEED, if necessary.
    # Keeps the proportion between left and right wheel speeds
    if np.abs(leftSpeed) > MAX_SPEED or np.abs(rightSpeed) > MAX_SPEED:
        speed_ratio = np.abs(rightSpeed)/np.abs(leftSpeed)
        is_saturated = True
        if speed_ratio > 1:
            rightSpeed = np.sign(rightSpeed)*MAX_SPEED
            leftSpeed = np.sign(leftSpeed)/speed_ratio
        else:
            leftSpeed = np.sign(leftSpeed)*MAX_SPEED
            rightSpeed = np.sign(rightSpeed)*speed_ratio
    else:
        is_saturated = False

    #leftSpeed = np.sign(leftSpeed) * min(np.abs(leftSpeed), MAX_SPEED)
    #rightSpeed = np.sign(rightSpeed) * min(np.abs(rightSpeed), MAX_SPEED)
    
    return leftSpeed, rightSpeed

#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Update sensor readings

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]

    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   

    #######################################################################
    # Robot Localization 
    x_old = x
    y_old = y
    phi_old = phi
    
    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)    
    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)
    # Compute cartesian speeds of the robot
    [dx, dy, dphi] = get_cartesian_speeds(u, w, phi, A)    
    # Compute new robot pose
    [x, y, phi] = get_robot_pose(x_old, y_old, phi_old, dx, dy, dphi, delta_t)


    #######################################################################
    # Robot Controller
    # Desired trajectory (you can use equations to define the trajectory):
    xd = 0.0 + 0.3*np.sin(0.005*counter)
    yd = 0.436
    dxd = 0.3*0.005*np.cos(0.005*counter) # This is the time derivative of yd
    dyd = 0.0

    # Desired trajectory 2
    #xd = 0.0 + 0.03*np.sin(0.009*counter)
    #yd = 0.436
    #dxd = 0.03*0.009*np.cos(0.009*counter) # This is the time derivative of yd
    #dyd = 0
    
    # Desired trajectory 3  
    #xd = 0.0 + 0.007*np.sin(0.05*counter)
    #yd = 0.436
    #dxd = 0.007*0.05*np.cos(0.05*counter) # This is the time derivative of yd
    #dyd = 0  

    Diste = PositionError(xd, yd, x, y, phi)

    # Trajectory tracking controller
    [u_ref, w_ref] = traj_tracking_controller(dxd, dyd, xd, yd, x, y, phi, A)
    # Convert reference speeds to wheel speed commands
    [leftSpeed, rightSpeed] = wheel_speed_commands(u_ref, w_ref, D, R)

    

    #######################################################################
    
    # update old encoder values and counter for the next cycle
    oldEncoderValues = encoderValues
    counter += 1

    # To help on debugging:
    print(f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} Distance error={Diste:.3f}m.')# , w_ref={w_ref:.3f} rad/s, Saturation = {is_saturated}.')    

    # Update reference velocities for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

