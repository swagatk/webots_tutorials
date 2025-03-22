"""odeometry_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot

def run_robot(robot):

    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28

    # get access to the motors
    left_motor = robot.getDevice('motor_1')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('motor_2')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    
    # create instance for wheel encoder
    left_pos = robot.getDevice('wheel_enc_1')
    left_pos.enable(timestep)
    
    right_pos = robot.getDevice('wheel_enc_2')
    right_pos.enable(timestep)
    
    pos_values = [0, 0]  # in radians
    dist_values = [0, 0]  # in meters
    
    # encoder unit to convert radian to linear distance in meters
    wheel_radius = 0.025
    wheel_circum = 2 * 3.14 * wheel_radius
    encoder_unit = wheel_circum / 6.28
    dist_betn_wheels = 0.09

    # computing robot pose
    robot_pose = [0, 0, 0]   # (x, y, theta)
    last_pos_values = [0, 0] 
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        pos_values[0] = left_pos.getValue()
        pos_values[1] = right_pos.getValue()
        
        
        print('position sensor values in rad: {:.2f}, {:.2f}'\
            .format(pos_values[0], pos_values[1]))
        
        # robot position in distance
        for i in range(2):
            diff = pos_values[i] - last_pos_values[i]
            if diff < 0.001:
                diff = 0
                pos_values[i] = last_pos_values[i]
            dist_values[i] = diff * encoder_unit

        
        print('position sensor values in meters: {:.2f}, {:.2f}'\
            .format(dist_values[0], dist_values[1]))
        
        # compute linear and angular velocity
        v = (dist_values[0] + dist_values[1])/2.0
        w = (dist_values[0] - dist_values[1])/dist_betn_wheels
        
        dt = 1
        robot_pose[2] += (w * dt)
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += (vx * dt)
        robot_pose[1] += (vy * dt)
        
        print('robot pose: {}'.format(robot_pose))
    
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(-max_speed)
        
        for i in range(2):
            last_pos_values[i] = pos_values[i]
        
    



if __name__ == '__main__':

    #create an instance of the robot
    my_robot = Robot()

    run_robot(my_robot)
