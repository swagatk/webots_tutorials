"""avoid_obstacle controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

def initialize_robot(robot):
    global timestep, max_speed
    global left_motor, right_motor, ds

    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28
    
    # initialize motors
    left_motor = robot.getDevice('motor_1')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('motor_2')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    
    # initialize sensors
    dsnames = ['ps_0', 'ps_1']
    ds = []
    for i in range(2):
        ds.append(robot.getDevice(dsnames[i]))
        ds[i].enable(timestep)
      


def avoid_obstacle_2(robot):
    scale = 0.3
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        
        avoid = False
        for i in range(2):
            if ds[i].getValue() < 1000.0:
                avoid = True
            
        if avoid: 
            left_speed = scale * max_speed
            right_speed = -1.0 * scale * max_speed
        else: # no obstacle
            left_speed = scale * max_speed
            right_speed = scale * max_speed
                    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)     
        
    # Enter here exit cleanup code.
    

def avoid_obstacle(robot):
   
    scale = 0.3
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    avoidObstacleCounter = 0
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        left_speed = scale * max_speed
        right_speed = scale * max_speed
        if avoidObstacleCounter > 0: # obstacle avoid
            avoidObstacleCounter -= 1
            left_speed = scale * max_speed
            right_speed = -1.0 * scale * max_speed
        else: # no obstacle
            for i in range(2):
                if ds[i].getValue() < 950.0:
                    avoidObstacleCounter = 50
                    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)     
        
    # Enter here exit cleanup code.
    

if __name__ == '__main__':


    # create the Robot instance.
    my_robot = Robot() 
    
    initialize_robot(my_robot)
    
    # avoid_obstacle(my_robot)
    
    avoid_obstacle_2(my_robot)
    
