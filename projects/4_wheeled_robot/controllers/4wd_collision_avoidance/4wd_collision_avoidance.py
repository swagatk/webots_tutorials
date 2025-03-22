"""4wd_collision_avoidance controller."""


# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())
timestep = 64
max_speed = 6.28
scale = 0.7

# initialize motors
motors = []
motor_names = ['motor_1', 'motor_2', 'motor_3', 'motor_4']

for name in motor_names:
    motors.append(robot.getDevice(name))
    
for i in range(4):
    motors[i].setPosition(float('inf'))

#initialize sensors
ds = []
ds_names = ['ds_left', 'ds_right']

for i in range(2):
    ds.append(robot.getDevice(ds_names[i]))
    ds[i].enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
avoidObstacleCounter = 0
while robot.step(timestep) != -1:

    left_speed = scale * max_speed
    right_speed = scale * max_speed 
    if avoidObstacleCounter > 0: # rotate to avoid obstacle
        avoidObstacleCounter -= 1
        left_speed = 1.0 * max_speed
        right_speed = -1.0 * max_speed
    else: # read sensors
        for i in range(2):
            #print(ds[i].getValue())
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 10
                
    # actuate the motors
    motors[0].setVelocity(left_speed)
    motors[1].setVelocity(right_speed)
    motors[2].setVelocity(left_speed)
    motors[3].setVelocity(right_speed)
        
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    

# Enter here exit cleanup code.
