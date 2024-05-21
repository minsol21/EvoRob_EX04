from swarmy.perception import Perception
from swarmy.actuation import Actuation
import pygame
import math
import numpy as np
import random


class Proximity_Controller(Actuation):
   
    def __init__(self, agent, config):
        super().__init__(agent)
        self.config = config
        self.init_pos = True
        self.control_params = [random.uniform(-1, 1) for _ in range(6)]  # Initialize a random genome

    def controller(self):
        if self.init_pos:
            self.agent.initial_position()
            self.init_pos = False

        # Get sensor data
        perception = self.agent.get_perception()
        sensor_id, sensor_values = perception
        sensor_l, sensor_r, sensor_m = sensor_values

        # Calculate wheel speeds based on the control parameters
        vl = self.control_params[0]*sensor_l + self.control_params[1]
        vr = self.control_params[2]*sensor_r + self.control_params[3] + self.control_params[4]*sensor_m + self.control_params[5]

        # Define a threshold distance to detect objects
        #threshold_distance = 50
        base_speed = 5
        turn_angle = 0
        wheel_base = 0.5
        print(f"L: {sensor_l}, M: {sensor_m}, R: {sensor_r}")

        if sensor_l == 0 and sensor_r == 0 and sensor_m == 0:
            speed  = base_speed

        elif sensor_m != 0 and sensor_l != 0: 
            speed = base_speed
            turn_angle = int(((vr - vl) / wheel_base)%360) #follow left wall
        elif sensor_m != 0 and sensor_r != 0: 
            speed = base_speed
            turn_angle = int(((vr - vl) / wheel_base)%360) #follow right wall
            
        else:
            speed = 1

        self.update_position(speed, turn_angle)

    def update_position(self, speed, turn_angle):
        """
        Update the agent's position and heading over time.
        """
        # Limit the speed and turn angle to their maximum values
        max_speed = 1  # maximum distance the robot can cover in one time step
        max_turn_angle = 45  # maximum angle the robot can turn per time step
        speed = min(speed, max_speed)
        turn_angle = min(turn_angle, max_turn_angle)
       
        # Get the current position and heading
        x, y, gamma = self.agent.get_position()

        # Calculate the change in position
        dx = speed * math.sin(math.radians(gamma))
        dy = speed * math.cos(math.radians(gamma))

        # Update the heading
        new_heading = (gamma + turn_angle) % 360

        # Update the position 
        new_x = (x + dx) % self.config['world_width']
        new_y = (y + dy) % self.config['world_height']
        self.agent.set_position(new_x, new_y, new_heading)
        #current_time = pygame.time.get_ticks()  # Get the current time in milliseconds
        #self.agent.trajectory.append((new_x, new_y, current_time))
        self.agent.trajectory.append((new_x, new_y))
        # Save the image
        self.agent.save_information(None)

    def torus(self):
        """
        Implement torus world by manipulating the robot position. Again self.agent.get_position and self.agent.set_position might be useful
        """
        robot_position_x, robot_position_y, robot_heading = self.agent.get_position()

        # Implement torus world by manipulating the robot position, here.

        robot_position_x = robot_position_x % self.config['world_width']
        robot_position_y = robot_position_y % self.config['world_height']

        self.agent.set_position(robot_position_x, robot_position_y, robot_heading)