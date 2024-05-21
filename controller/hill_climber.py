import random
from swarmy.actuation import Actuation
import yaml
import math

class HillClimber(Actuation):
    def __init__(self, agent, config):
        super().__init__(agent)
        self.config = config
        self.init_pos = True
        self.control_params = [random.uniform(-1, 1) for _ in range(6)]  # Initialize a random genome
        self.best_fitness = -1  # Initialize the best fitness to a very low value

    def controller(self):
        if self.init_pos:
            self.agent.initial_position()
            self.init_pos = False

        # Get sensor data
        sensor_id, sensor_values = self.agent.get_perception()
        sensor_l, sensor_r, sensor_m = sensor_values

        # Calculate wheel speeds based on the control parameters
        vl = self.control_params[0]*sensor_l + self.control_params[1]
        vr = self.control_params[2]*sensor_r + self.control_params[3] + self.control_params[4]*sensor_m + self.control_params[5]

        # Calculate new linear and angular velocities
        self.linear_velocity = (vl + vr) / 2
        self.angle_velocity = -(vr - vl) / 3
        print(self.angle_velocity*10)
        # Update the position
        self.update_position(self.linear_velocity, self.angle_velocity)

        # Calculate the fitness
        fitness = self.calculate_fitness()

        # If the fitness has improved, keep the control parameters. Otherwise, revert to the previous control parameters.
        if fitness > self.best_fitness:
            self.best_fitness = fitness
        else:
            self.control_params = self.previous_control_params

        # Mutate the control parameters to create a new candidate
        self.previous_control_params = self.control_params
        self.control_params = [param + random.uniform(-0.1, 0.1) for param in self.control_params]

    def update_position(self, speed, turn_angle):
        """
        Update the agent's position and heading over time.
        """
        # Get the current position and heading
        x, y, gamma = self.agent.get_position()

        # Calculate the change in position
        dx = speed * math.sin(math.radians(gamma))
        dy = speed * math.cos(math.radians(gamma))

        # Update the heading
        new_heading = int((gamma + turn_angle) % 360)

        # Update the position 
        new_x = (x + dx) % self.config['world_width']
        new_y = (y + dy) % self.config['world_height']
        self.agent.set_position(new_x, new_y, new_heading)
        #current_time = pygame.time.get_ticks()  # Get the current time in milliseconds
        #self.agent.trajectory.append((new_x, new_y, current_time))
        self.agent.trajectory.append((new_x, new_y))
        # Save the image
        self.agent.save_information(None)

    def calculate_fitness(self):
        # Implement your fitness calculation here. This is just a placeholder.
        return len(set(self.agent.trajectory))  # Fitness is the number of unique positions visited

    def torus(self):
        # Get current robot position and heading
        robot_position_x, robot_position_y, robot_heading = self.agent.get_position()

        # Implement torus world by manipulating the robot position here

        # Update robot's position and heading
        self.agent.set_position(robot_position_x, robot_position_y, robot_heading)