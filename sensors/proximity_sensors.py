from swarmy.perception import Perception
import pygame
import math
import numpy as np


class ProximitySensor(Perception):
    def __init__(self, agent, environment, config):
        super().__init__(agent, environment)
        self.agent = agent
        self.environment = environment
        self.config = config
        self.width = config['world_width']

    def sensor(self):
        robot_position_x, robot_position_y, robot_heading = self.agent.get_position()
        rob_pos = pygame.Vector2(robot_position_x, robot_position_y)
        reach = 0.15*self.width #The range r of our sensors should be about 15% of the square length. 

        sensor_1_direction_x_l = math.sin(math.radians(robot_heading + 40))
        sensor_1_direction_y_l = math.cos(math.radians(robot_heading + 40))
        sensor_2_direction_x_r = math.sin(math.radians(robot_heading - 40))
        sensor_2_direction_y_r = math.cos(math.radians(robot_heading - 40))
        sensor_3_direction_x_m = math.sin(math.radians(robot_heading))
        sensor_3_direction_y_m = math.cos(math.radians(robot_heading))

        sensor_l_pos = [(robot_position_x+(sensor_1_direction_x_l*reach)),
                   (robot_position_y+(sensor_1_direction_y_l*reach))]
        sensor_r_pos= [(robot_position_x+(sensor_2_direction_x_r*reach)),
                  (robot_position_y+(sensor_2_direction_y_r*reach))]
        sensor_m_pos = [(robot_position_x+(sensor_3_direction_x_m*reach)),
                  (robot_position_y+(sensor_3_direction_y_m*reach))]

        sensor_color = (255, 0, 0)

        sensor_l = [sensor_color, rob_pos, sensor_l_pos]
        sensor_r = [sensor_color, rob_pos, sensor_r_pos]
        sensor_m = [sensor_color, rob_pos, sensor_m_pos]

        self.environment.add_dynamic_line_object(sensor_l)
        self.environment.add_dynamic_line_object(sensor_r)
        self.environment.add_dynamic_line_object(sensor_m)

        # helper to transform lines to rectange objects, allows for collision detection
        helper_object_l = pygame.draw.line(self.agent.environment.displaySurface, sensor_color, rob_pos, sensor_l_pos)
        helper_object_r = pygame.draw.line(self.agent.environment.displaySurface, sensor_color, rob_pos, sensor_r_pos)
        helper_object_m = pygame.draw.line(self.agent.environment.displaySurface, sensor_color, rob_pos, sensor_m_pos)

        # list of all objects in the environment
        objects = self.environment.get_agent_object() + [wall[1] for wall in self.environment.get_static_rect_list()]


        # check the distance to the nearest object for each sensor
        min_distance_l = float('inf')
        min_distance_r = float('inf')
        min_distance_m = float('inf')
        for idx, line in enumerate(objects):
            if idx == self.agent.unique_id:  # skips the robot itself
                continue

            intersection_l = np.asarray(line.clip(helper_object_l))
            intersection_r = np.asarray(line.clip(helper_object_r))
            intersection_m = np.asarray(line.clip(helper_object_m))
            if intersection_l[2] > 0:
                distance = np.linalg.norm(intersection_l[:2] - np.array(rob_pos))
                min_distance_l = min(min_distance_l, distance)
            if intersection_r[2] > 0:
                distance = np.linalg.norm(intersection_r[:2] - np.array(rob_pos))
                min_distance_r = min(min_distance_r, distance)
            if intersection_m[2] > 0:
                distance = np.linalg.norm(intersection_m[:2] - np.array(rob_pos))
                min_distance_m = min(min_distance_m, distance)

        return min_distance_l if min_distance_l != float('inf') else 0, \
               min_distance_r if min_distance_r != float('inf') else 0, \
               min_distance_m if min_distance_m != float('inf') else 0