from swarmy.agent import Agent
import random
import pygame

class MyAgent(Agent):
    def __init__(self,environment,controller, sensor, config):
        super().__init__(environment,controller, sensor, config)

        self.environment = environment
        self.trajectory = []
        self.eval_params = []
        self.current_generation = 0
        self.wheel_distance = 0 



    def initial_position(self):
        """
        Define the initial position of the agent.
        Hint:
        Use x,y,gamma = self.set_position(x-position, y-position, heading) to set the position of the agent.
        """
        x = random.randint(0, self.config['world_width'])
        y = random.randint(0, self.config['world_height'])

        gamma = random.randint(0, 360)
        self.actuation.position[0] = x
        self.actuation.position[1] = y
        self.actuation.angle = gamma
        self.set_position(x, y, gamma)




    def save_information(self, last_robot):
        """
        Save information of the agent, e.g. trajectory or the environmental plot.
        Hint:
        - Use pygame.draw.lines() to draw the trajectory of the robot and access the surface of the environment with self.environment.displaySurface
        - pygame allows to save an image of the current environment
        """
        #print("Save information not implemented, check my_agent.py")
        """ your implementation here """
         # Draw the trajectory
        self.draw_trajectory()

        # Save the image to the local disk
        pygame.image.save(self.environment.displaySurface, "trajectory.png")
        pass

    def draw_trajectory(self):
        self.trajectory = [point for point in self.trajectory]
        
        if len(self.trajectory) > 1:  # Ensure there are at least two points to draw a line
            positions = [(pos[0], pos[1]) for pos in self.trajectory]
            pygame.draw.lines(self.environment.displaySurface, (255, 0, 0), False, positions, 2)



    def set_evaluation_params (self, eval_params):
        self.eval_params = eval_params

    def get_evaluation_params(self):
        return self.eval_params







