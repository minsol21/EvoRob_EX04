import numpy as np
import math

class Evolution():
    def __init__(self, config, agent, environment):
        self.config = config
        self.agent = agent
        self.environment = environment
        self.genome = np.random.uniform(-1, 1, 6)  # Initialize genome with random values
        self.best_genome = self.genome.copy()
        self.best_fitness = -np.inf

    def evaluate_genome(self):
        # Reset agent's position and heading
        self.agent.reset()

        # Run the agent for a certain period of time
        for _ in range(self.config['evaluation_steps']):
            # Get sensor readings
            sl, sm, sr = self.agent.get_perception()

            # Calculate wheel speeds based on genome
            vl = self.genome[0]*sl + self.genome[1]
            vr = self.genome[2]*sr + self.genome[3] + self.genome[4]*sm + self.genome[5]

            # Adjust wheel speeds to make the robot move forward, backward, or turn
            if vl > vr:
                # If the left wheel is moving faster than the right wheel, make the robot turn right
                vr += abs(vl - vr)
            elif vr > vl:
                # If the right wheel is moving faster than the left wheel, make the robot turn left
                vl += abs(vr - vl)
            else:
                # If both wheels are moving at the same speed, make the robot move forward
                vl += 1
                vr += 1

            # Update agent's position based on wheel speeds
            self.agent.update_position(vl, vr)

        # Calculate fitness as the number of unique grid cells visited
        fitness = len(set(self.agent.trajectory))
        return fitness

    def run(self):
        for _ in range(self.config['generations']):
            # Evaluate current genome
            fitness = self.evaluate_genome()

            # If fitness has improved, keep the current genome
            if fitness > self.best_fitness:
                self.best_fitness = fitness
                self.best_genome = self.genome.copy()
            else:
                # If fitness has worsened, revert to the previous genome
                self.genome = self.best_genome.copy()

            # Mutate the genome to produce a new candidate
            self.genome += np.random.normal(0, 0.1, 6)