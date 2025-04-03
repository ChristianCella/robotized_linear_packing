''' 
What this code does.
'''

import random
import os
import sys
import numpy as np

# Get the full path to the text file
script_dir = os.path.dirname(os.path.realpath(__file__)) # Get absolute path of the current script
project_root = os.path.abspath(os.path.join(script_dir, "..")) # Go up one folder to "robotized_linear_packing"
save_path = os.path.join(project_root, "data", "travel_time.txt") # Build full path to bounds_experiment.txt

# Append the path for utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.motion_planning import MotionPlanner
from utils.parameters import TravelSimulationParameters


def main():

    params = TravelSimulationParameters()
    i = 0
    travel_times = []

    # Initialize the text file for the results
    with open(save_path, 'w') as f: f.write("Travel times for the problem\n")

    # Initiate the class
    motion_planner = MotionPlanner(params.v_max, params.a_max)
    
    #loop for the number of samples
    for i in range(0, params.N_iter):

        # Generate random start and goal positions
        start_pos = random.uniform(params.lower_bound, params.upper_bound)
        end_pos =  random.uniform(params.lower_bound, params.upper_bound)

        # Compute the time
        t = motion_planner.trapezoidal_velocity_profile(start_pos, end_pos)
        if params.verbose: print(f"Iteration: {i}; Time taken: {t} seconds")
        travel_times.append(t)

        #inserisci nel file di testo
        with open(save_path, "a") as f: f.write(f"Iteration: {i}; Time taken: {t} seconds \n")

    # Compute the mean and variance of the travel times
    mean = np.mean(travel_times)
    std = np.std(travel_times)

    # Save the results in the text file
    with open(save_path, "a") as f: f.write(f"********** The mean time is: {mean} seconds; The standard deviation is: {std} seconds *********\n")

if __name__ == "__main__":

    # Run the code
    main()
            





        
        