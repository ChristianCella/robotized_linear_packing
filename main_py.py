''' 
Python side of the code for the robotized linear packing algorithm.
'''

import sys
import os
import struct

# Set the path to the 3D bin packing library
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item, Scene

from utils.motion_planning import MotionPlanner 
from utils.parameters import RealSimulationParameters
from utils.packing import PackingListCreator
from utils.socket_manager import send_array, send_strings, recv_msg
params = RealSimulationParameters()

# Get the full path to the text file
script_dir = os.path.dirname(os.path.realpath(__file__)) # Get absolute path of the current script
save_path = os.path.join(script_dir, "data", params.file_name) # Build full path to bounds_experiment.txt

import socket
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time

# Create useful lists

# Parameters to be initilaized
best_tradeoff = params.max_cost
base_position_sequence = [] 
optimal_sequence = [] # sequence of the objects to be packed (pick side)
items = []
personal_best_positions = []
personal_best_scores = []

# Create the packing condition
packing_creator = PackingListCreator(params)
packing_creator.create()

def main():

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((params.host, params.port))
    s.settimeout(60.0) # Raise a socket timeout exception if no data is received within 3 seconds

    with open(save_path, 'w') as f: f.write(f"Data of the complete use case \n")

    # Instantiate the class for the motion planning
    current_pos = 0
    global_best_position = 0
    global_best_score = 0
    motion_planner = MotionPlanner(params.v_max, params.a_max)

    # ! Send 1: shared bumbers
    send_array(s, np.array([[params.N_sim_pso, params.N_particles, params.pre_post_height, params.n_decimals]], dtype = np.int32))
    time.sleep(0.005)

    # ! send shared list for items
    send_array(s, np.array([[3, 3]], dtype = np.int32))
    time.sleep(0.005)

    # ! send shared list for bins
    send_array(s, np.array([[1, 1]], dtype = np.int32))
    time.sleep(0.005)

    # ! Send 2: shared strings
    send_strings(s, [params.items_root_name, params.bins_root_name, params.robot_program_name])
    time.sleep(0.005)

    type_obj = 0

    # * STEP 1 => for all the types of objects you have ...
    while type_obj < len(params.items_of_each_type):

        # Get iteration specific parameters
        bin = 0
        pick_objects = [] # array containing the index of objects (pick side) already picked and placed
        with open (save_path, 'a') as f: f.write(f"********* Type of item: {type_obj} \n")

        # * STEP 2 => for all the bins available for a specific type of object ...
        while bin < params.bins_of_each_type[type_obj]: 

            place_points = []
            rotations = []
            k = 0
            x_offset_box, y_offset_box, z_offset_box = packing_creator.packers[type_obj].bins[bin].get_offset()  # coordinates of the lower-left corner of the bin

            # Get the place points
            for idx1 in range(len(packing_creator.packers[type_obj].bins[bin].items)):
                place_points.append(packing_creator.packers[type_obj].bins[bin].items[idx1].get_center())
                rotations.append(packing_creator.packers[type_obj].bins[bin].items[idx1].rotation_type) 
                k += 1

            num_objects = k # number of objects inside the bin ('spots' available for objects in a bin)
            num_objects_send = np.array([[num_objects]], dtype = np.int32) 
            send_array(s, num_objects_send)
           
            with open (save_path, 'a') as f: f.write(f"\t Considered bin number: {bin} \n")

            # * STEP 3 => for all the objects ('place-side') that can be packed in a specific bin ...
            i = 0 
            while i < num_objects:

                with open (save_path, 'a') as f: f.write(f"\t \t 'Place-side' obejct number: {i} \n")

                # send the 'absolute' coordinates of the 'place position' associated to the item to be packed (assume all items identical)
                place_x = place_points[i][0] + x_offset_box
                place_y = place_points[i][1] + y_offset_box
                place_z = place_points[i][2] + z_offset_box + params.items_sizes_and_weight[type_obj][2]/2
                rotation = rotations[i] # Also send the rotation
                place_point_send = np.array ([[place_x, place_y, place_z, rotation]], dtype = np.int32)
                send_array(s, place_point_send)

                # Initialize variables for the PSO algorithm
                c = 0 # initilaize the index of the object to be picked (pick side) 
                best_tradeoff = params.max_cost # Re-initilize the cost
              
                # * STEP 4 => for all the items that can be picked (pick side) after knowing a specific 'place-side' spot ...
                while c < params.items_of_each_type[type_obj]: 

                    '''
                    Core of the optimization procedure
                    * The available 'pick-side' objects are scanned (c is the index), one is chosen and removed from the list
                    * The PSO algorithm is applied to find the best position for the base of the robot
                    * The best position is chosen according to the trade-off between manipulability and time
                    '''

                    if c not in pick_objects: 
                        
                        # Initialize the PSO parameters
                        skip = np.array([[0]], np.int32) # skip = 0 => Do not skip and test this object
                        send_array(s, skip)
                        trigger_end = 0 # Iteration counter for PSO algorithm

                        # Initialization of the pso particles 
                        particle_positions = np.random.uniform(params.lower_bound, params.upper_bound, params.N_particles)
                        particle_velocities = np.random.uniform(-1, 1, params.N_particles) 
                                              
                        with open(save_path, 'a') as f: f.write(f"\t \t \t 'Pick-side' obejct number: {c} \n")
                        with open(save_path, 'a') as f: f.write(f"\t \t \t ***** Starting the PSO ***** \n")
                        
                        '''
                        PSO algorithm 
                        '''
                        while trigger_end < params.N_sim_pso:

                            # * Update inertia and layout
                            inertia_weight = params.w0 + params.delta_w * trigger_end
                            layout = np.array([[int(particle_positions[q]) for q in range(params.N_particles)]], dtype=np.int32)
                            print(f"PSO iteration: {trigger_end}; Particle positions: {particle_positions}")
                            send_array(s, layout)

                            
                            ''' 
                            TPS computes the fitness values for all particles (C# side)
                            '''

                            

                            # ! Get all the vectors
                            try:
                                result_flag = recv_msg(s)
                                result_manip = recv_msg(s)
                                result_time_pp = recv_msg(s)



                                flag = [int(num) for num in result_flag.split(',')]
                                print(f"Flag: {flag}")

                                
                                mean_determinant = [params.max_cost if int(num) == 1 else int(num) / (10 ** params.n_decimals)
                                    for num in result_manip.split(',')]
                                print(f"Mean determinant: {mean_determinant}")

                                
                                execution_time = [params.max_cost if int(num) == 1 else int(num) / (10 ** params.n_decimals)
                                    for num in result_time_pp.split(',')]
                                print(f"Execution time: {execution_time}")
                            except socket.timeout:
                                print("Timeout waiting for simulator response. Breaking PSO loop.")
                                with open(save_path, 'a') as f:
                                    f.write("Timeout during recv() — simulator unresponsive.\n")
                                break 

                            # Compute the vector of fitness values: alpha * z_t + beta * (1 / z_m) + xi (MINIMIZE THIS)
                            xi = [0 if num == 0 else params.max_cost for num in flag] 
                            z_m = []
                            for val in mean_determinant:
                                if val == params.max_cost:
                                    z_m.append(1)  # Or whatever default you want
                                else:
                                    z = (val - params.mean_manip[type_obj]) / params.std_manip[type_obj]
                                    z_m.append(z)
                            z_t = []
                            for val in execution_time:
                                if val == params.max_cost:
                                    z_t.append(1)  # Or whatever default you want
                                else:
                                    z = (val - params.mean_time_pp[type_obj]) / params.std_time_pp[type_obj]
                                    z_t.append(z)
                            z_t = np.array(z_t)
                            z_m = np.array(z_m)
                            xi = np.array(xi)
                            fitness = params.alpha_fitness * z_t + params.beta_fitness * (1 / z_m) + xi
                            trigger_end += 1
                            with open (save_path, 'a') as f: f.write(f"\t \t \t \t PSO iteration: {trigger_end}; \n")

                            # * Set personal best and global best (the mask allows to remove a for loop)
                            if trigger_end == 1:

                                personal_best_positions = particle_positions.copy()
                                personal_best_scores = fitness.copy()

                                global_best_position = personal_best_positions[np.argmin(personal_best_scores)]                               
                                global_best_score = np.min(personal_best_scores)

                            else :
                                # Vectorized update of personal bests
                                better_mask = fitness < personal_best_scores
                                personal_best_positions[better_mask] = particle_positions[better_mask]
                                personal_best_scores[better_mask] = fitness[better_mask]

                                # Update global best
                                min_index = np.argmin(fitness)
                                if fitness[min_index] < global_best_score:
                                    global_best_position = particle_positions[min_index]
                                    global_best_score = fitness[min_index]
                                    with open (save_path, 'a') as f: f.write(f"\t \t \t \t \t New global best found. Base position: {global_best_position}; Fitness value: {global_best_score} \n")

                            # * Update particles
                            # Random vectors for cognitive and social components
                            r1 = np.random.random(size = params.N_particles)
                            r2 = np.random.random(size = params.N_particles)

                            # Vectorized velocity update
                            inertia = inertia_weight * particle_velocities
                            cognitive = params.c1 * r1 * (personal_best_positions - particle_positions)
                            social = params.c2 * r2 * (global_best_position - particle_positions)
                            particle_velocities = inertia + cognitive + social

                            # Update positions
                            particle_positions = particle_positions.astype(float)
                            particle_positions += particle_velocities

                            # Convert to int
                            particle_positions = particle_positions.astype(int)

                            # Clip positions within bounds
                            particle_positions = np.clip(particle_positions, params.lower_bound, params.upper_bound)
                              

                        
                        with open(save_path, 'a') as f: f.write(f"\t \t \t ***** PSO ended for 'Pick-side' object number  {c} ***** \n")
                        
                        #Time-manipulability trade-off for the sequence update
                                                
                        travel_time = motion_planner.trapezoidal_velocity_profile(current_pos, global_best_position)  
                        tradeoff = params.alpha_tradeoff * (1 / ((global_best_score - params.mean_manip[type_obj]) / params.std_manip[type_obj])) + params.beta_tradeoff * (((travel_time - params.mean_travel_time) / params.std_travel_time)) 
                        
                        if (tradeoff < best_tradeoff):
                            best_tradeoff = tradeoff
                            next_position = global_best_position
                            next_item = c
                            with open (save_path, 'a') as f: f.write(f"\t \t \t New best trade-off found. Base position: {next_position}; Trade-off value: {tradeoff} \n")
                            print(f"Checkpoint: computed the motion law for the base")
                        
                    else: # the 'c-th' object has already been picked and placed, so I skip it
                        skip = np.array([[1]], dtype = np.int32) # skip = 1 => Skip
                        send_array(s, skip)
                        print(f" Skipping the object {c} because it has already been picked and placed")

                    # Update the counter
                    c = c + 1 # go to the next object (pick side) to be tested
                    print(f"counter c = {c}")

                # * All 'pick-side' items scanned: I know the object to be picked (sequence) and where to put the robot (base position)
                
                current_pos = next_position 
                pick_objects.append(next_item)
                optimal_sequence.append(next_item)
                base_position_sequence.append(current_pos)
                with open(save_path, 'a') as f: f.write(f"\t \t All 'Pick-side' objects scanned. The best one is: {next_item}; the robot is moved to {current_pos} \n")
                print(f"Checkpoint: Pick-side objects scanned")
                
                # * Update the counter of objects packed so far inside the bin-th box
                i = i + 1
                with open(save_path, 'a') as f: f.write(f"\t \t Still need to pack {k-i} objects in the bin {bin} \n")
                print(f"Checkpoint: Still need to pack {k-i} objects in the bin {bin}")

            # * A box for a specific type of object is full: I go to the next one available
            with open(save_path, 'a') as f: f.write(f"\t All objects packed in the bin {bin} \n") 
            bin = bin + 1   
            print(f"Checkpoint: All objects packed in the bin {bin}")
            
        # * All items of a specific type are packed: I go to the next type of object
        with open(save_path, 'a') as f: f.write(f"All objects of type {type_obj} have been packed \n")
        type_obj = type_obj + 1
        print(f"Checkpoint: All objects of type {type_obj} have been packed")

    # Close the connection
    s.close()

    # Save the results in a text file
    with open(save_path, 'a') as f: f.write(f"********** Results ************ \n")
    with open(save_path, 'a') as f: f.write(f"The optimal sequence is: {optimal_sequence} \n")
    with open(save_path, 'a') as f: f.write(f"The optimal base positions are: {base_position_sequence} \n")

# Run the code
if __name__ == "__main__":

    # Run the code
    main()
            
