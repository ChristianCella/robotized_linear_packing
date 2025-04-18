''' 
Python side of the code for the robotized linear packing algorithm.
'''

import os
import socket
import numpy as np
import os
import time

from utils.motion_planning import MotionPlanner 
from utils.parameters import RealSimulationParameters
from utils.packing import PackingListCreator
from utils.socket_manager import send_array, send_strings, recv_msg
params = RealSimulationParameters()

# Get the full path to the text file
save_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", params.file_name)

def main():

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((params.host, params.port))
    s.settimeout(params.timeout) # if nothing is received within 60 seconds => timeout

    with open(save_path, 'w') as f: f.write(f"Data of the complete use case \n\n")

    # Create the packing scenario
    packing_creator = PackingListCreator(params)
    packing_creator.create()

    # Parameters to be initilaized
    base_position_sequence = [] # sequence of the base positions
    optimal_sequence = [] # sequence of the objects to be packed (pick side)
    optimal_set = [] # sequence of the parameters to be used for the pick&place
    optimal_travel_times = [] # travel times for the base position
    optimal_pp_times = [] # sequence that is updated with the p&p time of the selected object
    total_times = []
    cumulative_total_time = 0
    cumulative_pick_and_place_time = 0 
    cumulative_travel_time = 0
    current_base_pos = 0
    pp_time = 0 # Time associated to the best position found for a specific object
    best_parameters_pp = 0 # Parameters associated to the best position found for a specific object
    keep_travel_time = 0 # temporary variable that is overwritten in the loop  
    global_best_position = 0
    global_best_score = params.max_cost # Re-initilize the cost
    type_obj = 0

    # Instantiate the class for the motion planning   
    motion_planner = MotionPlanner(params.v_max, params.a_max)

    # Send a cascade of messages to the C# side
    send_array(s, np.array([[params.N_sim_pso, params.N_particles, params.pre_post_height, params.n_decimals, params.mean_travel_time]], dtype = np.int32))
    send_array(s, np.array([params.mean_time_pp], dtype = np.int32))
    send_array(s, np.array([params.items_of_each_type], dtype = np.int32))
    send_array(s, np.array([params.bins_of_each_type], dtype = np.int32))
    send_strings(s, [params.items_root_name, params.robot_program_name, params.robot_name,
                      params.tool_name, params.tcp_ee_name, params.tcp_flange_name, params.human_name])

    # * STEP 1 => for all the types of objects you have ...
    while type_obj < len(params.items_of_each_type):

        # Get iteration specific parameters
        bin = 0
        pick_objects = [] # array containing the index of objects (pick side) already picked and placed
        with open (save_path, 'a') as f: f.write(f"********* Type of item: {type_obj} \n")

        # * STEP 2 => for all the bins available for a specific type of object ...
        while bin < params.bins_of_each_type[type_obj]: 
            
            # Get iteration specific parameters
            place_points = []
            rotations = []
            k = 0 # counter for number of objectsb that can be placed in a box
            i = 0  # counter to loop through the num_objects = k 'place-side' objects

            # get the coordinates of the lower-left corner of the bin
            x_offset_box, y_offset_box, z_offset_box = packing_creator.packers[type_obj].bins[bin].get_offset()  

            # Get the place points
            for idx1 in range(len(packing_creator.packers[type_obj].bins[bin].items)):
                place_points.append(packing_creator.packers[type_obj].bins[bin].items[idx1].get_center())
                rotations.append(packing_creator.packers[type_obj].bins[bin].items[idx1].rotation_type) 
                k += 1

            num_objects = k # number of objects inside the bin ('spots' available for objects in a bin)
            send_array(s, np.array([[num_objects]], dtype = np.int32))           
            with open (save_path, 'a') as f: f.write(f"\t Considered bin number: {bin} \n")

            # * STEP 3 => for all the objects ('place-side') that can be packed in a specific bin ...            
            while i < num_objects:

                with open (save_path, 'a') as f: f.write(f"\t \t Considering the 'place-side' obejct number: {i}/{k} \n")
                with open (save_path, 'a') as f: f.write(f"\t \t Cumulative total time: {cumulative_total_time}, of which: {cumulative_pick_and_place_time} for the P&P, while {cumulative_travel_time} for the travel.\n")

                # send the 'absolute' coordinates of the 'place position' associated to the item to be packed (assume all items identical)
                place_x = place_points[i][0] + x_offset_box
                place_y = place_points[i][1] + y_offset_box
                place_z = place_points[i][2] + z_offset_box + params.items_sizes_and_weight[type_obj][2]/2
                rotation = rotations[i]
                cum_time_to_send = int((round(cumulative_total_time, params.n_decimals) * (10 ** params.n_decimals)))
                send_array(s, np.array ([[place_x, place_y, place_z, rotation, cum_time_to_send]], dtype = np.int32))

                # Initialize variables for the PSO algorithm
                c = 0 # index of the object to be picked (pick side) 
                best_tradeoff = params.max_cost # Re-initilize the cost
              
                # * STEP 4 => for all the items that can be picked (pick side) after knowing a specific 'place-side' spot ...
                while c < params.items_of_each_type[type_obj]: 

                    '''
                    Core of the optimization procedure
                    * The available 'pick-side' objects are scanned (c is the index) 
                    * The one that minimizes the fitness function is chosen and removed from the list
                    * The PSO algorithm is applied to find the best position for the base of the robot
                    * The best position is chosen according to the trade-off between manipulability and time
                    '''

                    # If there is an object to be picked (pick side) and it has not been picked yet
                    if c not in pick_objects: 
                        
                        # Initialize the PSO parameters
                        send_array(s, np.array([[0]], np.int32)) # skip = 0 => Do not skip and test this object
                        trigger_end = 0 # Iteration counter for PSO algorithm

                        # Initialization of the pso particles 
                        particle_positions = np.random.uniform(params.base_lower_bound, params.base_upper_bound, params.N_particles)
                        particle_velocities = np.random.uniform(params.vel_lower_bound, params.vel_upper_bound, params.N_particles) 
                                              
                        with open(save_path, 'a') as f: f.write(f"\t \t \t 'Pick-side' obejct number: {c} \n")
                        with open(save_path, 'a') as f: f.write(f"\t \t \t ***** Starting the PSO ***** \n")
                        
                        '''
                        PSO algorithm => Find the robot base for each object to be picked
                        '''
                        while trigger_end < params.N_sim_pso:

                            # * Update inertia and layout
                            inertia_weight = params.w0 + params.delta_w * trigger_end
                            send_array(s, np.array([[int(particle_positions[q]) for q in range(params.N_particles)]], dtype = np.int32))

                            # ! TPS is evaluating manipulability, time, feasibility for each particle

                            try: # ? If everything is fine, the results from TPS are recived

                                # receive the results from the simulator (NOTE: one after the other!)
                                result_flag = recv_msg(s)
                                result_col = recv_msg(s)
                                result_manip = recv_msg(s)
                                result_time_pp = recv_msg(s)
                                set_parameters_pp = recv_msg(s)

                                # Convert the results to the appropriate format
                                flag = [int(num) for num in result_flag.split(',')]
                                if params.verbose: print(f"@ PSO iteration:{trigger_end}, the successful particles are: {flag}")

                                col = [int(num) for num in result_col.split(',')]
                                if params.verbose: print(f"@ PSO iteration:{trigger_end}, the collisions are: {col}")
                               
                                mean_determinant = [params.max_cost if int(num) == 1 else int(num) / (10 ** params.n_decimals)
                                    for num in result_manip.split(',')]
                                if params.verbose: print(f"@ PSO iteration:{trigger_end}, the determinants are: {mean_determinant}")
                               
                                execution_time = [params.max_cost if int(num) == 1 else int(num) / (10 ** params.n_decimals)
                                    for num in result_time_pp.split(',')]
                                print(f"Execution time: {execution_time}")
                                if params.verbose: print(f"@ PSO iteration:{trigger_end}, the execution times are: {execution_time}")

                                parameters_pp = [int(num) for num in set_parameters_pp.split(',')]
                                if params.verbose: print(f"@ PSO iteration:{trigger_end}, set of parameters: {parameters_pp}")

                            except socket.timeout: # ? TPS did not compute a response within the timeout period => break
                                print("Timeout waiting for simulator response. Breaking PSO loop.")
                                with open(save_path, 'a') as f: f.write("Timeout during the receptiojn of data. \n")
                                break 

                            # Compute the Z-scores
                            xi = [0 if num == 0 else params.max_cost for num in flag] 
                            eta = [0 if num == 0 else params.max_cost for num in col] 
                            z_m = []
                            z_t = []

                            for val in mean_determinant:
                                z_m.append((1 / val - params.mean_manip[type_obj]) / params.std_manip[type_obj])
                            
                            for val in execution_time:
                                z_t.append((val - params.mean_time_pp[type_obj]) / params.std_time_pp[type_obj])

                            z_t = np.array(z_t)
                            z_m = np.array(z_m)
                            xi = np.array(xi)
                            eta = np.array(eta)

                            # Compute the vector of fitness values: alpha * z_t + beta * z_m + xi + eta (NOTE: minimize!)
                            fitness = params.alpha_fitness * z_t + params.beta_fitness * z_m + xi + eta

                            # Update the counter for the PSO iteration
                            trigger_end += 1
                            with open (save_path, 'a') as f: f.write(f"\t \t \t \t PSO iteration: {trigger_end}; the fitness values are: {fitness} \n")

                            # * Set personal best and global best (the mask allows to remove a for loop)
                            if trigger_end == 1:
                                print(f"Checkpoint 1: first PSO iteration")
                                personal_best_positions = particle_positions.copy()
                                personal_best_scores = fitness.copy()

                                global_best_position = personal_best_positions[np.argmin(personal_best_scores)]                               
                                global_best_score = np.min(personal_best_scores)

                            else :
                                # Vectorized update of personal bests
                                print(f"Checkpoint 2: no more the first iteration")
                                better_mask = fitness < personal_best_scores
                                personal_best_positions[better_mask] = particle_positions[better_mask]
                                personal_best_scores[better_mask] = fitness[better_mask]

                                # Update global best
                                min_index = np.argmin(fitness)
                                print(f"The minimum index is: {min_index} and it corresponds to the fitness value {fitness[min_index]}") # ! REMOVE
                                if fitness[min_index] < global_best_score:
                                    global_best_position = particle_positions[min_index]
                                    global_best_score = fitness[min_index]
                                    pp_time = execution_time[min_index] # Time associated to the best position found
                                    best_parameters_pp = parameters_pp[min_index]
                                    print(f"The parameter pp_time is: {pp_time}") # ! REMOVE
                                    with open (save_path, 'a') as f: f.write(f"\t \t \t \t \t New global best score found. Base position: {global_best_position} \n")
                                    with open (save_path, 'a') as f: f.write(f"\t \t \t \t \t Fitness value: {global_best_score} \n")
                                    with open (save_path, 'a') as f: f.write(f"\t \t \t \t \t Pick&Place time: {pp_time} \n")
                                    with open (save_path, 'a') as f: f.write(f"\t \t \t \t \t Parameters set: {best_parameters_pp} \n")

                            # * Update particles
                            # Random vectors for cognitive and social components
                            r1 = np.random.random(size = params.N_particles)
                            r2 = np.random.random(size = params.N_particles)

                            # Vectorized velocity update
                            inertia = inertia_weight * particle_velocities
                            cognitive = params.c1 * r1 * (personal_best_positions - particle_positions)
                            social = params.c2 * r2 * (global_best_position - particle_positions)
                            particle_velocities = inertia + cognitive + social

                            # Update positions (convertion to float needed for the velocities)
                            particle_positions = particle_positions.astype(float)
                            particle_positions += particle_velocities

                            # Convert back to int
                            particle_positions = particle_positions.astype(int)

                            # Clip positions within bounds
                            particle_positions = np.clip(particle_positions, params.base_lower_bound, params.base_upper_bound)

                        # The PSO for a specific 'pick-side' object is over                       
                        with open(save_path, 'a') as f: f.write(f"\t \t \t ***** PSO ended for 'Pick-side' object number  {c} ***** \n")
                        
                        # ? Time-manipulability trade-off for the sequence update                                               
                        travel_time = motion_planner.trapezoidal_velocity_profile(current_base_pos, global_best_position)  
                        tradeoff = travel_time
                        
                        # Check the improvement
                        if (tradeoff < best_tradeoff):
                            best_tradeoff = tradeoff
                            next_position = global_best_position
                            next_item = c 
                            keep_travel_time = travel_time  
                            keep_pp_time = pp_time 
                            best_set = best_parameters_pp                  
                            if params.verbose: print(f"Checkpoint: computed the motion law for the base")
                        
                    else: # the 'c-th' object has already been picked and placed, so I skip it
                        skip = np.array([[1]], dtype = np.int32) # skip = 1 => Skip
                        send_array(s, skip)
                        if params.verbose: print(f" Skipping the object {c} because it has already been picked and placed")

                    # Update the counter
                    c = c + 1 # go to the next object (pick side) to be tested
                    if params.verbose: print(f"counter c = {c}")

                # * All 'pick-side' items scanned: I know the object to be picked (sequence) and where to put the robot (base position)                
                current_base_pos = next_position 
                pick_objects.append(next_item) # Remove the c-th item from the available ones pick-side (this list will be re-initialized)
                
                # Send the index of the object to color in LightBlue
                send_array(s, np.array([[next_item]], dtype = np.int32))

                optimal_sequence.append(next_item) # Augment the sequence of objects to be packed (pick side)
                base_position_sequence.append(current_base_pos) # Augment the sequence of base positions
                optimal_set.append(best_set)
                optimal_travel_times.append(keep_travel_time) # Augment the travel time sequence
                optimal_pp_times.append(keep_pp_time) # pick&place time associated to the selected object
                total_times.append(keep_travel_time + keep_pp_time) # Augment the sequence of total times (not 'absolute' but relative to the previous one)
                cumulative_pick_and_place_time = sum(optimal_pp_times) 
                cumulative_travel_time = sum(optimal_travel_times)
                cumulative_total_time = sum(total_times)
                print(f"The cumulative time for picking and placing is: {cumulative_pick_and_place_time} \n")
                print(f"The cumulative time for the travel is: {cumulative_travel_time} \n")
                print(f"The cumulative time for the total is: {cumulative_total_time} \n")

                with open(save_path, 'a') as f: f.write(f"\t \t All 'Pick-side' objects scanned. The best one is: {next_item} \n")
                with open(save_path, 'a') as f: f.write(f"\t \t The robot is moved to: {current_base_pos} mm wrt the center \n")
                with open(save_path, 'a') as f: f.write(f"\t \t The optimal set of parameters is: {optimal_set} \n")
                if params.verbose: print(f"Checkpoint: Pick-side objects scanned")
                
                # * Update the counter of objects packed so far inside the bin-th box
                i = i + 1
                with open(save_path, 'a') as f: f.write(f"\t \t Still need to pack {k-i} objects in the bin {bin} \n")
                if params.verbose: print(f"Checkpoint: Still need to pack {k-i} objects in the bin number {bin}")

            # * A box for a specific type of object is full: I go to the next one available
            with open(save_path, 'a') as f: f.write(f"\t All objects packed in the bin number {bin} \n") 
            bin = bin + 1   
            if params.verbose: print(f"Checkpoint: All objects packed in the bin {bin}")
            
        # * All items of a specific type are packed: I go to the next type of object
        with open(save_path, 'a') as f: f.write(f"All objects of type {type_obj} have been packed \n")
        type_obj = type_obj + 1
        if params.verbose: print(f"Checkpoint: All objects of type {type_obj} have been packed")

    # Close the connection
    s.close()

    # Save the results in a text file
    with open(save_path, 'a') as f: f.write(f"********** Results ************ \n")
    with open(save_path, 'a') as f: f.write(f"The optimal sequence is: {optimal_sequence} \n")
    with open(save_path, 'a') as f: f.write(f"The optimal base positions are: {base_position_sequence} \n")
    with open(save_path, 'a') as f: f.write(f"The optimal set of robot parameters is: {optimal_set} \n")

# Run the code
if __name__ == "__main__":

    # Run the code
    main()
            
