''' 
Python side of the code for the robotized linear packing algorithm.
'''

import sys
import os

# Get the path to the 3dbinpacking folder (look at the readme to understand how the structure should be)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '3dbinpacking'))
sys.path.append(project_root)

from py3dbp import Packer, Bin, Item, Scene

import socket
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import time

from utils.motion_planning import MotionPlanner 

# Define the socket parameters
host = '127.0.0.1'
port = 12345

# Parameters for the seventh axis
v_base = 700 # [mm/s]
a_base = 900 # [mm/s^2]
upper_bound = 290 # [mm]
lower_bound = -300 # [mm]

# PSO parameters
Nsim = 50 # Number of simulations (PSO)
num_particles = 20 # Number of particles in the swarm
w0 = 0.9 # Upper bound on inertia
wN = 0.4 # Lower bound on inertia
delta_w = (wN - w0) / Nsim # Inertia weight decrease at each iteration
cognitive_component = 2
social_component = 2 

# Parameters for the packintg algorithm
num_types = 2 # Different types of objects
num_objects_0 = 3 # objects of type 0
num_objects_1 = 3 # objects of type 1
num_bin_0 = 1 # N° of bins of type 0
num_bin_1 = 1 # N° of bins of type 1

num_bins_array = [num_bin_0, num_bin_1]
num_objects_array = [num_objects_0, num_objects_1]

# Parameters obtained with the baseline experiment: manipulabiity #! The values are for sure not correct
mean_man1 = 10523.872909698997
mean_man2 = 10979.224080267559
mean_man_vec = [mean_man1, mean_man2]

var_man1 = math.sqrt(4262714.748894526)
var_man2 = math.sqrt(1276617.8388812821)
var_man_vec =[var_man1, var_man2]

# Parameters obtained with the baseline experiment: time (for the linear axis, not the pick and place)
mean_t = 0.8504154353582923 
var_t = math.sqrt(0.126730866728932)

# Parameters to be initilaized
best_tradeoff = -999999999 # -infinity
type_obj = 0
current_pos = 0 # y = 0 => robot starts in the center of the line
base_position_sequence = [] 
items = []

# File name
overall_path = "all_data_file.txt"

def send_array(sock, array):
    # Send the shape and type of the array first
    shape = np.array(array.shape, dtype = np.int32)
    sock.sendall(shape.tobytes())
    sock.sendall(array.tobytes())

def main():

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    print("the connection has happened succesfully \n")

    # Start computing the time
    start_time = time.time()
    with open(overall_path, 'w') as f: f.write(f"inizio tempo: {start_time} \n \n")

    # Instantiate the class to start the 'Best Fit Algorithm'
    packers = []
    packer1 = Packer()
    packer2 = Packer()
    packers.append(packer1)
    packers.append(packer2)

    Bin_00 = Bin('Type1_box1', 300, 200, 130, 20)
    Bin_00.set_offset(-900, -530, -107) # FIXME: coordinates of the lower-left corner
    packer1.add_bin(Bin_00)

    Bin_10 = Bin('Type2_box1', 300, 200, 130, 20)
    Bin_10.set_offset(-400, -530, -107) # FIXME: coordinates of the lower-left corner
    packer2.add_bin(Bin_10)

    # append all the items to be accounted for
    packer1.add_item(Item('Cube_00', 75, 150, 80, 1))
    packer1.add_item(Item('Cube_01', 75, 150, 80, 1))
    packer1.add_item(Item('Cube_02', 75, 150, 80, 1))
    packer2.add_item(Item('Cube_10', 100, 70, 80, 1))
    packer2.add_item(Item('Cube_11', 100, 70, 80, 1))
    packer2.add_item(Item('Cube_12', 100, 70, 80, 1))

    # Instantiate the class for the motion planning
    motion_planner = MotionPlanner(v_base, a_base)

    # * STEP 1 => for all the types of objects you have ...
    while type_obj < num_types:

        # Get iteration specific parameters
        packer = packers[type_obj]
        num_bins = num_bins_array[type_obj]
        bin = 0
        num_obj_pick = num_objects_array[type_obj] # How many objects for a carteian type I have
        pick_objects = [] # array containing the index of objects (pick side) already picked and placed
        var_man = var_man_vec[type_obj] # ? still need to obtain these parameters
        mean_man = mean_man_vec[type_obj] # ? still need to obtain these parameters

        #! think of removing this part (take out one socket call => efficiency increases) 
        generic_bin = packer.bins[0] # First sorted bin (the one with biggest volume)
        generic_item = generic_bin.items[0] # First sorted item (the one with biggest volume)
        z_top_face = (generic_item.depth) / 2
        z_top_face_send = np.array ([[z_top_face]], dtype = np.int32)
        send_array(s, z_top_face_send) #! socket call 1

        with open (overall_path, 'a') as f: f.write(f"Type: {type_obj} \n")

        # * STEP 2 => for all the bins available for a specific type of object ...
        while bin < num_bins: 

            place_points = []
            rotations = []
            k = 0
            b = packer.bins[bin] 
            x_offset_box, y_offset_box, z_offset_box = b.get_offset() # coordinates of the lower-left corner of the bin

            #! Suggestion by Sole: you have to clear items (still do not know what she meant)
            for item in b.items:
                items.append(item) # items is a vector containing all the 'place-side-objects'
                k = k + 1
                place_points.append(item.get_center()) 
                rotations.append(item.rotation_type)

            num_objects = k # number of objects inside the bin ('spots' available for objects in a bin)
            num_objects_send = np.array ([[num_objects]], dtype = np.int32) 
            send_array(s, num_objects_send) #! socket call 2
           
            with open (overall_path, 'a') as f:
                f.write(f"Bin: {bin} \n")

            # * STEP 3 => for all the objects ('place-side') that can be packed in a specific bin ...
            i = 0 
            while i < num_objects:

                with open (overall_path, 'a') as f: f.write(f"place side object: {i} \n")

                # send the 'absolute' coordinates of the 'place position' associated to the item to be packed (assume all items identical)
                place_x = place_points[i][0] + x_offset_box
                place_y = place_points[i][1] + y_offset_box
                place_z = place_points[i][2] + z_offset_box
                rotation = rotations[i] # Also send the rotation
                place_point_send = np.array ([[place_x, place_y, place_z, rotation]], dtype = np.int32)
                send_array(s, place_point_send) #! socket call 3

                # Initialize variables for the PSO algorithm
                c = 0 # initilaize the index of the object to be picked (pick side) 
                best_tradeoff = -9999999999 #se cambio oggetto di place azzer il tradeoff #! What?
              
                # * STEP 4 =>  for all the items that can be picked (pick side) after knowing a specific 'place-side' spot ...
                while c < num_obj_pick: 

                    '''
                    Core of the optimization procedure
                    * The available 'pick-side' objects are scanned (c is the index), one is chosen and removed from the list
                    * The PSO algorithm is applied to find the best position for the base of the robot
                    * The best position is chosen according to the trade-off between manipulability and time
                    '''

                    if c not in pick_objects: 
   
                        skip = np.array([[0]], np.int32) # skip = 0 => Do not skip and test this object
                        send_array(s, skip) #! socket call 4
                        trigger_end = 1 # Iteration counter for PSO algorithm

                        #initialization of the pso particles 
                        particle_positions = np.random.uniform(lower_bound, upper_bound, num_particles)
                        particle_velocities = np.random.uniform(-1, 1, num_particles) 
                                              
                        with open(overall_path, 'a') as f: f.write(f"Object: {c} \n")
                        
                        '''
                        PSO algorithm 
                        '''
                        while trigger_end <= Nsim:

                            # * Update inertia and layout
                            inertia_weight = w0 + delta_w * trigger_end
                            layout = np.array([[int(particle_positions[i]) for i in range(num_particles)]], dtype=np.int32)
                            send_array(s, layout) #! socket call 5
                            
                            ''' 
                            TPS computes the fitness values for all particles (C# side)
                            '''

                            fitness = s.recv(1024).decode() #! receive 5
                            fitness = np.array([int(num) for num in fitness.split(',')])

                            with open (overall_path, 'a') as f: f.write(f"Iteration: {trigger_end}; Particle positions: {layout}; Fitness: {fitness} \n")

                            # * Set personal best and global best (the mask allows to remove a for loop)
                            if trigger_end == 1 :

                                personal_best_positions = particle_positions.copy()
                                global_best_position = personal_best_positions[np.argmax(personal_best_scores)]

                                personal_best_scores = fitness.copy()
                                global_best_score = np.max(personal_best_scores)

                            else :
                                # Vectorized update of personal bests
                                better_mask = fitness > personal_best_scores
                                personal_best_positions[better_mask] = particle_positions[better_mask]
                                personal_best_scores[better_mask] = fitness[better_mask]

                                # Update global best
                                max_index = np.argmax(fitness)
                                if fitness[max_index] > global_best_score:
                                    global_best_position = particle_positions[max_index]
                                    global_best_score = fitness[max_index]

                            # * Update particles
                            for i in range (num_particles):

                                # Random vectors for cognitive and social components
                                r1 = np.random.random(size = num_particles)
                                r2 = np.random.random(size = num_particles)

                                # Vectorized velocity update
                                inertia = inertia_weight * particle_velocities
                                cognitive = cognitive_component * r1[:, np.newaxis] * (personal_best_positions - particle_positions)
                                social = social_component * r2[:, np.newaxis] * (global_best_position - particle_positions)
                                particle_velocities = inertia + cognitive + social

                                # Update positions
                                particle_positions += particle_velocities

                                # Convert to int
                                particle_positions = particle_positions.astype(int)

                                # Clip positions within bounds
                                particle_positions = np.clip(particle_positions, lower_bound, upper_bound)
   
                        ''' 
                        Time-manipulability trade-off
                        '''
                        travel_time = motion_planner.trapezoidal_velocity_profile(current_pos, global_best_position)  
                        tradeoff = ((global_best_score - mean_man) / var_man) - (((travel_time - mean_t) / var_t)) #! Consider to change it  
                        
                        if (tradeoff > best_tradeoff):
                            best_tradeoff = tradeoff
                            next_position = global_best_position
                            next_item = c
   
                    else: # the 'c-th' object has already been picked and placed, so I skip it
                        skip = np.array([[1]], dtype = np.int32) # skip = 1 => Skip
                        send_array(s, skip)


                    c = c + 1 # go to the next object (pick side) to be tested

                # * All 'pick-side' items scanned: I know the object to be picked (sequence) and where to put the robot (base position)
                current_pos = next_position 
                pick_objects.append(next_item)
                base_position_sequence.append(current_pos)

                # * Update the counter of objects packed so far inside the bin-th box
                i = i + 1

            # * A box for a specific type of object is full: I go to the next one available
            bin = bin + 1      

        # * All items of a specific type are packed: I go to the next type of object
        type_obj = type_obj + 1

    # Close the connection
    s.close()
    end_time = time.time()
    elapsed_time = end_time - start_time
    with open(overall_path, 'a') as f: f.write(f"The total time taken by the algorithm is: {elapsed_time} seconds \n")

# Run the code
if __name__ == "__main__":

    # Run the code
    main()
            
