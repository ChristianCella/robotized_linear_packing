''' 
Evaluate boundaries on the robotized linear packing problem.
this code also represents a more structured attempt to test the socket communication:
    * the complete procedure to find the best boundaries could have been enforced in C# only.
    * to avoid testing all the 6 items and the 2 bins, we consider the worst situiation:
        bin more on the left, item more on the right and closer to the robot ('bounds.drawio' in 'images').
'''

import socket
import numpy as np
import random
import os
import sys

# Set the path to the 3D bin packing library
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item

# Get the full path to the text file
script_dir = os.path.dirname(os.path.realpath(__file__)) # Get absolute path of the current script
project_root = os.path.abspath(os.path.join(script_dir, "..")) # Go up one folder to "robotized_linear_packing"
save_path = os.path.join(project_root, "data", "bounds_experiment.txt") # Build full path to bounds_experiment.txt

# Append the path for utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.parameters import BoundarySimulationParameters

def send_array(sock, array):
    # Send the shape and type of the array first
    shape = np.array(array.shape, dtype=np.int32)
    sock.sendall(shape.tobytes())
    sock.sendall(array.tobytes())

def send_strings(sock, strings):
    count = np.array([len(strings)], dtype=np.int32)
    sock.sendall(count.tobytes())

    for text in strings:
        encoded = text.encode('utf-8')
        length = np.array([len(encoded)], dtype=np.int32)
        sock.sendall(length.tobytes())
        sock.sendall(encoded)

def main():

    # Import the simulation parameters
    params = BoundarySimulationParameters()

    # Create a socket connection to the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((params.host, params.port))

    # Initialize the text file for the results
    with open(save_path, 'w') as f: f.write("Upper and lower bounds for the problem\n")

    # Initialize the parameters for the simulation
    i = 0
    place_points = []    
    keep = []
    discard = []

    # ! Dynamic way of creating the packing scene
    packers = []

    # * Loop over the items of each type ...
    for i in range(len(params.items_of_each_type)):
        packers.append(Packer())
        N_current_bins = params.bins_of_each_type[i]
        N_current_items = params.items_of_each_type[i]
        
        # * Create the instances of the bins for the current type of items
        for j in range(N_current_bins):
            bin_name = f"{params.bins_root_name}{i}{j}"
            bin_size = params.bins_sizes_and_weight[i][:-1]
            bin_weight = params.bins_sizes_and_weight[i][-1]
            current_bin = Bin(bin_name, *bin_size, bin_weight)
            bin_center = params.bins_centers[i]
            current_bin.set_offset(bin_center[0] - bin_size[0]/2, bin_center[1] - bin_size[1]/2, bin_center[2])
            packers[i].add_bin(current_bin)

        # * For the type you are considering, loop over all the items of that type
        for k in range(N_current_items):
            item_name = f"{params.items_root_name}_{i}{k}"
            item_size = params.items_sizes_and_weight[i][:-1]
            item_weight = params.items_sizes_and_weight[i][-1]
            packers[i].add_item(Item(item_name, *item_size, item_weight))

    # Simplified case: I only have one bin and one item
    x_offset_box, y_offset_box, z_offset_box = packers[0].bins[0].get_offset() # => [-900, -530, -107.14]
    place_points = packers[0].items[0].get_center() # => [37, 75, 40]
    rotation = packers[0].items[0].rotation_type # => 0
    #item_name = packers[0].items[0].name # => 'Item_00'
    item_name = 'Cube_02' # I am forcing it (look at bounds.drawio inside images)
    box_name = packers[0].bins[0].name # => 'Bin_00'

    # Send the shared numbers
    send_array(s, np.array([[params.Nsim, params.pre_post_height, params.n_decimals]], dtype = np.int32))

    # Send the shared strings
    send_strings(s, [item_name, box_name, params.robot_program_name])

    # loop over all the needed simulations
    while(i < params.Nsim):

        # Generate data to test
        base_position = random.uniform(params.lower_bound, params.upper_bound)
        place_x = place_points[0] + x_offset_box
        place_y = place_points[1] + y_offset_box
        place_z = place_points[2] + z_offset_box + params.items_sizes_and_weight[0][2]/2
        if params.verbose: print(f"Place pose: {place_x}, {place_y}, {place_z}")
        layout = np.array ([[base_position, place_x, place_y, place_z, rotation]], dtype = np.int32)
        send_array(s, layout)

        # ! ... C# is evaluating the fitness function ...

        # Get the manipulability measure
        result = s.recv(1024).decode()
        result = np.array([int(num) for num in result.split(',')]) # fitness, flag
        flag = result[0]
        mean_determinant = result[1] / (10 ** 5)
        if params.verbose: print(f"the success variables is {flag}, while the fitness is {mean_determinant}")

        # Add the value to the text file
        if flag == 0:
            keep.append(base_position)
            with open(save_path, 'a') as f: f.write(f"Iteration: {i}; Base position: {base_position} mm; Feasibility: YES!; Manipulability: {mean_determinant} \n")

        else:
            discard.append(base_position)
            with open(save_path, 'a') as f: f.write(f"Iteration {i}; Base position: {base_position} mm; Feasibility: NO! \n")

        # Increase the counter
        i += 1

    # Close the socket communication
    s.close()

    # Get the maximum and minimum values from the keep list
    if keep == []:
        if params.verbose: print("No feasible solutions found.")
        with open(save_path, 'a') as f: f.write(f"No feasible solutions found. \n")
        return
    min_feasible = min(keep)
    max_feasible = max(keep)
    with open(save_path, 'a') as f: f.write(f"\n")
    with open(save_path, 'a') as f: f.write(f"********* Supposed feasible range: from {min_feasible} mm to {max_feasible} mm *********")


if __name__ == "__main__":

    # Run the code
    main()
            






        

        











