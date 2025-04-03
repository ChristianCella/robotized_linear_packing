''' 
What this code does
'''

import socket
import numpy as np
import random
import os
import sys

# Choose the type of objects you want to use for the test
selected_type = 1
if selected_type == 0:
    file_name = "manipulability_type0.txt"
elif selected_type == 1:
    file_name = "manipulability_type1.txt"

# Set the path to the 3D bin packing library
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item

# Get the full path to the text file
script_dir = os.path.dirname(os.path.realpath(__file__)) # Get absolute path of the current script
project_root = os.path.abspath(os.path.join(script_dir, "..")) # Go up one folder to "robotized_linear_packing"
save_path = os.path.join(project_root, "data", file_name) # Build full path to bounds_experiment.txt

# Append the path for utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.parameters import ManipulabilitySimulationParameters
from utils.packing import PackingListCreator

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
    params = ManipulabilitySimulationParameters()
    creator = PackingListCreator(params)
    creator.create()

    # Create a socket connection to the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((params.host, params.port))

    # Initialize the text file for the results
    with open(save_path, 'w') as f: f.write("Upper and lower bounds for the problem\n")

    # Initialize the parameters for the simulation
    i = 0
    place_points = []    
    keep = []
    item_names = []
    place_points = []
    rotations = []

    # Simplified case: I only have one bin and 3 items
    x_offset_box, y_offset_box, z_offset_box = creator.packers[selected_type].bins[0].get_offset() 
    for idx1 in range(len(creator.packers[selected_type].bins[0].items)):
        place_points.append(creator.packers[selected_type].bins[0].items[idx1].get_center())
        rotations.append(creator.packers[selected_type].bins[0].items[idx1].rotation_type) 
    for idx2 in range(len(creator.packers[selected_type].bins[0].items)):
        item_names.append(creator.packers[selected_type].bins[0].items[idx2].name) 
    box_name = creator.packers[selected_type].bins[0].name # You only have 1 box

    if params.verbose:
        print(f"Item name: {item_names}")
        print(f"Box name: {box_name}")
        print(f"Place points: {place_points}")
        print(f"Rotation: {rotations}")

    # Send the shared numbers
    send_array(s, np.array([[params.Nsim, params.pre_post_height, params.n_decimals]], dtype = np.int32))

    # Send the shared strings
    send_strings(s, [box_name, params.robot_program_name])

    # loop over all the needed simulations
    while(i < params.Nsim):

        # Randomize the object to be picked
        rand_pick = random.choice([0, 1, 2])

        # Randomize the place position
        rand_place = random.choice([0, 1, 2])
        place_x = place_points[rand_place][0] + x_offset_box
        place_y = place_points[rand_place][1] + y_offset_box
        place_z = place_points[rand_place][2] + z_offset_box + params.items_sizes_and_weight[0][2]/2
        rotation = rotations[rand_place]

        # Generate the new position for the robot base
        base_position = random.uniform(params.lower_bound, params.upper_bound)

        # Send the integer data to the C# server
        layout = np.array ([[base_position, place_x, place_y, place_z, rotation]], dtype = np.int32)
        send_array(s, layout)

        # Send the string that corresponds to the item to be picked
        send_strings(s, [item_names[rand_pick]])

        # ! ... C# is evaluating the fitness function ...

        # Get the manipulability measure
        result = s.recv(1024).decode()
        result = np.array([int(num) for num in result.split(',')]) # fitness, flag
        flag = result[0]
        mean_determinant = result[1] / (10 ** 5)
        print(f"Iteration number: {i}")

        # Add the value to the text file
        if flag == 0:
            keep.append(mean_determinant)
            with open(save_path, 'a') as f: f.write(f"Iteration: {i};  Base position: {base_position} mm; Feasibility: YES!; Manipulability: {mean_determinant} \n")

        else:
            with open(save_path, 'a') as f: f.write(f"Iteration {i}; Item picked: {item_names[rand_pick]}; Base position: {base_position} mm; Feasibility: NO! \n")

        # Increase the counter
        i += 1

    # Close the socket communication
    s.close()

    # Calculate the mean and standard deviation of the manipulability
    if keep == []:
        if params.verbose: print("No feasible solutions found.")
        with open(save_path, 'a') as f: f.write(f"No feasible solutions found. \n")
        return
    mean = np.mean(keep)
    std = np.std(keep)
    with open(save_path, 'a') as f: f.write(f"\n")
    with open(save_path, 'a') as f: f.write(f"********* Mean manipulability: {mean}; Standard deviation: {std} *********")


if __name__ == "__main__":

    # Run the code
    main()
            






        

        











