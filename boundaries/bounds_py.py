''' 
Evaluate boundaries on the robotized linear packing problem.
this code also represents a more structured attempt to test the socket communication:
    the complete procedure to find the best boundaries could have been enforced in C# only.
'''

import socket
import numpy as np
import random
import os
import sys

# Set the path to the 3D bin packing library
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item, Scene

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

def main():

    # Import the simulation parameters
    params = BoundarySimulationParameters()

    # Create a socket connection to the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((params.host, params.port))

    # Initialize the text file for the results
    with open(save_path, 'w') as f: f.write("Upper and lower bounds for the problem\n")

    # ? Initialize the parameters for the simulation
    i = 0
    items = []
    place_points = []    
    keep = []
    discard = []

    # Create the bin and item objects
    packer1 = Packer()
    b = Bin('Type1_box1', 300, 200, 130, 20)
    b.set_offset(-900, -530, -107.14)
    packer1.add_bin(b)
    packer1.add_item(Item('Cube_02', 75, 150, 80, 1))

    # Call the packing method and get the coordinates of where to put the item
    packer1.pack()
    x_offset_box, y_offset_box, z_offset_box = b.get_offset()
    if params.verbose: print(f"offset: {x_offset_box}, {y_offset_box}, {z_offset_box}")

    # 'place_points' contains half the lengths specified for Cube_00 (and only one 'place-side' obejct)
    for item in b.items:
        items.append(item)
        place_points.append(item.get_center()) # ! quite a useless function 
        if params.verbose: print(f"{place_points} \n")
    
    # loop over all the needed simulations
    while(i < params.Nsim):

        # Generate data to test
        base_position = random.uniform(params.lower_bound, params.upper_bound)
        rotation = 0
        place_x = place_points[0][0] + x_offset_box
        place_y = place_points[0][1] + y_offset_box
        place_z = place_points[0][2] + z_offset_box + params.z_offset
        if params.verbose: print(f"Place position: {place_x}, {place_y}, {place_z}")
        layout = np.array ([[base_position, place_x, place_y, place_z, rotation]], dtype = np.int32)
        send_array(s, layout)

        # ! C# is evaluating the fitness function

        # Get the manipulability measure
        result = s.recv(1024).decode()
        result = np.array([int(num) for num in result.split(',')]) # fitness, flag
        flag = result[0]
        mean_determinant = result[1] / 100000
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
    with open(save_path, 'a') as f: f.write(f"\n ************ ")
    with open(save_path, 'a') as f: f.write(f"Supposed feasible range: from {min_feasible} mm to {max_feasible} mm \n")


if __name__ == "__main__":

    # Run the code
    main()
            






        

        











