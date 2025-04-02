""" 
Test code to verify the data exchange with a socket-based architecture comprising nested loops.
"""

import socket
import numpy as np 
import sys 
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))) # 'test' becomes the root if you do ctrl + R here
from utils.parameters import VanillaSimulationParameters

verbose = True
shared_parameters = VanillaSimulationParameters()

def send_array(sock, array):
    # Send the shape and type of the array first
    shape = np.array(array.shape, dtype = np.int32)
    sock.sendall(shape.tobytes())
    sock.sendall(array.tobytes())

def main():

    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Define the host and port
    host = '127.0.0.1'
    port = 12345

    # Connect to the server
    s.connect((host, port))

    # Number of simulations
    Nsim = shared_parameters.Nsim
    trigger_end = shared_parameters.trigger_end
    dim_test_vec = shared_parameters.dim_test_vec
    nested_idx = shared_parameters.nested_idx
    loop_idx = shared_parameters.loop_idx

    # Define the vector to be sent in the inner-most loop
    test_vec = np.zeros((dim_test_vec, 1), dtype = np.int32)

    # Send these information to the server (C#)
    send_array(s, np.array([[Nsim, trigger_end, nested_idx, loop_idx]], dtype = np.int32))

    while trigger_end < Nsim - 1:

        # Create two numpy arrays
        sequence = np.array([[0, 0, trigger_end]], dtype = np.int32)
        tasks = np.array([[2, 3]], dtype = np.int32)
        starting_times = np.array([[trigger_end, trigger_end + 1, trigger_end + 2]], dtype = np.int32)
        if verbose: print(f"The arrays sent to C# at iteration {trigger_end} are:\n Sequence: {sequence};\n Tasks: {tasks};\n Starting times: {starting_times}")

        # Send arrays to the server (C#)
        send_array(s, sequence)
        send_array(s, tasks)
        send_array(s, starting_times)

        # Re-initialize the nested index
        nested_idx = 0

        while nested_idx <= loop_idx:

            # Craete a numpy array with all elements equal to nested_idx
            test_vec[:] = nested_idx
            send_array(s, test_vec)

            #! The C# code is processing the information ...
            nested_idx = int(s.recv(1024).decode())

            # Verify that C# is working correctly
            if verbose: print(f"Nested loop index: {nested_idx}")


        #! ... waiting for C# code to finish processing ...

        # Receive two arrays (all elements are equal)
        result1 = s.recv(4096).decode()
        result1 = np.array([int(num) for num in result1.split(',')])

        result2 = s.recv(4096).decode()
        result2 = np.array([int(num) for num in result2.split(',')])
        if verbose: print(f"The arrays received from C# at iteration {trigger_end} are:\n Result1: {result1};\n Result2: {result2}")

        # Ask the user to press a key to continue => easy way to check the synchronization
        input("Press Enter to continue...")

        # Update the counter
        trigger_end += 1

    # Close the connection
    s.close()

if __name__ == "__main__":

    # Run the code
    main()