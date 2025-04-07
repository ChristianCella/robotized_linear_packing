''' 
This module contains the parameters for the simulation.
'''

from dataclasses import dataclass, field
from typing import List

# data for the 'vanilla' socket example contained in the directory 'test'
@dataclass
class VanillaSimulationParameters:
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 5
    trigger_end: int = 0
    dim_test_vec: int = 7
    nested_idx: int = 0
    loop_idx: int = 5

# data for the the problem of determining the boundaries for the robot
@dataclass
class BoundarySimulationParameters:
    verbose : bool = True
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 1000
    lower_bound: int = -1500 # [mm]
    upper_bound: int = 1500 # [mm]
    items_root_name: str = 'Cube'
    bins_root_name: str = 'Bin'
    robot_program_name: str = 'RobotProgram'
    pre_post_height: int = 200 # [mm]
    n_decimals: int = 5 
    items_of_each_type: List[int] = field(default_factory=lambda: [1])
    items_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[75, 150, 80, 1]]) # '1' is a random weight
    bins_of_each_type: List[int] = field(default_factory=lambda: [1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[300, 200, 130, 20]]) # '20' is a random weight 
    bins_centers: List[List[float]] = field(default_factory=lambda: [[-750, -430, -107.14]])

# data for determining the manipulability's baseline
@dataclass
class ManipulabilitySimulationParameters:
    verbose : bool = False
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 1000
    lower_bound: int = -1500 # [mm]
    upper_bound: int = 1500 # [mm]
    items_root_name: str = 'Cube'
    bins_root_name: str = 'Bin'
    robot_program_name: str = 'RobotProgram'
    pre_post_height: int = 200 # [mm]
    n_decimals: int = 5 
    items_of_each_type: List[int] = field(default_factory=lambda: [3, 3])
    items_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[75, 150, 80, 1], [100, 70, 80, 1]]) # '1' is a random weight
    bins_of_each_type: List[int] = field(default_factory=lambda: [1, 1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[300, 200, 130, 20], [300, 200, 130, 20]]) # '20' is a random weight
    bins_centers: List[List[float]] = field(default_factory=lambda: [[-750, -430, -107.14], [-250, -430, -107.14]])

# data for determining the manipulability's baseline
@dataclass
class TravelSimulationParameters:
    verbose : bool = False
    N_iter: int = 1000
    lower_bound: int = -1500 # [mm]
    upper_bound: int = 1500 # [mm]
    v_max : int = 700 # [mm / s]
    a_max : int = 900 # [mm / s^2]

# data for the the test of the function inside 'packing.py'
@dataclass
class PackingSimulationParameters:
    verbose : bool = True
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 20
    lower_bound: int = -1500 # [mm]
    upper_bound: int = 1500 # [mm]
    items_root_name: str = 'Cube'
    bins_root_name: str = 'Bin'
    robot_program_name: str = 'RobotProgram'
    pre_post_height: int = 200 # [mm]
    n_decimals: int = 5 
    items_of_each_type: List[int] = field(default_factory=lambda: [6, 3])
    items_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[75, 150, 80, 1], [100, 70, 80, 1]]) # '1' is a random weight
    bins_of_each_type: List[int] = field(default_factory=lambda: [2, 1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[300, 200, 130, 20], [300, 200, 130, 20]]) # '20' is a random weight
    bins_centers: List[List[float]] = field(default_factory=lambda: [[-750, -430, -107.14], [-1250, -200, -55], [-250, -430, -107.14]]) # Specify all the centers!



# Data for the real optimization procedure (to be implemented)
@dataclass
class RealSimulationParameters:
    verbose : bool = True
    timeout: int = 60 # [s] timeout for the socket connection
    host: str = '127.0.0.23' # socket address
    port : int = 123 # socket port
    v_max: int = 700 # [mm / s] => for the line
    a_max: int = 900 # [mm / s^2] => for the line
    base_upper_bound: int = 1500 # [mm] for the robot base (NOT taken from /data/bounds_experiment.txt)
    base_lower_bound: int = -1500 # [mm] for the robot base (NOT taken from /data/bounds_experiment.txt)
    vel_upper_bound: int = 1 # Upper bound for particle velocity
    vel_lower_bound: int = -1 # Lower bound for particle velocity
    N_sim_pso: int = 3 # N simulations for the pso
    N_particles: int = 2 # N particles for the pso
    w0: float = 0.9 # upper bound on inertia
    wN: float = 0.4 # lower bound on inertia
    delta_w: float = (w0 - wN) / N_sim_pso # decay of the inertia
    c1: int = 2 # cognitive component
    c2: int = 2 # social component
    max_cost: int = 9000000 # max cost for the pso
    mean_travel_time: float = 2.183 # mean travel time (taken from /data/trvel_time.txt)
    std_travel_time: float = 1.0714 # std travel time (taken from /data/trvel_time.txt)
    mean_manip: List[float] = field(default_factory = lambda: [0.1766, 0.1366]) # mean manipulability of the objects (taken from /data/manip_and_time.txt)
    std_manip: List[float] = field(default_factory = lambda: [0.0244, 0.0273]) # std manipulability of the objects (taken from /data/manip_and_time.txt)
    mean_time_pp: List[float] = field(default_factory = lambda: [10.337, 9.747]) # mean time for the pick and place (taken from /data/manip_and_time.txt)
    std_time_pp: List[float] = field(default_factory = lambda: [0.201, 0.536]) # std time for the pick and place (taken from /data/manip_and_time.txt)
    items_root_name: str = 'Cube' # shared name between the objects
    bins_root_name: str = 'Bin' # shared name between the objects
    robot_program_name: str = 'RobotProgram'
    robot_name: str = 'GoFa12'
    tool_name: str = 'Suction_cup'
    tcp_ee_name: str = 'tgripper_tf'
    tcp_flange_name: str = 'TOOLFRAME'
    file_name: str = 'use_case.txt' # file name for the results
    pre_post_height: int = 200 # [mm] height of the pre and post pick and place
    n_decimals: int = 3 # number of decimals for the results
    alpha_fitness: float = 0.5 # weight for the fitness function (fitness of each particle)   
    beta_fitness: float = 0.5 # weight for the fitness function (fitness of each particle)
    alpha_tradeoff: float = 0.5 # weight for the trade-off function (cost associated to moving the robot base)
    beta_tradeoff: float = 0.5 # weight for the trade-off function (cost associated to moving the robot base)
    items_of_each_type: List[int] = field(default_factory = lambda: [3, 3])
    items_sizes_and_weight: List[List[float]] = field(default_factory = lambda: [[75, 150, 80, 1], [100, 70, 80, 1]])
    bins_of_each_type: List[int] = field(default_factory = lambda: [1, 1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory = lambda: [[300, 200, 130, 20], [300, 200, 130, 20]])
    bins_centers: List[List[float]] = field(default_factory = lambda: [[-750, -430, -107.14], [-250, -430, -107.14]])
    

# Test (think of using this also in the final version)
if __name__ == "__main__":

    parameters = RealSimulationParameters()
    print("Items of each type: ", parameters.items_of_each_type)

