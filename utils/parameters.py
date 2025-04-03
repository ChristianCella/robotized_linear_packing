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
    items_of_each_type: List[int] = field(default_factory=lambda: [3])
    items_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[75, 150, 80, 1]]) # '1' is a random weight
    bins_of_each_type: List[int] = field(default_factory=lambda: [1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[300, 200, 130, 20]]) # '20' is a random weight
    bins_centers: List[List[float]] = field(default_factory=lambda: [[-750, -430, -107.14]])

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
    items_root_name: str = 'Item'
    bins_root_name: str = 'Bin'
    items_of_each_type: List[int] = field(default_factory=lambda: [3, 3])
    items_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[75, 150, 80, 1], [10, 70, 80, 1]])
    bins_of_each_type: List[int] = field(default_factory=lambda: [1, 1])
    bins_sizes_and_weight: List[List[float]] = field(default_factory=lambda: [[300, 200, 130, 20], [300, 200, 130, 20]])
    bins_centers: List[List[float]] = field(default_factory=lambda: [[-750, -430, -107.14], [-250, -430, -107.14]])
    

# Test (think of using this also in the final version)
if __name__ == "__main__":

    parameters = RealSimulationParameters()
    print("Items of each type: ", parameters.items_of_each_type)

