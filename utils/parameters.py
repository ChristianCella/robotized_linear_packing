from dataclasses import dataclass, field
from typing import List

import os
import sys
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item, Scene

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
    verbose : bool = False
    host: str = '127.0.0.1'
    port: int = 12345
    Nsim: int = 1000
    lower_bound: int = -1500 # [mm]
    upper_bound: int = 1500 # [mm]
    z_offset: int = 40 # [mm]
    

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

    items_of_each_type = parameters.items_of_each_type
    items_sizes_and_weight = parameters.items_sizes_and_weight
    items_root_name = parameters.items_root_name
    bins_root_name = parameters.bins_root_name
    bins_of_each_type = parameters.bins_of_each_type
    bins_sizes_and_weight = parameters.bins_sizes_and_weight
    bins_centers = parameters.bins_centers
    packers = []

    # * Loop over the items of each type
    for i in range(len(items_of_each_type)):
        packers.append(Packer())
        N_current_bins = bins_of_each_type[i]
        N_current_items = items_of_each_type[i]

        # * Create the bin for the current type of items
        for j in range(N_current_bins):
            bin_name = f"{parameters.bins_root_name}{i}{j}"
            bin_size = bins_sizes_and_weight[i][:-1]
            bin_weight = bins_sizes_and_weight[i][-1]
            current_bin = Bin(bin_name, *bin_size, bin_weight)
            bin_center = bins_centers[i]
            current_bin.set_offset(bin_center[0] - bin_size[0]/2, bin_center[1] - bin_size[1]/2, bin_center[2])
            packers[i].add_bin(Bin(bin_name, *bin_size, bin_weight))

        # * For the type you are considering, loop over all the items of that type
        for k in range(N_current_items):
            item_name = f"{items_root_name}{i}{k}"
            item_size = items_sizes_and_weight[i][:-1]
            item_weight = items_sizes_and_weight[i][-1]
            packers[i].add_item(Item(item_name, *item_size, item_weight))

    # Display the packers
    for i, packer in enumerate(packers):
        print(f"Packers {i}:")
        print(f"  Bins: {[bin.name for bin in packer.bins]}")
        print(f"  Items: {[item.name for item in packer.items]}")
        print(f"  Bin centers: {bins_centers[i]}")
        print()
