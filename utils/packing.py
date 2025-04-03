'''
This module defines the PackingListCreator class, responsible for setting up the list of packers.
'''

import os
import sys
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', '3dbinpacking'))
sys.path.append(project_root)
from py3dbp import Packer, Bin, Item, Scene

class PackingListCreator:
    def __init__(self, params):
        self.params = params
        self.packers = []

    def create(self):
        self.packers = []  # reset in case of reuse

        # * Loop over the items of each type ...
        for i in range(len(self.params.items_of_each_type)):
            packer = Packer()
            N_current_bins = self.params.bins_of_each_type[i]
            N_current_items = self.params.items_of_each_type[i]

            # * Create the instances of the bins for the current type of items
            for j in range(N_current_bins):
                bin_name = f"{self.params.bins_root_name}_{i}{j}"
                bin_size = self.params.bins_sizes_and_weight[i][:-1]
                bin_weight = self.params.bins_sizes_and_weight[i][-1]
                current_bin = Bin(bin_name, *bin_size, bin_weight)

                bin_center = self.params.bins_centers[i]
                current_bin.set_offset(
                    bin_center[0] - bin_size[0] / 2,
                    bin_center[1] - bin_size[1] / 2,
                    bin_center[2]
                )
                packer.add_bin(current_bin)

            # * For the type you are considering, loop over all the items of that type
            for k in range(N_current_items):
                item_name = f"{self.params.items_root_name}_{i}{k}"
                item_size = self.params.items_sizes_and_weight[i][:-1]
                item_weight = self.params.items_sizes_and_weight[i][-1]
                item = Item(item_name, *item_size, item_weight)
                packer.add_item(item)

            self.packers.append(packer)

        for packer in self.packers:
            packer.pack(distribute_items=True)

    def list_bin_names(self):
        bin_list = []
        for packer in self.packers:
            for bin in packer.bins:
                bin_list.append(bin.name)
        return bin_list
    
    def list_unfitted_items(self):
        unfitted = {}
        for packer in self.packers:
            for bin in packer.bins:
                if bin.unfitted_items:
                    unfitted[bin.name] = [item.name for item in bin.unfitted_items]
        return unfitted

    
    def list_items_names(self):
        item_list = []
        for packer in self.packers:
            for bin in packer.bins:
                for item in bin.items:
                    item_list.append(item.name)
        return item_list

    
    def list_place_points(self):
        place_points = []
        for packer in self.packers:
            for bin in packer.bins:
                for item in bin.items:
                    place_points.append(item.get_center())
        return place_points

    
    def list_rotations(self):
        rotations = []
        for packer in self.packers:
            for bin in packer.bins:
                for item in bin.items:
                    rotations.append(item.rotation_type)
        return rotations

    

from parameters import PackingSimulationParameters

# Run test scenario
if __name__ == '__main__':
    params = PackingSimulationParameters()
    creator = PackingListCreator(params)

    creator.create()

    bin_names_list = creator.list_bin_names()
    items_names_list = creator.list_items_names()
    place_points_list = creator.list_place_points()
    rotations_list = creator.list_rotations()
    unfitted_items_list = creator.list_unfitted_items()

    print(f"The bins are: {bin_names_list}")
    print(f"The items are: {items_names_list}")
    print(f"The place points are: {place_points_list}")
    print(f"The rotations are: {rotations_list}")
    print(f"The unfitted items are: {unfitted_items_list}")
