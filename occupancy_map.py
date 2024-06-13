import numpy as np
from environment import Environment

class OccupancyMap:
    def __init__(self, world:Environment, resolution:float):
        self.world = world
        self.resolution = resolution
        self.grid = self._to_occupancy_grid()
    
    
    def _to_occupancy_grid(self):
        x_min, x_max, y_min, y_max, z_min, z_max = self.world.map_bounds
        x_dim = int(np.ceil((x_max - x_min) / self.resolution))
        y_dim = int(np.ceil((y_max - y_min) / self.resolution))
        z_dim = int(np.ceil((z_max - z_min) / self.resolution))

        occupancy_map = np.zeros((x_dim, y_dim, z_dim))
        for obj in self.world.objects:
            x, y, z = obj['position']
            dx, dy, dz = obj['size']
            x_start = int((x - x_min) / self.resolution)
            x_end = int((x + dx - x_min) / self.resolution)
            y_start = int((y - y_min) / self.resolution)
            y_end = int((y + dy - y_min) / self.resolution)
            z_start = int((z - z_min) / self.resolution)
            z_end = int((z + dz - z_min) / self.resolution)
            occupancy_map[x_start:x_end, y_start:y_end, z_start:z_end] = 1

        return occupancy_map
    
    def position_to_index(self, position):
        x, y, z = position
        x_min, _, y_min, _, z_min, _ = self.world.map_bounds
        i = int(np.floor((x - x_min)/self.resolution))
        j = int(np.floor((y - y_min)/self.resolution))
        k = int(np.floor((z - z_min)/self.resolution))

        return (i, j, k)
    

    def is_valid_index(self, index):
        for i in range(3):
            if (index[i] > self.grid.shape[i]) or i < 0:
                return False
        return True
    
    def is_valid_position(self, position):
        bounds = self.world.map_bounds
        for i in range(3):
            if i == 2:
                if (position[i] < bounds[i*2]) or (position[i] >= bounds[i*2+1]):  ## allow z-axis to be on the ground but not on the ceil of map
                    return False
            elif (position[i] <= bounds[i*2]) or (position[i] >= bounds[i*2+1]):  ## do not allow to be on map boundaries
                return False
        return True

    def is_occupied_index(self, index):
        if self.grid[tuple(index)] == 1:
            return True
        return False
    
    def is_occupied_position(self, position):
        idx = self.position_to_index(position)
        return self.is_occupied_index(idx)