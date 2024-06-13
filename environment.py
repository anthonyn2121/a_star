import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Environment:
    def __init__(self, world_data):
        """
        Initializes the environment with given bounds.

        Parameters:
        
        world_data (dict): dict containing keys 'bounds' and 'blocks'
            bounds (tuple): (x_min, x_max, y_min, y_max, z_min, z_max)
            blocks (dict): dict containing keys 'position' and 'size'
                position (tuple): (x, y, z) position center of cuboid 
                size (tuple): (dx, dy, dz) size of object
        """
        assert ('bounds' in world_data)
        self.map_bounds = world_data['bounds']
        if 'blocks' in world_data.keys():
            self.objects = [block for block in world_data['blocks']]
        else:
            self.objects = []

    def add_object(self, position, size):
        """
        Adds an object to the environment.

        Parameters:
        obj_type (str): Type of the object ('cube')
        position (tuple): (x, y, z) position of the object
        size (tuple): (dx, dy, dz) size of the object
        """
        obj = {
            'position': position,
            'size': size
        }
        self.objects.append(obj)

    def plot_environment(self):
        """Plots the 3D map with all the objects."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Set map bounds
        x_min, x_max, y_min, y_max, z_min, z_max = self.map_bounds
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_zlim(z_min, z_max)

        # Plot objects
        for obj in self.objects:
            x, y, z = obj['position']
            dx, dy, dz = obj['size']
            self._plot_cube(ax, x, y, z, dx, dy, dz)

        plt.show()

    def _plot_cube(self, ax, x, y, z, dx, dy, dz):
            """
            Plots a cube at the specified position with the given size.

            Parameters:
            ax (Axes3D): The 3D axis to plot on.
            x, y, z (float): The position of the cube.
            dx, dy, dz (float): The size of the cube.
            """
            # Generate a random translucent color
            color = np.random.rand(4)
            color[3] = 0.5  # Set alpha to 0.5 for translucency

            # Define the vertices of the cube
            vertices = [
                [x, y, z],
                [x+dx, y, z],
                [x+dx, y+dy, z],
                [x, y+dy, z],
                [x, y, z+dz],
                [x+dx, y, z+dz],
                [x+dx, y+dy, z+dz],
                [x, y+dy, z+dz]
            ]

            # Define the 6 faces of the cube
            faces = [
                [vertices[j] for j in [0, 1, 5, 4]],
                [vertices[j] for j in [7, 6, 2, 3]],
                [vertices[j] for j in [0, 3, 7, 4]],
                [vertices[j] for j in [1, 2, 6, 5]],
                [vertices[j] for j in [0, 1, 2, 3]],
                [vertices[j] for j in [4, 5, 6, 7]]
            ]

            # Create a Poly3DCollection object for the cube
            poly3d = Poly3DCollection(faces, facecolors=color, linewidths=1, edgecolors='r')
            ax.add_collection3d(poly3d)


if __name__ == "__main__":
    # Example usage
    world = {'bounds': (0, 10, 0, 10, 0, 10),
             'blocks': [{'position':(1, 1, 1), 'size': (2, 2, 2)},
                        {'position':(5, 5, 0), 'size': (3, 3, 3)},
                        {'position':(7, 2, 4), 'size': (1, 4, 1)}]
            }

    env = Environment(world)

    env.plot_environment()
