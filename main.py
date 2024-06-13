from heapq import heapify, heappop, heappush
import numpy as np
from environment import Environment
from occupancy_map import OccupancyMap

## lambda functions because I'm lazy -- and yes, they are the same
## Used 2 for clarification on what my cost functions are
## TODO: Make cost functions configurable?
h = lambda x, goal: np.linalg.norm(np.asarray(x) - np.asarray(goal))
g = lambda x, start: np.linalg.norm(np.asarray(x) - np.asarray(start))

def get_neighbors(pos, resolution):
    x , y, z = pos
    neighbors = []

    diff = [-resolution, 0, resolution]
    for i in diff:
        for j in diff:
            for k in diff:
                neighbor_pos = (x + i, y + j, z + k)
                neighbors.append(neighbor_pos)

    return neighbors

# def get_path(parent_dict, start, goal):
#     path = []

def graph_search(world, start, goal, resolution = 1.0, margin=5.0):
    '''
        Algorithm followed from http://robotics.caltech.edu/wiki/images/e/e0/Astar.pdf
    '''
    x_min, x_max, y_min, y_max, z_min, z_max = world.map_bounds
    occ = OccupancyMap(world, resolution=resolution)
    # assert(occ.is_valid_position(start) and occ.is_valid_position(goal))
    assert(occ.is_valid_position(start))
    assert(occ.is_valid_position(goal))
    # assert(not occ.is_occupied_position(start) and not occ.is_occupied_position(goal))
    assert(not occ.is_occupied_position(start))
    assert(not occ.is_occupied_position(goal))
    
    heuristic = lambda a, b: np.linalg.norm(np.asarray(a) - np.asarray(b))

    graph = [(i, j, k) for i in range(x_min, x_max+1) for j in range(y_min, y_max+1) for k in range(z_min, z_max+1)]
    
    parent = {}
    g_scores = {start: 0}
    f_scores = {start: heuristic(start, goal)}

    open = [(f_scores[start], start)]
    heapify(open)
    closed = []

    while open:  ## while open list is not empty
        curr = heappop(open)[1]
        closed.append(curr)
        
        if (curr == goal) or (np.linalg.norm(np.array(curr) - np.array(goal)) <= margin):
            waypoints = [curr]
            while curr in parent:
                waypoints.append(curr)
                curr = parent[curr]
            return waypoints[::-1]  ## put in correct order

        neighbors = get_neighbors(curr, resolution)
        for node in neighbors:
            if (not occ.is_valid_position(node)) or (occ.is_occupied_position(node)):
                continue

            g_score = g_scores[curr] + resolution
            if node not in g_scores or (g_score < g_scores.get(node, 0)):
                parent[node] = curr
                g_scores[node] = g_score
                f_scores[node] = g_score + heuristic(node, goal)
                heappush(open, (f_scores[node], node))
            # print(f"Pushed node: {node}\n parent[{node}]={parent[node]}\n f[{node}]={f_scores[node]}")

    return False

if __name__ == "__main__":
    world = {'bounds': (0, 10, 0, 10, 0, 10),
            'blocks': [{'position':(1, 1, 1), 'size': (2, 2, 2)},
                       {'position':(5, 5, 0), 'size': (3, 3, 3)},
                       {'position':(7, 2, 4), 'size': (1, 4, 1)}]
            }

    env = Environment(world)

    start = (4, 2, 0)
    goal = (8, 2, 0)

    waypoints = graph_search(env, start, goal, margin=1.0)
    print(waypoints)