from heapq import heapify, heappop, heappush
import matplotlib.pyplot as plt
import numpy as np
from environment import Environment
from occupancy_map import OccupancyMap

## lambda function because I'm lazy
heuristic = lambda a, b: np.linalg.norm(np.asarray(a) - np.asarray(b))

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

def graph_search(world, start, goal, resolution = 1.0, margin=5.0):
    '''
        Algorithm followed from http://robotics.caltech.edu/wiki/images/e/e0/Astar.pdf
    '''
    occ = OccupancyMap(world, resolution=resolution)

    ## individual asserts for easy debug
    assert(occ.is_valid_position(start))
    assert(occ.is_valid_position(goal))
    assert(not occ.is_occupied_position(start))
    assert(not occ.is_occupied_position(goal))
    
    heuristic = lambda a, b: np.linalg.norm(np.asarray(a) - np.asarray(b))
    
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
            return waypoints[::-1]  ## put in order from start to goal

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

    return False

if __name__ == "__main__":
    world = {'bounds': (0, 10, 0, 10, 0, 10),
            'blocks': [{'position':(1, 1, 1), 'size': (2, 2, 2)},
                       {'position':(5, 5, 0), 'size': (3, 3, 3)},
                       {'position':(7, 2, 4), 'size': (1, 4, 1)}]
            }

    env = Environment(world)

    start = (4, 2, 0)
    goal = (9, 8, 8)

    waypoints = graph_search(env, start, goal, margin=0.5, resolution=0.5)

    ax = env.get_plot()
    
    waypoints = [start] + waypoints + [goal]
    xs, ys, zs = zip(*waypoints)
    ax.plot(xs, ys, zs, color='b', marker='o')
    ax.plot([start[0]], [start[1]], [start[2]], color='green', marker='o')
    ax.text(start[0], start[1], start[2], "START", color='green')
    ax.plot([goal[0]], [goal[1]], [goal[2]], color='red', marker='o')
    ax.text(goal[0], goal[1], goal[2], "GOAL", color='red')
    plt.savefig('waypoints.png')
    plt.show()