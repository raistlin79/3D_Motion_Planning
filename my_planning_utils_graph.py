from enum import Enum
from queue import PriorityQueue
import numpy as np
from bresenham import bresenham
import numpy.linalg as LA

# For Voroni
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt

# For graph representation
import sys
import pkg_resources
import networkx as nx

# Should be 2.1
nx.__version__

# For Medial Axis
from skimage.morphology import medial_axis
from skimage.util import invert


# from Voronoi Solution
def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, int(north_min), int(east_min), edges

def show_start_goal(grid, edges):
    plt.imshow(grid, origin='lower', cmap='Greys')

    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')


        plt.plot(start_ne[1], start_ne[0], 'rx')
        plt.plot(goal_ne[1], goal_ne[0], 'rx')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()




def a_star(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost



def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))


def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = (int(p[0]) , int(p[1]))
            dist = d
    return closest_point


#------------------------------------
#------------------------------------

# Prune path collinearty check.

def prune_path(path):
    if len(path) < 2:
        return path
    else:
        p = 2
        while p < len(path) - 2:
            if collinearity(path[p], path[p+1], path[p+2]):
                path.remove(path[p+1])
                print("Removed: ", path[p+1])
            else:
                p += 1
                print("Go on with: ", p)
        return path

# Implementation from course

# Define a function to take three points and test for collinearity by evaluating the determinant using the simplified version for the 2D case:
#
# $ det according Sarrus rule$

def collinearity(p1, p2, p3):
    collinear = False
    # Calculate the determinant of the matrix using integer arithmetic
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    # Set collinear to True if the determinant is equal to zero
    if det == 0:
        collinear = True
    return collinear

# ### 3D case
#
# Define a function to determine collinearity for the 3D case using the `np.linalg.det()` function. Introduce the `epsilon` threshold to deal with numerical precision issues and/or allow a tolerance for collinearity. If the determinant is less than `epsilon` then the points are collinear.

# Define a simple function to add a z coordinate of 1

def collinearity_float(p1, p2, p3, epsilon=1e-6):
    collinear = False
    # Create the matrix out of three points
    # Add points as rows in a matrix
    mat = np.vstack((point(p1), point(p2), point(p3)))
    # Calculate the determinant of the matrix.
    det = np.linalg.det(mat)
    # Set collinear to True if the determinant is less than epsilon
    if det < epsilon:
        collinear = True

    return collinear
#
