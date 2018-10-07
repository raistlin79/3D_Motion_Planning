# 3D_Motion_Planning
FCND Project - 3D Motion Planning

The README at hand provides an overview where and how project criteria are met.
I realized two solutions for the 3D Motion Planning Project. On the one hand the "Minimal Solution" with an augmented A*. On the other a Graph based planning.

A* solution is contained in motion_planning.py and planning_utils.py
Graph solution is available in my_motion_planning_graph.py and my_planning_utils_graph.py

1. Explain the Starter Code
While the code contains markups at all relevant steps (explaining imports, conversions, etc.) the main algorithm goes as follows:
 - Switching State to planning
 - Get home position from cvs file and set it as global home. Needed as reference for global_to_local for further local calculations.
 - Load colliders.cvs and build grid from it.

    - For graph solution edges have to calculated as well. The edges have to be put together to a graph.

 - Ask for goal coordinates (For example: lon = -122.39725, lat = 37.79392).
 - Derive local goal coordinates and calculate grid goal.

    - Again, only for graph solution you have to match grid_start and grid_goal with corresponding graph edges using closest_point.

 - Now you have a start and goal coordinate on the grid/graph and can find a way between avoiding obstacles of course. I used A* in both cases. In the minimal solution with added diagonals. For the graph solution adaption have to be made to use voronoi points (networkx).

 - Check if path-knots can be reduced (using method prune_path with was added to the util files). Collinearty check.
 - At last path has to be converted to waypoints (cast to int) and sent to simulation to visualize flight.

2. Implementing Your Path Planning Algorithm

Please find comments within the code marked with TODO.

3. Executing the flight

I tried a few spots.
