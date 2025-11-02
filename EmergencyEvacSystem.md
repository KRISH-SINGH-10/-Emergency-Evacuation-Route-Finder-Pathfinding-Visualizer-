## Ramdeobaba University, Nagpur
## Department of Computer Science and Engineering
## Session: 2025-26

## Subject: Design and Analysis of Algorithms (DAA) Lab Project
## III Semester
## LAB PROJECT REPORT

## Group Members with Roll number and Section:

	HARSHVARDHAN TALOKAR    11   A1 
	KRISH SINGH 		    03   A1
	SHAURYA KUMAR  	        08   A1
	AMBER SHUKLA		    42   A1





#### TITLE: Emergency Evacuation Route Finder (Pathfinding Visualizer)

## Objectives: 
To design and implement a GUI-based pathfinding visualizer using Python’s Tkinter.
To demonstrate and compare multiple pathfinding algorithms:
Breadth-First Search (BFS)
Depth-First Search (DFS)
Dijkstra’s Algorithm
A-Star (A*) Algorithm
To allow users to interactively create grids, add obstacles, and define start and goal points.
To visualize how each algorithm explores the grid and finds a path.
To compare algorithms based on path length, nodes visited, and execution time.
To help students learn graph traversal and shortest-path algorithms through visualization.







## Introduction:
Pathfinding is a fundamental problem in computer science, robotics, and AI, where the objective is to find the shortest or most efficient path between two points.
This project implements a graphical pathfinding visualizer using Python’s Tkinter framework.
The system simulates an emergency evacuation scenario, showing how various algorithms search for an exit from a grid (representing a building or area) while avoiding obstacles.
It helps users understand the logic, differences, and performance of four major algorithms.
Features:
Interactive GUI built with Tkinter.
Customizable grid size.
Multiple drawing tools (Start, Goal, Obstacle, Eraser).
Automatic comparison of all four algorithms.
Animated visualization of the shortest path.

## Algorithms/Technique used:
algorithms used in the project, with pseudocode and explanations:
# 1️. Breadth-First Search (BFS)
Type: Uninformed Search
Nature: Level-Order Traversal (uses Queue)
Goal: Finds the shortest path in an unweighted grid.
Algorithm Steps:
1. Initialize a queue with the start node.
2. Mark the start node as visited.
3. While the queue is not empty:
   a. Dequeue the front node.
   b. If it is the goal, stop.
   c. For each unvisited neighbor that isn’t an obstacle:
      i. Mark it as visited.
      ii. Enqueue it with its path.
4. Return the shortest path found.
Explanation:
BFS explores nodes in layers, moving outward evenly from the start.
Because it expands equally in all directions, it guarantees the shortest path in terms of steps.

# 2️. Depth-First Search (DFS)
Type: Uninformed Search
 Nature: Recursive / Stack-Based
 Goal: Explore as deep as possible before backtracking.
Algorithm Steps:
1. Push the start node onto a stack.
2. While the stack is not empty:
   a. Pop the top node.
   b. If it is the goal, stop.
   c. For each unvisited neighbor:
      i. Push it onto the stack.
3. Return the first path found.

Explanation:
 DFS follows one path deeply until it reaches a dead end, then backtracks.
 It does not guarantee the shortest path, but it explores depth effectively.
3️. Dijkstra’s Algorithm
Type: Weighted Graph Shortest Path
Nature: Greedy
Goal: Finds the shortest path with minimal total cost.
Algorithm Steps:
1. Initialize a priority queue with the start node (distance = 0).
2. While queue not empty:
   a. Dequeue the node with the smallest distance.
   b. For each neighbor:
      i. Calculate new distance = current distance + 1.
      ii. If this is smaller than the stored distance, update and enqueue.
# 3. Stop when the goal is reached.
Explanation:
 Dijkstra ensures that the first time a node is reached, it is via the shortest possible distance.
 In an unweighted grid, Dijkstra and BFS perform similarly, but Dijkstra handles weighted costs.






# 4️. A-Star (A*) Algorithm
Type: Informed Search (uses Heuristic)
Heuristic Used: Manhattan Distance
Goal: Find shortest path efficiently by estimating remaining distance.
Algorithm Steps:
1. Initialize open list (priority queue) with start node.
2. For each node, maintain:
   - g = cost from start
   - h = heuristic estimate to goal
   - f = g + h
3. Pick a node with the smallest f.
4. For each neighbor:
   a. Calculate new g and f.
   b. If the new f is smaller, update and enqueue.
5. Continue until the goal is reached.

Explanation:
A* combines Dijkstra’s guaranteed optimality with heuristic guidance to reach the goal faster.
The Manhattan heuristic guides the search toward the goal, avoiding unnecessary exploration.

Time Complexity and its explanation:
Algorithm
Time Complexity
Space Complexity
Notes
BFS
O(V + E)
O(V)
Explores all vertices and edges once; shortest path guaranteed in unweighted grid.
DFS
O(V + E)
O(V)
Can go deep before backtracking; may not find the shortest path.
Dijkstra
O((V + E) log V)
O(V)
Uses priority queue; handles weighted paths efficiently.
A*
O((V + E) log V)
O(V)
Faster than Dijkstra due to heuristic pruning.

Where:
V = number of cells (nodes) in the grid
E = number of connections (edges) between adjacent cells




## Results:
Graphical Interface:

The GUI window titled “Emergency Evacuation Route Finder” opens.
You can specify the grid size (e.g., 10×10).
Select Start Point, Exit Point, and draw Obstacles.
Click Find Path to run all algorithms.
Output Window:
 A result table is generated:
Algorithm Path     Steps   Nodes   Time (ms)
---------------------------------------------
BFS        Yes      21      60      0.00
DFS        Yes      47      60      0.00
Dijkstra   Yes      21      78      0.00
A*         Yes      21      58      0.00

 Shortest path found by A* (14 steps).



## Animation:
 The shortest path is drawn cell-by-cell in blue using smooth animation.
 If no valid route exists, a message “No path found by any algorithm” appears.
Screenshots:
Empty grid window.
 
Start and Goal nodes placed.

Obstacles drawn.

Traversal in progress

Results table shown in text box


## Conclusion and future scope:

The project successfully demonstrates multiple pathfinding algorithms in an interactive visual form.It clearly shows the difference between uninformed (BFS, DFS) and informed (Dijkstra, A*) searches. The visualization helps understand algorithm behavior, path optimality, and performance trade-offs.

## Future Scope:
Add diagonal movements (8-directional search).
Allow custom movement costs or weighted obstacles.
Add maze generation and random testing features.
Export result data (time, nodes) as a CSV report.
Convert the visualizer into a web-based React or JavaScript application.
Integrate with real-world maps for route-planning simulation.



