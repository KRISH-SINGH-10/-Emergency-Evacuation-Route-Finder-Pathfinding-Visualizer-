# Emergency Evacuation Route Finder (Pathfinding Visualizer)

A Python Tkinter app that finds the way out of a building during an emergency and shows how four pathfinding algorithms search for it. You draw a grid with walls, a start and an exit; the app runs **BFS, DFS, Dijkstra and A\***, animates each search, and compares them on path length, nodes visited and run time.

Built as the Design and Analysis of Algorithms lab project (Semester III, 2025-26) at Ramdeobaba University, Nagpur.

![Visualizer](SYSTEM%20IMAGES/a1.png)

## Features

- Interactive grid with Start, Goal, Obstacle and Eraser tools, and a custom grid size.
- Runs all four algorithms on the same grid and animates the path each one finds.
- Results table comparing path length, nodes explored and execution time.

## Algorithms

| Algorithm | Type | Shortest path? |
|---|---|---|
| Breadth-First Search | Uninformed, queue | Yes, on an unweighted grid |
| Depth-First Search | Uninformed, stack | No |
| Dijkstra | Weighted shortest path | Yes |
| A\* | Heuristic (Manhattan distance) | Yes, with an admissible heuristic |

## In this repo

- [EmergencyEvacSystem.md](EmergencyEvacSystem.md): the full project report, with pseudocode, results and future scope.
- `SYSTEM IMAGES/`: screenshots of the app.

## Team

Krish Singh, Harshvardhan Talokar, Shaurya Kumar, Amber Shukla
