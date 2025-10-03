# AI-Maze-pathfinding
An Automatic Maze Generator and Solver using Python and DFS/BFS Algorithms

### ⚙️ Prerequisites

You must have Python 3.x installed.
The project requires the following libraries:

- `numpy`
- `matplotlib`
- `collections` (standard library)



### 💻 Implementation & Technologies

- **Language:** Python 3.x
- **Visualization:** `matplotlib` is used to dynamically plot the maze generation, the search progress (expanded nodes), and the final path found by the algorithm.
- **Data Structure:** The maze is converted into a **graph** (using a `defaultdict` adjacency list) for efficient traversal by the search algorithms.
- **Algorithms Implemented:**
    - **Informed Search:** A* Search (Manhattan Distance heuristic), Greedy Best-First Search.
    - **Uninformed Search:** Breadth-First Search (BFS), Depth-First Search (DFS), Iterative Deepening Search (IDS).
- **Maze Generation:** Prim's Algorithm is used to create a perfect maze (no loops, every point reachable).

### 🎮 How to Use

1. **Enter Size:** The program will first prompt you to enter the size of the maze/graph (e.g., `25`).
2. **Choose Algorithm:** You will then be asked to select one of the implemented pathfinding algorithms: `A*`, `BFS`, `DFS`, `Greedy`, or `IDS`.

The program will then:
- Generate a random maze using Prim's algorithm.
- Visualize the chosen algorithm's search process step-by-step.
- Display the **final path** found.
- Show metrics: total **expanded nodes** and **elapsed time**.
### 📁 Project Structure

- `main.py`: Main execution file. Handles user input, maze generation, graph conversion, and calls the selected algorithm.
- `algorithm.py`: Contains the implementations for all pathfinding algorithms (A*, BFS, DFS, Greedy, IDS). *Note: This file should not contain graphical elements.*
- `graphics.py`: Contains all visualization functions using Matplotlib (e.g., `update_visualization`, `reconstruct_path`).
- `mazeGenerator.py`: Implements Prim's Algorithm for dynamic maze creation.
