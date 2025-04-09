# RRT/RRT* Path Planning Simulator

This project implements a **Rapidly-exploring Random Tree (RRT)** and **RRT Star** path planning simulator using **PyQt5** for the GUI and **NumPy** for calculations.

## Features
- Simulates **RRT** and **RRT*** algorithms for path planning.
- With and without obstacles
- Visualize obstacles, paths, and tree growth in real-time.
- Adjustable parameters: **step size**, **goal sampling rate**, **planning constant**, and **goal position**.
- **Path cost** and **Time steps** are displayed.

## Requirements
Install dependencies:
```bash
pip install PyQt5 numpy matplotlib
```

## Usage
1. Run the simulator:
   ```bash
   python simulator.py
   ```

2. Set parameters via the GUI (step size, goal position, etc.).
3. Click **"Start"** to begin the simulation. The tree will grow, and a path will be found.
4. Press **"Reset"** to clear and start again.

## Algorithms
- **RRT**: Quickly explores the space, finding a feasible path.
- **RRT***: Optimizes the path by rewiring the tree, ensuring an optimal solution over time.
