# Solo Project Task 2: Modern Navigation Systems

## Overview

This project implements modern navigation algorithms, including A\* and Contraction Hierarchies, enhanced with techniques like pre-computation and route caching. These algorithms are designed to efficiently handle large-scale pathfinding problems, making them suitable for real-world navigation systems.

## Features

- **A\* Algorithm**: An improvement over Dijkstra's algorithm, incorporating heuristic functions to optimize pathfinding.
- **Contraction Hierarchies**: A preprocessing technique to accelerate shortest path queries by simplifying the graph.
- **Route Caching**: Precomputes and caches important routes to reduce computation time during queries.
- **GeoJSON and CSV Outputs**: Supports exporting paths and stops in GeoJSON and CSV formats for visualization and analysis.
- **Performance Benchmarks**: Includes modules to benchmark the performance of A\* and Contraction Hierarchies.

## Project Structure

```
Solo_Project_Task2_CS163/
├── constructions/       # Classes for stops, paths, and route variables
├── network/             # Graph optimization and contraction hierarchies
├── shortestPath/        # A* and Dijkstra algorithms with caching
├── performance/         # Benchmarking modules
├── input/               # Input data files (JSON, CSV)
├── output/              # Output files (GeoJSON, CSV, TXT)
├── figures/             # Visualizations and performance graphs
├── main.py              # Entry point for the application
├── README.md            # Project documentation
```

## Key Components

### 1. **Graph Optimization**

- `Graph_Optimized` class initializes the graph with stops, paths, and route variables.
- Supports distance calculations and caching mechanisms.

### 2. **Shortest Path Algorithms**

- **A\* Algorithm**: Implements heuristic-based pathfinding.
- **Dijkstra's Algorithm**: Computes shortest paths for all pairs of nodes.
- **Fixed Caching**: Enhances A\* with precomputed fixed routes.

### 3. **Contraction Hierarchies**

- Preprocesses the graph to simplify queries.
- Adds shortcuts and removes unnecessary nodes to optimize performance.

### 4. **Performance Benchmarking**

- Benchmarks A\* and Contraction Hierarchies for various scenarios.
- Measures processing time and memory usage.

## Input Data

- **`vars.json`**: Contains route variables.
- **`stops.json`**: Defines stops with attributes like latitude, longitude, and zone.
- **`paths.json`**: Specifies paths between stops.

## Output Data

- **Shortest Path**: Saved in `output/shortestPath.txt`.
- **GeoJSON**: Exported to `output/geoJson.json` for visualization.
- **CSV**: Stops and paths exported to CSV for analysis.

## How to Run

1. Place input files (`vars.json`, `stops.json`, `paths.json`) in the `jsonFiles/` directory.
2. Run the main script:
   ```
   python main.py
   ```
3. Follow the prompts to input source and destination nodes.
4. Outputs will be saved in the `output/` directory.

## Dependencies

- Python 3.8+
- Required libraries: `tqdm`, `pyproj`, `pandas`
- Install dependencies using:
  ```
  pip install -r requirements.txt
  ```

## Figures

- Visualizations of important stops, zones, and performance metrics are available in the `figures/` directory.

## Future Enhancements

- Integrate real-time traffic data for dynamic pathfinding.
- Extend support for multi-modal transportation networks.
- Optimize memory usage for large-scale graphs.
