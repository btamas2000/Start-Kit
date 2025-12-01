#!/usr/bin/env python3
"""Visualization script for clustering results."""

import argparse
import json
from typing import List, Tuple, Dict, Any

def read_octile_map(path: str) -> Tuple[List[List[str]], int, int]:
    """Load an octile-format map file into a grid of characters."""
    with open(path, "r", encoding="ascii") as handle:
        lines = [line.rstrip("\n") for line in handle]

    if not lines or lines[0].strip().lower() != "type octile":
        raise ValueError("Expected octile map format")

    try:
        height = int(lines[1].split()[1])
        width = int(lines[2].split()[1])
    except (IndexError, ValueError) as exc:
        raise ValueError("Invalid height/width header") from exc

    try:
        map_index = lines.index("map") + 1
    except ValueError as exc:
        raise ValueError("Missing map header") from exc

    raw_rows = lines[map_index : map_index + height]
    if len(raw_rows) != height:
        raise ValueError("Map height mismatch")
    if any(len(row) != width for row in raw_rows):
        raise ValueError("Map width mismatch")

    grid = [list(row) for row in raw_rows]
    return grid, height, width

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize clustering results")
    parser.add_argument("map", help="Path to the original octile map file")
    parser.add_argument("json", help="Path to the clustering JSON output")
    return parser.parse_args()

def main() -> None:
    args = parse_args()
    
    # Read the original map
    grid, height, width = read_octile_map(args.map)
    
    # Read the JSON results
    with open(args.json, "r") as f:
        data = json.load(f)
    
    clusters = data["clusters"]
    stats = clusters["stats"]
    labels = clusters["labels"]
    cell_assignment = data["cell_assignment"]
    
    # Loop through each cluster
    for cluster_id in sorted(stats.keys()):
        cluster_id = int(cluster_id)  # JSON keys are strings
        print(f"\nCluster {cluster_id}:")
        print(f"  Label: {labels[str(cluster_id)]}")
        print(f"  Stats: {stats[str(cluster_id)]}")
        
        # Print the map with '0' for cells in this cluster
        for y in range(height):
            for x in range(width):
                if cell_assignment[y][x] == cluster_id:
                    print('0', end='')
                else:
                    print(grid[y][x], end='')
            print()
        
        # Wait for user input
        input("Press Enter to continue to next cluster...")

if __name__ == "__main__":
    main()