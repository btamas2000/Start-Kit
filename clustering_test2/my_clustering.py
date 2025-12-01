#!/usr/bin/env python3
"""Semantic clustering tool for MAPF grid maps."""

import argparse
from collections import deque
from typing import Dict, List, Tuple
import heapq
import json

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

def is_free(cell: str) -> bool:
	return cell != "@"

def bfs(grid: List[List[str]], width: int, height: int) -> List[List[int]]:
	areas = [[-1 for _ in range(width)] for _ in range(height)]
	area_counter = 0

	for y in range(height):
		for x in range(width):
			if areas[y][x] == -1 and is_free(grid[y][x]):
				q = deque()
				q.append((y, x))
				areas[y][x] = area_counter
				
				while q:
					cy, cx = q.popleft()
					for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
						ny, nx = cy + dy, cx + dx
						if 0 <= nx < width and 0 <= ny < height:
							if areas[ny][nx] == -1 and is_free(grid[ny][nx]):
								areas[ny][nx] = area_counter
								q.append((ny, nx))
				
				area_counter += 1
				
	return areas

def pad_grid(grid: List[List[str]], width: int, height: int) -> List[List[str]]:
	padded_grid = [["x" for _ in range(width + 2)] for _ in range(height + 2)]
	for y in range(height + 2):
		for x in range(width + 2):
			if x == 0 or x == width + 1 or y == 0 or y == height + 1:
				padded_grid[y][x] = "@"
			else:
				padded_grid[y][x] = grid[y - 1][x - 1]
	return padded_grid

def distance_transform(grid: List[List[str]], width: int, height: int) -> List[List[int]]:
    
	padded_grid = pad_grid(grid, width, height)
	new_width = width + 2
	new_height = height + 2
    
	dists = [[-1 for _ in range(new_width)] for _ in range(new_height)]
    
	q = deque()

	for y in range(new_height):
		for x in range(new_width):
			if not is_free(padded_grid[y][x]):
				dists[y][x] = 0
				q.append((y, x))
    
	directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

	while q:
		cy, cx = q.popleft()
		for dy, dx in directions:
			ny, nx = cy + dy, cx + dx
			if 0 <= nx < new_width and 0 <= ny < new_height:
				if dists[ny][nx] == -1:
					dists[ny][nx] = dists[cy][cx] + 1
					q.append((ny, nx))
    
	return [row[1:new_width-1] for row in dists[1:new_height-1]]

def find_global_min_max(distances: List[List[int]], width: int, height: int) -> Tuple[int, int]:
    min_val = -1
    max_val = -1
    for y in range(height):
        for x in range(width):
            if min_val == -1:
                min_val = distances[y][x]
            if max_val == -1:
                max_val = distances[y][x]
            if distances[y][x] < min_val:
                min_val = distances[y][x]
            if distances[y][x] > max_val:
                max_val = distances[y][x]
                
    return min_val, max_val

def select_cells_at_distance(distances: List[List[int]], width: int, height: int, target_distance: int) -> List[Tuple[int, int]]:
    selected_cells = []
    for y in range(height):
        for x in range(width):
            if distances[y][x] == target_distance:
                selected_cells.append((y, x))
                
    return selected_cells

def cell_in_plateau(y: int, x: int, plateaus: List[Tuple[int, List[Tuple[int, int]]]]) -> bool:
    for _, plateau in plateaus:
        if (y, x) in plateau:
            return True
        
    return False
                
def find_local_maximas(distances: List[List[int]], width: int, height: int, threshold: int) -> List[Tuple[int, List[Tuple[int, int]]]]:
    min_v, max_v = find_global_min_max(distances, width, height)
    local_maxima_plateaus = []
    false_plateaus = set()
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for i in reversed(range(min_v, max_v + 1)):
        if i < threshold:
            break
        
        cells_i = select_cells_at_distance(distances, width, height, i)
        for (y, x) in cells_i:
            if cell_in_plateau(y, x, local_maxima_plateaus):
                continue
            
            if (y, x) in false_plateaus:
                continue
            
            potential_maximum_plateau = [(y, x)]
            false_plateau = False
            q = deque()
            q.append((y, x))
            while q and not false_plateau:
                cy, cx = q.popleft()
                for dy, dx in directions:
                    ny, nx = cy + dy, cx + dx
                    if 0 <= nx < width and 0 <= ny < height:
                        if distances[ny][nx] > distances[cy][cx]:
                            false_plateau = True
                        elif (ny, nx) in false_plateaus:
                            false_plateau = True
                        elif distances[ny][nx] == distances[cy][cx]:
                            if (ny, nx) not in potential_maximum_plateau:
                                potential_maximum_plateau.append((ny, nx))
                                q.append((ny, nx))
            
            if not false_plateau:
                local_maxima_plateaus.append((i, potential_maximum_plateau))
            else:
                for (y, x) in potential_maximum_plateau:
                    false_plateaus.add((y, x))

    return local_maxima_plateaus

# Take the local maxima plateaus and expand them in maxima-1 rings to form initial clusters
# Limited by drop_fraction of maxima value
def form_initial_clusters(local_maxima: List[Tuple[int, List[Tuple[int, int]]]], distances: List[List[int]], width: int, height: int, drop_fraction: float) -> List[Dict[str, any]]:
	cluster_map = [[-1 for _ in range(width)] for _ in range(height)]
	q = deque()
 
	for i in range(len(local_maxima)):
		for (y, x) in local_maxima[i][1]:
			cluster_map[y][x] = i
			q.append((y, x, local_maxima[i][0], local_maxima[i][0] - 2)) # -2 because logic but expansion is maxima-1 rings

	directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
		
	while q:
		cy, cx, maxima, max_expansion = q.popleft()
		for dy, dx in directions:
			ny, nx = cy + dy, cx + dx
			if 0 <= nx < width and 0 <= ny < height:
				if distances[ny][nx] <= distances[cy][cx] and distances[ny][nx] > 0 and distances[ny][nx] >= drop_fraction * maxima:
					if cluster_map[ny][nx] == -1:
						cluster_map[ny][nx] = cluster_map[cy][cx]
						if max_expansion > 0:
							q.append((ny, nx, maxima, max_expansion - 1))
       
    # Collect clusters
	clusters = []
	for i in range(len(local_maxima)):
		cluster_cells = []
		for y in range(height):
			for x in range(width):
				if cluster_map[y][x] == i:
					cluster_cells.append((y, x))
		clusters.append({
			"id": i,
			"degree": 0,
			"maxima": local_maxima[i][0],
			"neighbors": [],
			"size": len(cluster_cells),
			"cells": cluster_cells
		})
		
	return clusters

# See if enough unassigned dt=1 cells can form a cluster on their own
# if size>min_size, create new cluster
def cluster_dt_1_cells(clusters: List[Dict[str, any]], distances: List[List[int]], width: int, height: int) -> List[Dict[str, any]]:
	updated_clusters = clusters.copy()
	cluster_id = len(updated_clusters)
	
	dt_1_map = [[-1 for _ in range(width)] for _ in range(height)]
	for y in range(height):
		for x in range(width):
			if not distances[y][x] == 1:
				dt_1_map[y][x] = -2

	directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

	for y in range(height):
		for x in range(width):
			if dt_1_map[y][x] == -1:
				q = deque()
				q.append((y, x))
				current_cluster_cells = [(y, x)]
				
				while q:
					cy, cx = q.popleft()
					for dy, dx in directions:
						ny, nx = cy + dy, cx + dx
						if 0 <= nx < width and 0 <= ny < height:
							if dt_1_map[ny][nx] == -1 and (ny, nx) not in current_cluster_cells:
								current_cluster_cells.append((ny, nx))
								q.append((ny, nx))
				
				if len(current_cluster_cells) >= 4:
					for (cy, cx) in current_cluster_cells:
						dt_1_map[cy][cx] = cluster_id
					updated_clusters.append({
						"id": cluster_id,
						"degree": 0,
						"maxima": 1,
						"neighbors": [],
						"size": len(current_cluster_cells),
						"cells": current_cluster_cells
					})
					cluster_id += 1
				else:
					for (cy, cx) in current_cluster_cells:
						dt_1_map[cy][cx] = -2

	return updated_clusters

# Assign any cells without a cluster to the neighboring cluster
# (expand first from lower dt maxima clusters to not let big clusters swallow meaningful info)
def assign_leftover_cells(clusters: List[Dict[str, any]], distances: List[List[int]], width: int, height: int) -> List[Dict[str, any]]:
	updated_clusters = clusters.copy()
	
	sorted_clusters = sorted(updated_clusters, key=lambda c: c["maxima"])
 
	assigning_map = [[-1 for _ in range(width)] for _ in range(height)]
 
	q = deque()
 
	for cluster in sorted_clusters:
		for (y, x) in cluster["cells"]:
			assigning_map[y][x] = cluster["id"]
			q.append((y, x, cluster["id"]))
	
	directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
		
	while q:
		cy, cx, cluster_id = q.popleft()
		for dy, dx in directions:
			ny, nx = cy + dy, cx + dx
			if 0 <= nx < width and 0 <= ny < height:
				if assigning_map[ny][nx] == -1 and distances[ny][nx] > 0:
					assigning_map[ny][nx] = cluster_id
					q.append((ny, nx, cluster_id))
    
	for cluster in updated_clusters:
		cluster_cells = []
		for y in range(height):
			for x in range(width):
				if assigning_map[y][x] == cluster["id"]:
					cluster_cells.append((y, x))
		cluster["cells"] = cluster_cells
		cluster["size"] = len(cluster_cells)
	
	return updated_clusters

def update_cluster_neighbors(clusters: List[Dict[str, any]], width: int, height: int) -> List[Dict[str, any]]:
	updated_clusters = clusters.copy()
 
	cluster_map = [[-1 for _ in range(width)] for _ in range(height)]
	for cluster in updated_clusters:
		for (y, x) in cluster["cells"]:
			cluster_map[y][x] = cluster["id"]
 
	directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
 
	for cluster in updated_clusters:
		neighbor_set = set()
		for (y, x) in cluster["cells"]:
			for dy, dx in directions:
				ny, nx = y + dy, x + dx
				if 0 <= nx < width and 0 <= ny < height:
					neighbor_id = cluster_map[ny][nx]
					if neighbor_id != -1 and neighbor_id != cluster["id"]:
						neighbor_set.add(neighbor_id)
		cluster["neighbors"] = list(neighbor_set)
		cluster["degree"] = len(neighbor_set)
 
	return updated_clusters

def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Semantic clustering for MAPF maps")
	parser.add_argument("map", help="Path to octile map file")
	parser.add_argument("output", help="Path to write clustering JSON")
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	grid, height, width = read_octile_map(args.map)
	print(f"Loaded map of size {width}x{height}")
	for y in range(height): 
		for x in range(width):
			print(grid[y][x], end="")
		print("")
	areas = bfs(grid, width, height)
	for y in range(height):
		for x in range(width):
			if areas[y][x] == -1:
				print("¤", end="")
			else:
				print(f"{areas[y][x]}", end="")
		print("")
		
	distances = distance_transform(grid, width, height)
	for y in range(height):
		for x in range(width):
			if distances[y][x] == -1:
				print(" ", end="")
			else:
				print(f"{distances[y][x]}", end="")
		print("")

	print("")
	print("")
 
	plateaus = find_local_maximas(distances, width, height, 2)
  
	maxima_grid = [[-1 for _ in range(width)] for _ in range(height)]
	for m, plateau in plateaus:
		(y, x) = plateau[0]
		for (y, x) in plateau:
			maxima_grid[y][x] = m
	for y in range(height):
		for x in range(width):
			if maxima_grid[y][x] == -1:
				if is_free(grid[y][x]):
					print(".", end="")
				else:
					print("¤", end="")
			else:
				print(f"{maxima_grid[y][x]}", end="")
		print("")
   
	print("")
	print("")
 
	initial_clusters = form_initial_clusters(plateaus, distances, width, height, 0.5)
	print(f"Formed {len(initial_clusters)} initial clusters from local maxima")
	clusters_with_dt1 = cluster_dt_1_cells(initial_clusters, distances, width, height)
	print(f"Added DT=1 clusters, now have {len(clusters_with_dt1)} clusters")
	final_clusters = assign_leftover_cells(clusters_with_dt1, distances, width, height)
	print(f"Assigned leftover cells, now have {len(final_clusters)} final clusters")
	final_clusters = update_cluster_neighbors(final_clusters, width, height)
	print(f"Updated cluster neighbors")
 
	for cluster in final_clusters:
		print(f"Cluster {cluster['id']} size: {cluster['size']} maxima: {cluster['maxima']} degree: {cluster['degree']} neighbors: {cluster['neighbors']}")
		for y in range(height):
			for x in range(width):
				if (y, x) in cluster["cells"]:
					print('O', end='')
				else:
					if is_free(grid[y][x]):
						print('.', end='')
					else:
						print('¤', end='')	
			print("")

if __name__ == "__main__":
	main()

