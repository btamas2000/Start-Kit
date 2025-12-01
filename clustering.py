#!/usr/bin/env python3
"""Semantic clustering tool for MAPF grid maps."""

import argparse
import json
import math
import os
from collections import deque
from typing import Dict, Iterable, List, Optional, Set, Tuple


Coord = Tuple[int, int]


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


def neighbors4(row: int, col: int, height: int, width: int) -> Iterable[Coord]:
	if row > 0:
		yield row - 1, col
	if row + 1 < height:
		yield row + 1, col
	if col > 0:
		yield row, col - 1
	if col + 1 < width:
		yield row, col + 1


def compute_clearance(grid: List[List[str]]) -> List[List[int]]:
	"""Compute Manhattan distance to the nearest obstacle for each cell."""

	height = len(grid)
	width = len(grid[0]) if height else 0
	dist = [[math.inf] * width for _ in range(height)]
	queue = deque()

	for r in range(height):
		for c in range(width):
			if not is_free(grid[r][c]):
				dist[r][c] = 0
				queue.append((r, c))

	if not queue:
		return [[0] * width for _ in range(height)]

	while queue:
		row, col = queue.popleft()
		for nr, nc in neighbors4(row, col, height, width):
			if dist[nr][nc] > dist[row][col] + 1:
				dist[nr][nc] = dist[row][col] + 1
				queue.append((nr, nc))

	# Replace infinities (purely free map) with maximum observed distance
	finite_values = [dist[r][c] for r in range(height) for c in range(width) if dist[r][c] < math.inf]
	fallback = max(finite_values, default=0)
	for r in range(height):
		for c in range(width):
			if dist[r][c] == math.inf:
				dist[r][c] = fallback

	return dist


def find_components(free_mask: List[List[bool]]) -> List[List[Coord]]:
	height = len(free_mask)
	width = len(free_mask[0]) if height else 0
	seen = [[False] * width for _ in range(height)]
	components: List[List[Coord]] = []

	for r in range(height):
		for c in range(width):
			if not free_mask[r][c] or seen[r][c]:
				continue
			queue = deque([(r, c)])
			seen[r][c] = True
			component: List[Coord] = []
			while queue:
				row, col = queue.popleft()
				component.append((row, col))
				for nr, nc in neighbors4(row, col, height, width):
					if free_mask[nr][nc] and not seen[nr][nc]:
						seen[nr][nc] = True
						queue.append((nr, nc))
			components.append(component)

	return components


def select_seed_cells(
	component: List[Coord],
	clearance: List[List[int]],
	corridor_threshold: int,
	min_seed_clearance: int,
	min_seed_spacing: int,
) -> List[Coord]:
	candidates: List[Tuple[int, Coord]] = []
	for row, col in component:
		if clearance[row][col] <= corridor_threshold:
			continue
		if clearance[row][col] < min_seed_clearance:
			continue
		local_max = True
		for nr, nc in neighbors4(row, col, len(clearance), len(clearance[0])):
			if clearance[nr][nc] > clearance[row][col]:
				local_max = False
				break
		if local_max:
			candidates.append((clearance[row][col], (row, col)))

	if not candidates and component:
		best = max(component, key=lambda xy: clearance[xy[0]][xy[1]])
		return [best]

	candidates.sort(reverse=True)
	seeds: List[Coord] = []
	for _, coord in candidates:
		if all(abs(coord[0] - sr) + abs(coord[1] - sc) >= min_seed_spacing for sr, sc in seeds):
			seeds.append(coord)
	return seeds


def grow_open_clusters(
	cluster_map: List[List[Optional[int]]],
	free_mask: List[List[bool]],
	clearance: List[List[int]],
	seeds: List[Coord],
	corridor_threshold: int,
	next_cluster_id: int,
	cluster_types: Dict[int, str],
) -> int:
	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0
	queue = deque()

	for seed in seeds:
		row, col = seed
		if cluster_map[row][col] is not None:
			continue
		cluster_id = next_cluster_id
		next_cluster_id += 1
		cluster_map[row][col] = cluster_id
		cluster_types[cluster_id] = "open_area"
		queue.append((row, col, cluster_id))

	while queue:
		cr, cc, cluster_id = queue.popleft()
		for nr, nc in neighbors4(cr, cc, height, width):
			if not free_mask[nr][nc]:
				continue
			if cluster_map[nr][nc] is not None:
				continue
			if clearance[nr][nc] <= corridor_threshold:
				continue
			cluster_map[nr][nc] = cluster_id
			queue.append((nr, nc, cluster_id))

	return next_cluster_id


def assign_corridor_clusters(
	cluster_map: List[List[Optional[int]]],
	free_mask: List[List[bool]],
	clearance: List[List[int]],
	corridor_threshold: int,
	next_cluster_id: int,
	cluster_types: Dict[int, str],
) -> int:
	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0

	for r in range(height):
		for c in range(width):
			if not free_mask[r][c]:
				continue
			if cluster_map[r][c] is not None:
				continue
			if clearance[r][c] > corridor_threshold:
				continue
			cluster_id = next_cluster_id
			next_cluster_id += 1
			queue = deque([(r, c)])
			cluster_map[r][c] = cluster_id
			while queue:
				cr, cc = queue.popleft()
				for nr, nc in neighbors4(cr, cc, height, width):
					if not free_mask[nr][nc]:
						continue
					if cluster_map[nr][nc] is not None:
						continue
					if clearance[nr][nc] > corridor_threshold:
						continue
					cluster_map[nr][nc] = cluster_id
					queue.append((nr, nc))
			cluster_types[cluster_id] = "corridor"

	return next_cluster_id


def assign_residual_clusters(
	cluster_map: List[List[Optional[int]]],
	free_mask: List[List[bool]],
	clearance: List[List[int]],
	corridor_threshold: int,
	next_cluster_id: int,
	cluster_types: Dict[int, str],
) -> int:
	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0

	for r in range(height):
		for c in range(width):
			if not free_mask[r][c]:
				continue
			if cluster_map[r][c] is not None:
				continue
			cluster_id = next_cluster_id
			next_cluster_id += 1
			queue = deque([(r, c)])
			cluster_map[r][c] = cluster_id
			while queue:
				cr, cc = queue.popleft()
				for nr, nc in neighbors4(cr, cc, height, width):
					if not free_mask[nr][nc]:
						continue
					if cluster_map[nr][nc] is not None:
						continue
					# Allow residual clusters to span wider cells but stop at corridors
					if clearance[nr][nc] <= corridor_threshold:
						continue
					cluster_map[nr][nc] = cluster_id
					queue.append((nr, nc))
			cluster_types[cluster_id] = "residual_open"

	return next_cluster_id


def merge_small_clusters(
	cluster_map: List[List[Optional[int]]],
	cluster_types: Dict[int, str],
	min_cluster_size: int,
) -> None:
	if min_cluster_size <= 1:
		return

	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0
	cluster_cells: Dict[int, List[Coord]] = {}

	for r in range(height):
		for c in range(width):
			cluster_id = cluster_map[r][c]
			if cluster_id is None:
				continue
			cluster_cells.setdefault(cluster_id, []).append((r, c))

	changed = True
	while changed:
		changed = False
		small = [cid for cid, cells in cluster_cells.items() if len(cells) < min_cluster_size]
		if not small:
			break
		for cid in small:
			cells = cluster_cells.get(cid)
			if not cells:
				continue
			neighbors: Set[int] = set()
			for row, col in cells:
				for nr, nc in neighbors4(row, col, height, width):
					other = cluster_map[nr][nc]
					if other is None or other == cid:
						continue
					neighbors.add(other)
			if not neighbors:
				continue
			same_type = [nid for nid in neighbors if cluster_types.get(nid) == cluster_types.get(cid)]
			candidates = same_type if same_type else list(neighbors)
			target = max(candidates, key=lambda nid: len(cluster_cells.get(nid, [])))
			for row, col in cells:
				cluster_map[row][col] = target
				cluster_cells.setdefault(target, []).append((row, col))
			cluster_cells.pop(cid, None)
			cluster_types.pop(cid, None)
			changed = True


def compute_cluster_stats(
	cluster_map: List[List[Optional[int]]],
	clearance: List[List[int]],
	cluster_types: Dict[int, str],
) -> Dict[int, Dict[str, object]]:
	stats: Dict[int, Dict[str, object]] = {}
	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0

	for r in range(height):
		for c in range(width):
			cluster_id = cluster_map[r][c]
			if cluster_id is None:
				continue
			record = stats.setdefault(
				cluster_id,
				{
					"id": cluster_id,
					"type": cluster_types.get(cluster_id, "unknown"),
					"cell_count": 0,
					"bbox": {
						"min_row": r,
						"max_row": r,
						"min_col": c,
						"max_col": c,
					},
					"clearance_sum": 0,
					"clearance_min": clearance[r][c],
					"clearance_max": clearance[r][c],
					"cells": [],
				},
			)

			record["cell_count"] += 1
			record["bbox"]["min_row"] = min(record["bbox"]["min_row"], r)
			record["bbox"]["max_row"] = max(record["bbox"]["max_row"], r)
			record["bbox"]["min_col"] = min(record["bbox"]["min_col"], c)
			record["bbox"]["max_col"] = max(record["bbox"]["max_col"], c)
			record["clearance_sum"] += clearance[r][c]
			record["clearance_min"] = min(record["clearance_min"], clearance[r][c])
			record["clearance_max"] = max(record["clearance_max"], clearance[r][c])
			record["cells"].append([r, c])

	for record in stats.values():
		if record["cell_count"]:
			record["avg_clearance"] = record["clearance_sum"] / record["cell_count"]
		else:
			record["avg_clearance"] = 0.0
		record.pop("clearance_sum", None)

	return stats


def compute_adjacency(cluster_map: List[List[Optional[int]]]) -> Dict[int, Set[int]]:
	height = len(cluster_map)
	width = len(cluster_map[0]) if height else 0
	adjacency: Dict[int, Set[int]] = {}

	for r in range(height):
		for c in range(width):
			current = cluster_map[r][c]
			if current is None:
				continue
			nbrs = adjacency.setdefault(current, set())
			for nr, nc in neighbors4(r, c, height, width):
				other = cluster_map[nr][nc]
				if other is None or other == current:
					continue
				nbrs.add(other)
				adjacency.setdefault(other, set()).add(current)

	return adjacency


def stringify_cluster_grid(cluster_map: List[List[Optional[int]]]) -> List[List[Optional[int]]]:
	"""Prepare a JSON-friendly representation of the cluster grid."""

	return [[cell if cell is not None else None for cell in row] for row in cluster_map]


def run_clustering(
	map_path: str,
	output_path: str,
	corridor_threshold: int,
	min_seed_clearance: int,
	min_seed_spacing: int,
	min_cluster_size: int,
) -> None:
	grid, height, width = read_octile_map(map_path)
	free_mask = [[is_free(grid[r][c]) for c in range(width)] for r in range(height)]
	clearance = compute_clearance(grid)
	components = find_components(free_mask)

	cluster_map: List[List[Optional[int]]] = [[None] * width for _ in range(height)]
	cluster_types: Dict[int, str] = {}
	next_cluster_id = 0

	for component in components:
		seeds = select_seed_cells(
			component,
			clearance,
			corridor_threshold,
			min_seed_clearance,
			min_seed_spacing,
		)
		next_cluster_id = grow_open_clusters(
			cluster_map,
			free_mask,
			clearance,
			seeds,
			corridor_threshold,
			next_cluster_id,
			cluster_types,
		)

	next_cluster_id = assign_corridor_clusters(
		cluster_map,
		free_mask,
		clearance,
		corridor_threshold,
		next_cluster_id,
		cluster_types,
	)

	next_cluster_id = assign_residual_clusters(
		cluster_map,
		free_mask,
		clearance,
		corridor_threshold,
		next_cluster_id,
		cluster_types,
	)

	merge_small_clusters(cluster_map, cluster_types, min_cluster_size)

	cluster_stats = compute_cluster_stats(cluster_map, clearance, cluster_types)
	adjacency = compute_adjacency(cluster_map)

	output = {
		"map": {
			"path": os.path.abspath(map_path),
			"width": width,
			"height": height,
		},
		"parameters": {
			"corridor_threshold": corridor_threshold,
			"min_seed_clearance": min_seed_clearance,
			"min_seed_spacing": min_seed_spacing,
			"min_cluster_size": min_cluster_size,
		},
		"cluster_count": len(cluster_stats),
		"clusters": [
			{
				**record,
				"neighbors": sorted(adjacency.get(record["id"], set())),
			}
			for record in sorted(cluster_stats.values(), key=lambda item: item["id"])
		],
		"cluster_grid": stringify_cluster_grid(cluster_map),
	}

	with open(output_path, "w", encoding="utf-8") as handle:
		json.dump(output, handle, indent=2)


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Semantic clustering for MAPF maps")
	parser.add_argument("map", help="Path to octile map file")
	parser.add_argument(
		"output",
		help="Path to write clustering JSON",
	)
	parser.add_argument(
		"--corridor-threshold",
		type=int,
		default=1,
		help="Max clearance treated as corridor (default: 1)",
	)
	parser.add_argument(
		"--min-seed-clearance",
		type=int,
		default=3,
		help="Clearance required for open-area seed selection (default: 3)",
	)
	parser.add_argument(
		"--min-seed-spacing",
		type=int,
		default=6,
		help="Minimum Manhattan spacing between seeds (default: 6)",
	)
	parser.add_argument(
		"--min-cluster-size",
		type=int,
		default=4,
		help="Clusters smaller than this get merged into neighbors (default: 4)",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	run_clustering(
		map_path=args.map,
		output_path=args.output,
		corridor_threshold=args.corridor_threshold,
		min_seed_clearance=args.min_seed_clearance,
		min_seed_spacing=args.min_seed_spacing,
		min_cluster_size=args.min_cluster_size,
	)


if __name__ == "__main__":
	main()

