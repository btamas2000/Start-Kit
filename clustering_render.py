#!/usr/bin/env python3
"""Render cluster assignments as ASCII maps."""

import argparse
import json
import os
from typing import List, Optional


def load_clustering(path: str) -> dict:
	with open(path, "r", encoding="utf-8") as handle:
		return json.load(handle)


def render_cluster_map(
	cluster_grid: List[List[Optional[int]]],
	cluster_id: int,
	mark_obstacles: str,
	inside_char: str,
	outside_char: str,
) -> List[str]:
	height = len(cluster_grid)
	width = len(cluster_grid[0]) if height else 0
	rows: List[str] = []
	for r in range(height):
		chars = []
		for c in range(width):
			value = cluster_grid[r][c]
			if value is None:
				chars.append(mark_obstacles)
			elif value == cluster_id:
				chars.append(inside_char)
			else:
				chars.append(outside_char)
		rows.append("".join(chars))
	return rows


def render_all_clusters(
	data: dict,
	mark_obstacles: str,
	inside_char: str,
	outside_char: str,
) -> List[str]:
	cluster_grid = data["cluster_grid"]
	clusters = data.get("clusters", [])
	lines: List[str] = []
	for cluster in sorted(clusters, key=lambda item: item["id"]):
		cluster_id = cluster["id"]
		lines.append(f"cluster {cluster_id}")
		rows = render_cluster_map(
			cluster_grid,
			cluster_id,
			mark_obstacles,
			inside_char,
			outside_char,
		)
		lines.extend(rows)
		lines.append("")
	return lines


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Render clustering JSON to ASCII maps")
	parser.add_argument("input", help="Path to clustering JSON output")
	parser.add_argument(
		"--output",
		help="Optional path to write the rendered text; defaults to stdout",
	)
	parser.add_argument(
		"--inside",
		default="O",
		help="Character for cells belonging to the cluster (default: O)",
	)
	parser.add_argument(
		"--outside",
		default="X",
		help="Character for free cells outside the cluster (default: X)",
	)
	parser.add_argument(
		"--obstacle",
		default="@",
		help="Character for obstacles (default: @)",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	data = load_clustering(args.input)
	rendered = render_all_clusters(
		data,
		mark_obstacles=args.obstacle,
		inside_char=args.inside,
		outside_char=args.outside,
	)

	output_text = "\n".join(rendered)
	if args.output:
		with open(args.output, "w", encoding="ascii") as handle:
			handle.write(output_text)
	else:
		print(output_text)


if __name__ == "__main__":
	main()

