import argparse
import sys

def load_map(filepath):
    """
    Reads a Moving AI .map file and returns:
    - grid (list of lists of chars)
    - width (int)
    - height (int)
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    # Parse header
    width = None
    height = None
    map_start_idx = None

    for i, line in enumerate(lines):
        if line.lower().startswith("width"):
            width = int(line.split()[1])
        elif line.lower().startswith("height"):
            height = int(line.split()[1])
        elif line.lower() == "map":
            map_start_idx = i + 1
            break

    if width is None or height is None or map_start_idx is None:
        raise ValueError("Invalid .map file: missing width/height/map header.")

    # Load grid lines
    grid_lines = lines[map_start_idx:]
    if len(grid_lines) != height:
        raise ValueError(f"Map height mismatch: expected {height}, got {len(grid_lines)}.")
    if not all(len(line) == width for line in grid_lines):
        raise ValueError("Some map lines do not match the specified width.")

    grid = [list(line) for line in grid_lines]
    return grid, width, height


def get_grid_coordinates(width, height, index):
    """Convert flattened index to (row, col)."""
    if index < 0 or index >= width * height:
        raise ValueError(f"Index {index} out of range for grid size {width}x{height}.")
    row = index // width
    col = index % width
    return row, col


def main():
    parser = argparse.ArgumentParser(
        description="Get (row, col) and character from a .map file using a flattened index."
    )
    parser.add_argument("map_file", help="Path to the .map file.")
    parser.add_argument("index", type=int, help="Flattened grid index.")

    args = parser.parse_args()

    try:
        grid, width, height = load_map(args.map_file)
        row, col = get_grid_coordinates(width, height, args.index)
        char = grid[row][col]

        print(f"Map size: {width}x{height}")
        print(f"Index {args.index} => (row={row}, col={col}) => Character: '{char}'")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
