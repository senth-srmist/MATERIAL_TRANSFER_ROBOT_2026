#!/usr/bin/env python3
"""
Tile Overlap Analyzer & Config Generator

Reads tile yaml files, analyzes overlaps, visualizes layout,
and generates a complete tiles_config.yaml file.

Usage:
    python3 tile_overlap_analyzer.py [maps_directory]
    python3 tile_overlap_analyzer.py /path/to/tile_manager/maps

Outputs:
    - tile_layout.png      : Visualization of tiles and overlaps (in maps_directory)
    - tiles_config.yaml    : Complete configuration file (in maps_directory/../config/)
"""

import yaml
import sys
from pathlib import Path
from datetime import datetime

try:
    from PIL import Image, ImageDraw

    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Note: PIL not available, text output only")


def load_tile_yaml(filepath):
    """Load tile yaml and extract info"""
    with open(filepath, "r") as f:
        data = yaml.safe_load(f)

    return {
        "image": data.get("image"),
        "resolution": data.get("resolution", 0.05),
        "origin": data.get("origin", [0, 0, 0]),
    }


def get_image_dimensions(pgm_path):
    """Get PGM image dimensions"""
    try:
        if HAS_PIL:
            img = Image.open(pgm_path)
            return img.width, img.height

        with open(pgm_path, "rb") as f:
            header = f.readline().decode().strip()
            if header not in ["P5", "P2"]:
                return None, None

            line = f.readline().decode().strip()
            while line.startswith("#"):
                line = f.readline().decode().strip()

            parts = line.split()
            if len(parts) == 2:
                return int(parts[0]), int(parts[1])
            else:
                width = int(parts[0])
                height = int(f.readline().decode().strip())
                return width, height
    except Exception as e:
        print(f"  Warning: Could not read {pgm_path}: {e}")
        return None, None


def calculate_bounds(tile_info, width, height):
    """Calculate world bounds from tile info"""
    x_min = tile_info["origin"][0]
    y_min = tile_info["origin"][1]
    resolution = tile_info["resolution"]

    x_max = x_min + width * resolution
    y_max = y_min + height * resolution

    return {
        "x_min": round(x_min, 2),
        "x_max": round(x_max, 2),
        "y_min": round(y_min, 2),
        "y_max": round(y_max, 2),
    }


def calculate_overlap(bounds1, bounds2):
    """Calculate overlap between two tiles"""
    x_overlap_min = max(bounds1["x_min"], bounds2["x_min"])
    x_overlap_max = min(bounds1["x_max"], bounds2["x_max"])
    y_overlap_min = max(bounds1["y_min"], bounds2["y_min"])
    y_overlap_max = min(bounds1["y_max"], bounds2["y_max"])

    if x_overlap_min < x_overlap_max and y_overlap_min < y_overlap_max:
        return {
            "x_min": round(x_overlap_min, 2),
            "x_max": round(x_overlap_max, 2),
            "y_min": round(y_overlap_min, 2),
            "y_max": round(y_overlap_max, 2),
            "width": round(x_overlap_max - x_overlap_min, 2),
            "height": round(y_overlap_max - y_overlap_min, 2),
            "area": round(
                (x_overlap_max - x_overlap_min) * (y_overlap_max - y_overlap_min), 2
            ),
        }
    return None


def analyze_tiles(maps_dir):
    """Analyze all tiles in directory"""
    maps_path = Path(maps_dir)

    tile_files = sorted(maps_path.glob("tile*.yaml"))

    if not tile_files:
        print(f"No tile*.yaml files found in {maps_dir}")
        return None, None, None

    print("=" * 70)
    print("TILE OVERLAP ANALYZER")
    print("=" * 70)
    print(f"\nDirectory: {maps_dir}")
    print(f"Found {len(tile_files)} tile files")

    tiles = {}
    resolution = None

    for yaml_file in tile_files:
        name = yaml_file.stem
        try:
            tile_num = int("".join(filter(str.isdigit, name)))
        except:
            tile_num = name

        tile_info = load_tile_yaml(yaml_file)

        if resolution is None:
            resolution = tile_info["resolution"]

        pgm_path = maps_path / tile_info["image"]
        width, height = get_image_dimensions(pgm_path)

        if width and height:
            bounds = calculate_bounds(tile_info, width, height)
            tiles[tile_num] = {
                "info": tile_info,
                "width": width,
                "height": height,
                "bounds": bounds,
                "file": yaml_file.name,
                "neighbors": [],
            }

    print("\n" + "-" * 70)
    print("TILE BOUNDS")
    print("-" * 70)

    for tid in sorted(tiles.keys()):
        t = tiles[tid]
        b = t["bounds"]
        print(f"\nTile {tid} ({t['file']}):")
        print(f"  Image: {t['width']} x {t['height']} px")
        print(f"  Resolution: {t['info']['resolution']} m/px")
        print(f"  Origin: ({t['info']['origin'][0]}, {t['info']['origin'][1]})")
        print(
            f"  Bounds: X[{b['x_min']:.2f}, {b['x_max']:.2f}] Y[{b['y_min']:.2f}, {b['y_max']:.2f}]"
        )
        print(
            f"  Size: {b['x_max'] - b['x_min']:.2f}m x {b['y_max'] - b['y_min']:.2f}m"
        )

    print("\n" + "-" * 70)
    print("OVERLAP ANALYSIS")
    print("-" * 70)

    tile_ids = sorted(tiles.keys())
    overlaps = {}

    for i in range(len(tile_ids)):
        for j in range(i + 1, len(tile_ids)):
            ti, tj = tile_ids[i], tile_ids[j]
            overlap = calculate_overlap(tiles[ti]["bounds"], tiles[tj]["bounds"])

            if overlap:
                tiles[ti]["neighbors"].append(tj)
                tiles[tj]["neighbors"].append(ti)

                center_y = round((overlap["y_min"] + overlap["y_max"]) / 2, 2)
                switch_i_to_j = [round(overlap["x_max"] - 0.5, 2), center_y]
                switch_j_to_i = [round(overlap["x_min"] + 0.5, 2), center_y]

                switch_i_to_j[0] = round(
                    min(switch_i_to_j[0], overlap["x_max"] - 0.1), 2
                )
                switch_j_to_i[0] = round(
                    max(switch_j_to_i[0], overlap["x_min"] + 0.1), 2
                )

                overlaps[(ti, tj)] = {
                    "overlap": overlap,
                    "switch_i_to_j": switch_i_to_j,
                    "switch_j_to_i": switch_j_to_i,
                }

                is_adjacent = abs(ti - tj) == 1
                status = "✓" if is_adjacent else "⚠ (non-adjacent)"

                print(f"\n  {status} Tile {ti} ↔ Tile {tj}: OVERLAP")
                print(
                    f"    Bounds: X[{overlap['x_min']:.2f}, {overlap['x_max']:.2f}] Y[{overlap['y_min']:.2f}, {overlap['y_max']:.2f}]"
                )
                print(f"    Size: {overlap['width']:.2f}m x {overlap['height']:.2f}m")
                print(f"    Area: {overlap['area']:.2f} m²")

                if overlap["width"] < 1.0 or overlap["height"] < 1.0:
                    print(f"    ⚠ WARNING: Overlap is narrow!")

                print(f"    Switch point {ti}→{tj}: {switch_i_to_j}")
                print(f"    Switch point {tj}→{ti}: {switch_j_to_i}")

    print("\n\nConnection summary:")
    for i in range(len(tile_ids) - 1):
        ti, tj = tile_ids[i], tile_ids[i + 1]
        if (ti, tj) in overlaps or (tj, ti) in overlaps:
            print(f"  ✓ Tile {ti} ↔ Tile {tj}: Connected")
        else:
            print(f"  ✗ Tile {ti} ↔ Tile {tj}: GAP - No connection!")

    for tid in tiles:
        tiles[tid]["neighbors"] = sorted(tiles[tid]["neighbors"])

    return tiles, overlaps, resolution


def generate_config_yaml(tiles, overlaps, resolution, output_path):
    """Generate the tiles_config.yaml file"""

    lines = [
        "# ============================================================================",
        "# TILES CONFIGURATION",
        "# ============================================================================",
        f"# Auto-generated by tile_overlap_analyzer.py",
        f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "#",
        "# Structure:",
        "#   settings    - Global parameters",
        "#   tiles       - Tile definitions with bounds, neighbors, and rooms",
        "#   connections - Overlap regions and switch points between tiles",
        "#",
        "# Tile state persistence:",
        "#   Current tile is stored in /tmp/current_tile.txt",
        "#   This file is read/written by tile_check_action and mission_service",
        "# ============================================================================",
        "",
        "# ----------------------------------------------------------------------------",
        "# SETTINGS",
        "# ----------------------------------------------------------------------------",
        "settings:",
        "  switch_cooldown: 0.5      # seconds - minimum time between tile switches",
        "  post_switch_delay: 0.3    # seconds - wait after switch for costmap rebuild",
        f"  resolution: {resolution}",
        "",
        "",
        "# ----------------------------------------------------------------------------",
        "# TILES",
        "# ----------------------------------------------------------------------------",
        "tiles:",
    ]

    for tid in sorted(tiles.keys()):
        t = tiles[tid]
        b = t["bounds"]
        center_x = round((b["x_min"] + b["x_max"]) / 2, 2)
        center_y = round((b["y_min"] + b["y_max"]) / 2, 2)

        lines.extend(
            [
                "",
                f"  {tid}:",
                f'    file: "tile{tid}.yaml"',
                f'    description: ""  # TODO: Add description',
                f"    bounds: [{b['x_min']}, {b['x_max']}, {b['y_min']}, {b['y_max']}]",
                f"    neighbors: {t['neighbors']}",
                f"    rooms:",
                f"      # Add rooms for tile {tid}",
                f"      # room_name:",
                f"      #   coordinates: [x, y]",
                f'      #   description: ""',
                f"      placeholder:  # Remove this placeholder and add actual rooms",
                f"        coordinates: [{center_x}, {center_y}]",
                f'        description: "Center of tile {tid}"',
            ]
        )

    lines.extend(
        [
            "",
            "",
            "# ----------------------------------------------------------------------------",
            "# CONNECTIONS",
            "# ----------------------------------------------------------------------------",
            "connections:",
        ]
    )

    for (ti, tj), data in sorted(overlaps.items()):
        ov = data["overlap"]
        lines.extend(
            [
                "",
                f"  {ti}-{tj}:",
                f"    overlap: [{ov['x_min']}, {ov['x_max']}, {ov['y_min']}, {ov['y_max']}]",
                f"    switch_points:",
                f"      {ti}_to_{tj}: {data['switch_i_to_j']}",
                f"      {tj}_to_{ti}: {data['switch_j_to_i']}",
                f'    heading: "x"',
            ]
        )

    config_content = "\n".join(lines) + "\n"

    with open(output_path, "w") as f:
        f.write(config_content)

    print(f"\n✓ Generated config: {output_path}")
    return config_content


def visualize_tiles(tiles, overlaps, output_path="tile_layout.png"):
    """Create visualization of tiles with clear overlap regions"""
    if not HAS_PIL:
        print("\nSkipping visualization (PIL not available)")
        return

    all_bounds = [t["bounds"] for t in tiles.values()]
    world_x_min = min(b["x_min"] for b in all_bounds) - 3
    world_x_max = max(b["x_max"] for b in all_bounds) + 3
    world_y_min = min(b["y_min"] for b in all_bounds) - 3
    world_y_max = max(b["y_max"] for b in all_bounds) + 3

    scale = 15
    padding = 60

    canvas_width = int((world_x_max - world_x_min) * scale) + padding * 2
    canvas_height = int((world_y_max - world_y_min) * scale) + padding * 2

    def world_to_pixel(wx, wy):
        px = int((wx - world_x_min) * scale) + padding
        py = canvas_height - int((wy - world_y_min) * scale) - padding
        return px, py

    tile_colors = {
        1: (255, 120, 120),
        2: (120, 255, 120),
        3: (120, 120, 255),
        4: (255, 255, 120),
        5: (255, 120, 255),
        6: (120, 255, 255),
    }

    img = Image.new("RGBA", (canvas_width, canvas_height), (255, 255, 255, 255))
    draw = ImageDraw.Draw(img)

    # Grid
    for x in range(int(world_x_min), int(world_x_max) + 1, 5):
        px, _ = world_to_pixel(x, 0)
        draw.line(
            [(px, padding), (px, canvas_height - padding)],
            fill=(230, 230, 230),
            width=1,
        )
        draw.text((px - 5, canvas_height - padding + 5), str(x), fill=(150, 150, 150))

    for y in range(int(world_y_min), int(world_y_max) + 1, 5):
        _, py = world_to_pixel(0, y)
        draw.line(
            [(padding, py), (canvas_width - padding, py)], fill=(230, 230, 230), width=1
        )
        draw.text((padding - 25, py - 5), str(y), fill=(150, 150, 150))

    # Tiles (outlines only)
    for tid in sorted(tiles.keys()):
        t = tiles[tid]
        b = t["bounds"]
        color = tile_colors.get(tid, (128, 128, 128))

        x1, y1 = world_to_pixel(b["x_min"], b["y_min"])
        x2, y2 = world_to_pixel(b["x_max"], b["y_max"])

        draw.rectangle([x1, y2, x2, y1], outline=color, width=4)
        draw.text((x1 + 5, y2 + 5), f"Tile {tid}", fill=color)

    # Overlaps
    overlap_layer = Image.new("RGBA", (canvas_width, canvas_height), (0, 0, 0, 0))
    overlap_draw = ImageDraw.Draw(overlap_layer)

    for (ti, tj), data in overlaps.items():
        ov = data["overlap"]
        x1, y1 = world_to_pixel(ov["x_min"], ov["y_min"])
        x2, y2 = world_to_pixel(ov["x_max"], ov["y_max"])

        overlap_draw.rectangle(
            [x1, y2, x2, y1], fill=(255, 0, 0, 80), outline=(255, 0, 0), width=2
        )

        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        overlap_draw.text((cx - 15, cy - 5), f"{ti}↔{tj}", fill=(200, 0, 0, 255))

    img = Image.alpha_composite(img, overlap_layer)
    draw = ImageDraw.Draw(img)

    # Switch points
    for (ti, tj), data in overlaps.items():
        for sp in [data["switch_i_to_j"], data["switch_j_to_i"]]:
            px, py = world_to_pixel(sp[0], sp[1])
            draw.ellipse(
                [px - 5, py - 5, px + 5, py + 5], fill=(0, 200, 0), outline=(0, 100, 0)
            )

    # Legend
    draw.text((10, 10), "Tile Layout Visualization", fill=(0, 0, 0))
    draw.text(
        (10, 28),
        f"World: X[{world_x_min:.1f}, {world_x_max:.1f}] Y[{world_y_min:.1f}, {world_y_max:.1f}]",
        fill=(100, 100, 100),
    )

    legend_y = 50
    draw.text((10, legend_y), "Legend:", fill=(0, 0, 0))
    draw.rectangle(
        [10, legend_y + 18, 25, legend_y + 28],
        fill=(255, 100, 100, 80),
        outline=(255, 0, 0),
    )
    draw.text((30, legend_y + 15), "= Overlap", fill=(100, 100, 100))
    draw.ellipse([10, legend_y + 33, 20, legend_y + 43], fill=(0, 200, 0))
    draw.text((30, legend_y + 32), "= Switch point", fill=(100, 100, 100))

    # Save
    img_rgb = Image.new("RGB", img.size, (255, 255, 255))
    img_rgb.paste(img, mask=img.split()[3] if img.mode == "RGBA" else None)
    img_rgb.save(output_path)
    print(f"✓ Visualization saved: {output_path}")


def main():
    if len(sys.argv) > 1:
        maps_dir = sys.argv[1]
    else:
        candidates = [
            ".",
            "./maps",
            "../maps",
            "/workspace/ros_ws/src/tile_manager/maps",
        ]
        maps_dir = None
        for c in candidates:
            if Path(c).exists() and list(Path(c).glob("tile*.yaml")):
                maps_dir = c
                break

        if not maps_dir:
            print("Usage: python3 tile_overlap_analyzer.py <maps_directory>")
            sys.exit(1)

    maps_path = Path(maps_dir).resolve()

    result = analyze_tiles(maps_dir)
    if not result or result[0] is None:
        print("Failed to analyze tiles")
        sys.exit(1)

    tiles, overlaps, resolution = result

    # Visualization in maps directory
    viz_path = maps_path / "raw_maps" / "tile_layout.png"
    visualize_tiles(tiles, overlaps, str(viz_path))

    # Config in ../config/ or maps directory
    config_dir = maps_path.parent / "config"
    if not config_dir.exists():
        config_dir = maps_path

    config_path = config_dir / "tiles_config.yaml"
    generate_config_yaml(tiles, overlaps, resolution, str(config_path))

    print("\n" + "=" * 70)
    print("DONE")
    print("=" * 70)
    print(f"\nTiles: {len(tiles)} | Connections: {len(overlaps)}")
    print(f"\nGenerated:")
    print(f"  • {viz_path}")
    print(f"  • {config_path}")
    print(f"\n⚠ Edit tiles_config.yaml to add room definitions!")


if __name__ == "__main__":
    main()
