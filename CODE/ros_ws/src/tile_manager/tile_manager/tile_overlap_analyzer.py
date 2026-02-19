#!/usr/bin/env python3
"""
Tile Overlap Analyzer

Reads tile yaml files and visualizes overlaps.
Run from the maps directory containing tile*.yaml files.

Usage:
    python3 tile_overlap_analyzer.py [maps_directory]
    python3 tile_overlap_analyzer.py /path/to/tile_manager/maps
"""

import yaml
import sys
import os
from pathlib import Path

try:
    from PIL import Image, ImageDraw
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Note: PIL not available, text output only")


def load_tile_yaml(filepath):
    """Load tile yaml and extract info"""
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    
    return {
        'image': data.get('image'),
        'resolution': data.get('resolution', 0.05),
        'origin': data.get('origin', [0, 0, 0]),
    }


def get_image_dimensions(pgm_path):
    """Get PGM image dimensions"""
    try:
        with open(pgm_path, 'rb') as f:
            # Read PGM header
            header = f.readline().decode().strip()
            if header not in ['P5', 'P2']:
                # Try PIL
                img = Image.open(pgm_path)
                return img.width, img.height
            
            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()
            
            # Dimensions
            parts = line.split()
            if len(parts) == 2:
                return int(parts[0]), int(parts[1])
            else:
                # Width and height might be on separate lines
                width = int(parts[0])
                height = int(f.readline().decode().strip())
                return width, height
    except Exception as e:
        print(f"  Warning: Could not read {pgm_path}: {e}")
        return None, None


def calculate_bounds(tile_info, width, height):
    """Calculate world bounds from tile info"""
    x_min = tile_info['origin'][0]
    y_min = tile_info['origin'][1]
    resolution = tile_info['resolution']
    
    x_max = x_min + width * resolution
    y_max = y_min + height * resolution
    
    return {
        'x_min': x_min,
        'x_max': x_max,
        'y_min': y_min,
        'y_max': y_max,
    }


def calculate_overlap(bounds1, bounds2):
    """Calculate overlap between two tiles"""
    x_overlap_min = max(bounds1['x_min'], bounds2['x_min'])
    x_overlap_max = min(bounds1['x_max'], bounds2['x_max'])
    y_overlap_min = max(bounds1['y_min'], bounds2['y_min'])
    y_overlap_max = min(bounds1['y_max'], bounds2['y_max'])
    
    if x_overlap_min < x_overlap_max and y_overlap_min < y_overlap_max:
        return {
            'x_min': x_overlap_min,
            'x_max': x_overlap_max,
            'y_min': y_overlap_min,
            'y_max': y_overlap_max,
            'width': x_overlap_max - x_overlap_min,
            'height': y_overlap_max - y_overlap_min,
            'area': (x_overlap_max - x_overlap_min) * (y_overlap_max - y_overlap_min),
        }
    return None


def analyze_tiles(maps_dir):
    """Analyze all tiles in directory"""
    maps_path = Path(maps_dir)
    
    # Find all tile yaml files
    tile_files = sorted(maps_path.glob('tile*.yaml'))
    
    if not tile_files:
        print(f"No tile*.yaml files found in {maps_dir}")
        return None
    
    print("=" * 70)
    print("TILE OVERLAP ANALYZER")
    print("=" * 70)
    print(f"\nDirectory: {maps_dir}")
    print(f"Found {len(tile_files)} tile files")
    
    # Load all tiles
    tiles = {}
    for yaml_file in tile_files:
        # Extract tile number from filename (tile1.yaml -> 1)
        name = yaml_file.stem
        try:
            tile_num = int(''.join(filter(str.isdigit, name)))
        except:
            tile_num = name
        
        tile_info = load_tile_yaml(yaml_file)
        
        # Get image dimensions
        pgm_path = maps_path / tile_info['image']
        width, height = get_image_dimensions(pgm_path)
        
        if width and height:
            bounds = calculate_bounds(tile_info, width, height)
            tiles[tile_num] = {
                'info': tile_info,
                'width': width,
                'height': height,
                'bounds': bounds,
                'file': yaml_file.name,
            }
    
    # Print tile info
    print("\n" + "-" * 70)
    print("TILE BOUNDS")
    print("-" * 70)
    
    for tid in sorted(tiles.keys()):
        t = tiles[tid]
        b = t['bounds']
        print(f"\nTile {tid} ({t['file']}):")
        print(f"  Image: {t['width']} x {t['height']} px")
        print(f"  Resolution: {t['info']['resolution']} m/px")
        print(f"  Origin: ({t['info']['origin'][0]}, {t['info']['origin'][1]})")
        print(f"  Bounds: X[{b['x_min']:.2f}, {b['x_max']:.2f}] Y[{b['y_min']:.2f}, {b['y_max']:.2f}]")
        print(f"  Size: {b['x_max'] - b['x_min']:.2f}m x {b['y_max'] - b['y_min']:.2f}m")
    
    # Calculate overlaps
    print("\n" + "-" * 70)
    print("OVERLAP ANALYSIS")
    print("-" * 70)
    
    tile_ids = sorted(tiles.keys())
    overlaps = {}
    
    # Check expected connections (sequential)
    expected_connections = [(tile_ids[i], tile_ids[i+1]) for i in range(len(tile_ids)-1)]
    
    print("\nExpected connections (sequential tiles):")
    for i, j in expected_connections:
        overlap = calculate_overlap(tiles[i]['bounds'], tiles[j]['bounds'])
        if overlap:
            print(f"\n  ✓ Tile {i} ↔ Tile {j}: OVERLAP")
            print(f"    Bounds: X[{overlap['x_min']:.2f}, {overlap['x_max']:.2f}] Y[{overlap['y_min']:.2f}, {overlap['y_max']:.2f}]")
            print(f"    Size: {overlap['width']:.2f}m x {overlap['height']:.2f}m")
            print(f"    Area: {overlap['area']:.2f} m²")
            
            if overlap['width'] < 1.0 or overlap['height'] < 1.0:
                print(f"    ⚠ WARNING: Overlap is narrow!")
            
            # Calculate switch points
            center_y = (overlap['y_min'] + overlap['y_max']) / 2
            switch_i_to_j = [round(overlap['x_max'] - 0.5, 2), round(center_y, 2)]
            switch_j_to_i = [round(overlap['x_min'] + 0.5, 2), round(center_y, 2)]
            print(f"    Switch point {i}→{j}: {switch_i_to_j}")
            print(f"    Switch point {j}→{i}: {switch_j_to_i}")
            
            overlaps[(i, j)] = overlap
        else:
            print(f"\n  ✗ Tile {i} ↔ Tile {j}: NO OVERLAP - GAP EXISTS!")
            b1, b2 = tiles[i]['bounds'], tiles[j]['bounds']
            
            # Calculate gap
            x_gap = max(0, b2['x_min'] - b1['x_max'])
            y_gap = max(0, max(b1['y_min'], b2['y_min']) - min(b1['y_max'], b2['y_max']))
            
            if x_gap > 0:
                print(f"    X gap: {x_gap:.2f}m (Tile {i} ends at {b1['x_max']:.2f}, Tile {j} starts at {b2['x_min']:.2f})")
            if y_gap > 0:
                print(f"    Y gap: {y_gap:.2f}m")
    
    # Check unexpected connections
    print("\n\nNon-adjacent tile overlaps (should be none):")
    unexpected_found = False
    for i in range(len(tile_ids)):
        for j in range(i + 2, len(tile_ids)):  # Skip adjacent
            ti, tj = tile_ids[i], tile_ids[j]
            overlap = calculate_overlap(tiles[ti]['bounds'], tiles[tj]['bounds'])
            if overlap:
                print(f"  ⚠ Tile {ti} ↔ Tile {tj}: UNEXPECTED OVERLAP!")
                print(f"    Size: {overlap['width']:.2f}m x {overlap['height']:.2f}m")
                unexpected_found = True
    
    if not unexpected_found:
        print("  ✓ None found (correct)")
    
    return tiles, overlaps


def visualize_tiles(tiles, overlaps, output_path='tile_layout.png'):
    """Create visualization of tiles with clear overlap regions"""
    if not HAS_PIL:
        print("\nSkipping visualization (PIL not available)")
        return
    
    # Calculate canvas bounds
    all_bounds = [t['bounds'] for t in tiles.values()]
    world_x_min = min(b['x_min'] for b in all_bounds) - 3
    world_x_max = max(b['x_max'] for b in all_bounds) + 3
    world_y_min = min(b['y_min'] for b in all_bounds) - 3
    world_y_max = max(b['y_max'] for b in all_bounds) + 3
    
    scale = 15
    padding = 60
    
    canvas_width = int((world_x_max - world_x_min) * scale) + padding * 2
    canvas_height = int((world_y_max - world_y_min) * scale) + padding * 2
    
    def world_to_pixel(wx, wy):
        px = int((wx - world_x_min) * scale) + padding
        py = canvas_height - int((wy - world_y_min) * scale) - padding
        return px, py
    
    # Colors for tiles (semi-transparent via separate drawing)
    tile_colors = {
        1: (255, 120, 120),   # Red
        2: (120, 255, 120),   # Green
        3: (120, 120, 255),   # Blue
        4: (255, 255, 120),   # Yellow
        5: (255, 120, 255),   # Magenta
        6: (120, 255, 255),   # Cyan
    }
    
    # Create main image
    img = Image.new('RGBA', (canvas_width, canvas_height), (255, 255, 255, 255))
    draw = ImageDraw.Draw(img)
    
    # Draw grid
    for x in range(int(world_x_min), int(world_x_max) + 1, 5):
        px, _ = world_to_pixel(x, 0)
        draw.line([(px, padding), (px, canvas_height - padding)], fill=(230, 230, 230), width=1)
        draw.text((px - 5, canvas_height - padding + 5), str(x), fill=(150, 150, 150))
    
    for y in range(int(world_y_min), int(world_y_max) + 1, 5):
        _, py = world_to_pixel(0, y)
        draw.line([(padding, py), (canvas_width - padding, py)], fill=(230, 230, 230), width=1)
        draw.text((padding - 25, py - 5), str(y), fill=(150, 150, 150))
    
    # Draw tiles as outlines only (no fill) so overlaps are visible
    for tid in sorted(tiles.keys()):
        t = tiles[tid]
        b = t['bounds']
        color = tile_colors.get(tid, (128, 128, 128))
        
        x1, y1 = world_to_pixel(b['x_min'], b['y_min'])
        x2, y2 = world_to_pixel(b['x_max'], b['y_max'])
        
        # Draw thick outline only
        draw.rectangle([x1, y2, x2, y1], outline=color, width=4)
        
        # Label at corner (outside the box)
        label_x = x1 + 5
        label_y = y2 + 5
        draw.text((label_x, label_y), f"Tile {tid}", fill=color)
        
        # Draw small colored square in corner for legend
        draw.rectangle([x1 + 5, y2 + 20, x1 + 15, y2 + 30], fill=color, outline=color)
    
    # Highlight overlap regions with semi-transparent red fill
    overlap_layer = Image.new('RGBA', (canvas_width, canvas_height), (0, 0, 0, 0))
    overlap_draw = ImageDraw.Draw(overlap_layer)
    
    for (i, j), overlap in overlaps.items():
        x1, y1 = world_to_pixel(overlap['x_min'], overlap['y_min'])
        x2, y2 = world_to_pixel(overlap['x_max'], overlap['y_max'])
        
        # Semi-transparent red fill for overlap
        overlap_draw.rectangle([x1, y2, x2, y1], fill=(255, 0, 0, 80), outline=(255, 0, 0), width=2)
        
        # Label overlap
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        overlap_draw.text((cx - 15, cy - 5), f"{i}↔{j}", fill=(200, 0, 0, 255))
    
    # Composite overlap layer onto main image
    img = Image.alpha_composite(img, overlap_layer)
    draw = ImageDraw.Draw(img)
    
    # Draw switch points
    for (i, j), overlap in overlaps.items():
        center_y = (overlap['y_min'] + overlap['y_max']) / 2
        
        # Switch point i→j (near j side)
        sp1_x, sp1_y = overlap['x_max'] - 0.5, center_y
        px1, py1 = world_to_pixel(sp1_x, sp1_y)
        draw.ellipse([px1-5, py1-5, px1+5, py1+5], fill=(0, 200, 0), outline=(0, 100, 0))
        
        # Switch point j→i (near i side)
        sp2_x, sp2_y = overlap['x_min'] + 0.5, center_y
        px2, py2 = world_to_pixel(sp2_x, sp2_y)
        draw.ellipse([px2-5, py2-5, px2+5, py2+5], fill=(0, 200, 0), outline=(0, 100, 0))
    
    # Title and legend
    draw.text((10, 10), "Tile Layout Visualization", fill=(0, 0, 0))
    draw.text((10, 28), f"World: X[{world_x_min:.1f}, {world_x_max:.1f}] Y[{world_y_min:.1f}, {world_y_max:.1f}]", fill=(100, 100, 100))
    
    # Legend
    legend_y = 50
    draw.text((10, legend_y), "Legend:", fill=(0, 0, 0))
    draw.rectangle([10, legend_y + 18, 25, legend_y + 28], fill=(255, 0, 0, 80), outline=(255, 0, 0))
    draw.text((30, legend_y + 15), "= Overlap region", fill=(100, 100, 100))
    draw.ellipse([10, legend_y + 33, 20, legend_y + 43], fill=(0, 200, 0))
    draw.text((30, legend_y + 32), "= Switch point", fill=(100, 100, 100))
    
    # Save as PNG (convert RGBA to RGB for compatibility)
    img_rgb = Image.new('RGB', img.size, (255, 255, 255))
    img_rgb.paste(img, mask=img.split()[3] if img.mode == 'RGBA' else None)
    img_rgb.save(output_path)
    print(f"\nVisualization saved to: {output_path}")


def main():
    if len(sys.argv) > 1:
        maps_dir = sys.argv[1]
    else:
        # Try common locations
        candidates = [
            '.',
            './maps',
            '../maps',
            '/workspace/ros_ws/src/tile_manager/maps',
        ]
        maps_dir = None
        for c in candidates:
            if Path(c).exists() and list(Path(c).glob('tile*.yaml')):
                maps_dir = c
                break
        
        if not maps_dir:
            print("Usage: python3 tile_overlap_analyzer.py <maps_directory>")
            sys.exit(1)
    
    result = analyze_tiles(maps_dir)
    if result:
        tiles, overlaps = result
        visualize_tiles(tiles, overlaps, 'tile_layout.png')
        
        # Print config snippet
        print("\n" + "=" * 70)
        print("SUGGESTED tiles_config.yaml SNIPPET")
        print("=" * 70)
        print("\nconnections:")
        
        for (i, j), overlap in overlaps.items():
            center_y = (overlap['y_min'] + overlap['y_max']) / 2
            print(f"  {i}-{j}:")
            print(f"    overlap: [{overlap['x_min']:.2f}, {overlap['x_max']:.2f}, {overlap['y_min']:.2f}, {overlap['y_max']:.2f}]")
            print(f"    switch_points:")
            print(f"      {i}_to_{j}: [{overlap['x_max'] - 0.5:.2f}, {center_y:.2f}]")
            print(f"      {j}_to_{i}: [{overlap['x_min'] + 0.5:.2f}, {center_y:.2f}]")
            print(f"    heading: x")


if __name__ == '__main__':
    main()
