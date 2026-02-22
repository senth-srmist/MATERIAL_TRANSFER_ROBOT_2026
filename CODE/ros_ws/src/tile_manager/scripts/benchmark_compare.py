#!/usr/bin/env python3
"""
Benchmark Comparison Visualizer

Compares whole map vs tile-based navigation benchmarks.

Usage:
    python3 benchmark_compare.py whole_results.json tile_results.json
    python3 benchmark_compare.py whole_results.json tile_results.json --output comparison.png
"""

import argparse
import json
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not installed. Install with: pip install matplotlib")


def load_results(filepath):
    """Load benchmark results from JSON."""
    with open(filepath, 'r') as f:
        return json.load(f)


def get_nav2_memory(data):
    """Calculate total Nav2 process memory from samples."""
    nav2_procs = ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'lifecycle_manager']
    mem_totals = []
    
    for sample in data.get('samples', []):
        procs = sample.get('processes', {})
        total = sum(procs.get(p, {}).get('memory_mb', 0) for p in nav2_procs)
        if total > 0:
            mem_totals.append(total)
    
    if not mem_totals:
        return {'mean': 0, 'max': 0, 'min': 0}
    
    return {
        'mean': sum(mem_totals) / len(mem_totals),
        'max': max(mem_totals),
        'min': min(mem_totals),
    }


def get_process_memory(data, process_name):
    """Get memory stats for a specific process."""
    mem_values = []
    for sample in data.get('samples', []):
        procs = sample.get('processes', {})
        mem = procs.get(process_name, {}).get('memory_mb', 0)
        if mem > 0:
            mem_values.append(mem)
    
    if not mem_values:
        return {'mean': 0, 'max': 0, 'min': 0}
    
    return {
        'mean': sum(mem_values) / len(mem_values),
        'max': max(mem_values),
        'min': min(mem_values),
    }


def print_comparison(whole, tiles):
    """Print text comparison."""
    print("\n" + "=" * 70)
    print("BENCHMARK COMPARISON: WHOLE MAP vs TILE-BASED")
    print("=" * 70)
    
    whole_summary = whole.get('summary', {})
    tiles_summary = tiles.get('summary', {})
    
    # Calculate Nav2-specific memory
    whole_nav2_mem = get_nav2_memory(whole)
    tiles_nav2_mem = get_nav2_memory(tiles)
    
    whole_planner_mem = get_process_memory(whole, 'planner_server')
    tiles_planner_mem = get_process_memory(tiles, 'planner_server')
    
    print(f"\n{'Metric':<30} {'Whole Map':>15} {'Tile-Based':>15} {'Difference':>15}")
    print("-" * 75)
    
    # Costmap
    whole_costmap = whole_summary.get('costmap_cells', {}).get('mean', 0)
    tiles_costmap = tiles_summary.get('costmap_cells', {}).get('mean', 0)
    costmap_diff = ((tiles_costmap - whole_costmap) / whole_costmap * 100) if whole_costmap > 0 else 0
    print(f"{'Costmap Mean (cells)':<30} {whole_costmap:>15,.0f} {tiles_costmap:>15,.0f} {costmap_diff:>+14.1f}%")
    
    whole_costmap_max = whole_summary.get('costmap_cells', {}).get('max', 0)
    tiles_costmap_max = tiles_summary.get('costmap_cells', {}).get('max', 0)
    print(f"{'Costmap Max (cells)':<30} {whole_costmap_max:>15,.0f} {tiles_costmap_max:>15,.0f}")
    
    print()
    
    # Nav2 Process Memory
    nav2_mem_diff = ((tiles_nav2_mem['mean'] - whole_nav2_mem['mean']) / whole_nav2_mem['mean'] * 100) if whole_nav2_mem['mean'] > 0 else 0
    print(f"{'Nav2 Total Memory (MB)':<30} {whole_nav2_mem['mean']:>15.1f} {tiles_nav2_mem['mean']:>15.1f} {nav2_mem_diff:>+14.1f}%")
    
    planner_diff = ((tiles_planner_mem['mean'] - whole_planner_mem['mean']) / whole_planner_mem['mean'] * 100) if whole_planner_mem['mean'] > 0 else 0
    print(f"{'Planner Server Memory (MB)':<30} {whole_planner_mem['mean']:>15.1f} {tiles_planner_mem['mean']:>15.1f} {planner_diff:>+14.1f}%")
    
    print()
    
    # CPU
    whole_cpu = whole_summary.get('cpu', {}).get('mean', 0)
    tiles_cpu = tiles_summary.get('cpu', {}).get('mean', 0)
    cpu_diff = ((tiles_cpu - whole_cpu) / whole_cpu * 100) if whole_cpu > 0 else 0
    print(f"{'CPU Mean (%)':<30} {whole_cpu:>15.1f} {tiles_cpu:>15.1f} {cpu_diff:>+14.1f}%")
    
    whole_cpu_max = whole_summary.get('cpu', {}).get('max', 0)
    tiles_cpu_max = tiles_summary.get('cpu', {}).get('max', 0)
    print(f"{'CPU Max (%)':<30} {whole_cpu_max:>15.1f} {tiles_cpu_max:>15.1f}")
    
    print()
    
    # Tile switches & samples
    tile_switches = tiles_summary.get('tile_switches', 0)
    print(f"{'Tile Switches':<30} {'N/A':>15} {tile_switches:>15}")
    
    whole_samples = whole.get('metadata', {}).get('sample_count', 0)
    tiles_samples = tiles.get('metadata', {}).get('sample_count', 0)
    print(f"{'Sample Count':<30} {whole_samples:>15} {tiles_samples:>15}")
    
    print("\n" + "=" * 70)
    
    # Key findings
    print("\nKEY FINDINGS:")
    
    if costmap_diff < 0:
        print(f"  ✓ Tile-based uses {abs(costmap_diff):.1f}% smaller costmap ({tiles_costmap:,.0f} vs {whole_costmap:,.0f} cells)")
    
    if planner_diff < 0:
        print(f"  ✓ Planner uses {abs(planner_diff):.1f}% less memory ({tiles_planner_mem['mean']:.1f} vs {whole_planner_mem['mean']:.1f} MB)")
    
    if nav2_mem_diff < 0:
        print(f"  ✓ Nav2 stack uses {abs(nav2_mem_diff):.1f}% less memory total")
    else:
        print(f"  ~ Nav2 stack uses {nav2_mem_diff:.1f}% more memory (tile switch overhead)")
    
    if cpu_diff > 5:
        print(f"  ~ CPU {cpu_diff:.1f}% higher due to tile switch overhead (spikes during transitions)")
    elif cpu_diff < -5:
        print(f"  ✓ CPU {abs(cpu_diff):.1f}% lower with tiles")
    
    print(f"  • {tile_switches} tile switches during navigation")


def plot_comparison(whole, tiles, output_path=None):
    """Generate comparison plots."""
    if not HAS_MATPLOTLIB:
        print("Cannot generate plots without matplotlib")
        return
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('Navigation Benchmark: Whole Map vs Tile-Based', fontsize=14, fontweight='bold')
    
    whole_samples = whole.get('samples', [])
    tiles_samples = tiles.get('samples', [])
    
    # Extract time series
    whole_times = [s['timestamp'] for s in whole_samples]
    tiles_times = [s['timestamp'] for s in tiles_samples]
    
    whole_cpu = [s.get('system', {}).get('cpu_percent', 0) for s in whole_samples]
    tiles_cpu = [s.get('system', {}).get('cpu_percent', 0) for s in tiles_samples]
    
    # Nav2 process memory (sum of all Nav2 processes)
    nav2_procs = ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'lifecycle_manager']
    whole_nav2_mem = [sum(s.get('processes', {}).get(p, {}).get('memory_mb', 0) for p in nav2_procs) for s in whole_samples]
    tiles_nav2_mem = [sum(s.get('processes', {}).get(p, {}).get('memory_mb', 0) for p in nav2_procs) for s in tiles_samples]
    
    # Map server memory
    whole_map_mem = [s.get('processes', {}).get('map_server', {}).get('memory_mb', 0) for s in whole_samples]
    tiles_map_mem = [s.get('processes', {}).get('map_server', {}).get('memory_mb', 0) for s in tiles_samples]
    
    # Planner server memory
    whole_planner_mem = [s.get('processes', {}).get('planner_server', {}).get('memory_mb', 0) for s in whole_samples]
    tiles_planner_mem = [s.get('processes', {}).get('planner_server', {}).get('memory_mb', 0) for s in tiles_samples]
    
    whole_costmap = [s.get('costmap_cells', 0) for s in whole_samples]
    tiles_costmap = [s.get('costmap_cells', 0) for s in tiles_samples]
    
    tile_switch_times = [ts['timestamp'] for ts in tiles.get('tile_switches', [])]
    
    colors = {'whole': '#E74C3C', 'tiles': '#27AE60'}
    
    # Plot 1: CPU Usage
    ax1 = axes[0, 0]
    ax1.plot(whole_times, whole_cpu, color=colors['whole'], label='Whole Map', alpha=0.8)
    ax1.plot(tiles_times, tiles_cpu, color=colors['tiles'], label='Tile-Based', alpha=0.8)
    for ts in tile_switch_times:
        ax1.axvline(x=ts, color='orange', linestyle='--', alpha=0.5, linewidth=1)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('CPU Usage (%)')
    ax1.set_title('CPU Usage Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Nav2 Total Memory
    ax2 = axes[0, 1]
    ax2.plot(whole_times, whole_nav2_mem, color=colors['whole'], label='Whole Map', alpha=0.8)
    ax2.plot(tiles_times, tiles_nav2_mem, color=colors['tiles'], label='Tile-Based', alpha=0.8)
    for ts in tile_switch_times:
        ax2.axvline(x=ts, color='orange', linestyle='--', alpha=0.5, linewidth=1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Memory (MB)')
    ax2.set_title('Nav2 Total Memory Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Map Server Memory
    ax3 = axes[1, 0]
    ax3.plot(whole_times, whole_map_mem, color=colors['whole'], label='Whole Map', alpha=0.8)
    ax3.plot(tiles_times, tiles_map_mem, color=colors['tiles'], label='Tile-Based', alpha=0.8)
    for ts in tile_switch_times:
        ax3.axvline(x=ts, color='orange', linestyle='--', alpha=0.5, linewidth=1)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Memory (MB)')
    ax3.set_title('Map Server Memory Over Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Planner Server Memory
    ax4 = axes[1, 1]
    ax4.plot(whole_times, whole_planner_mem, color=colors['whole'], label='Whole Map', alpha=0.8)
    ax4.plot(tiles_times, tiles_planner_mem, color=colors['tiles'], label='Tile-Based', alpha=0.8)
    for ts in tile_switch_times:
        ax4.axvline(x=ts, color='orange', linestyle='--', alpha=0.5, linewidth=1)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Memory (MB)')
    ax4.set_title('Planner Server Memory Over Time')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Costmap Size
    ax5 = axes[2, 0]
    ax5.plot(whole_times, whole_costmap, color=colors['whole'], label='Whole Map', alpha=0.8)
    ax5.plot(tiles_times, tiles_costmap, color=colors['tiles'], label='Tile-Based', alpha=0.8)
    for ts in tile_switch_times:
        ax5.axvline(x=ts, color='orange', linestyle='--', alpha=0.5, linewidth=1)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Costmap Cells')
    ax5.set_title('Costmap Size Over Time')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.ticklabel_format(style='scientific', axis='y', scilimits=(0, 0))
    
    # Plot 6: Summary Bar Chart
    ax6 = axes[2, 1]
    
    # Calculate memory stats
    whole_nav2_mem_stats = get_nav2_memory(whole)
    tiles_nav2_mem_stats = get_nav2_memory(tiles)
    whole_planner_stats = get_process_memory(whole, 'planner_server')
    tiles_planner_stats = get_process_memory(tiles, 'planner_server')
    whole_map_stats = get_process_memory(whole, 'map_server')
    tiles_map_stats = get_process_memory(tiles, 'map_server')
    
    whole_summary = whole.get('summary', {})
    tiles_summary = tiles.get('summary', {})
    
    metrics = ['Map Server\n(MB)', 'Planner\n(MB)', 'Nav2 Total\n(MB)', 'Costmap\n(cells / 10K)']
    whole_vals = [
        whole_map_stats['mean'],
        whole_planner_stats['mean'],
        whole_nav2_mem_stats['mean'],
        whole_summary.get('costmap_cells', {}).get('mean', 0) / 10000,
    ]
    tiles_vals = [
        tiles_map_stats['mean'],
        tiles_planner_stats['mean'],
        tiles_nav2_mem_stats['mean'],
        tiles_summary.get('costmap_cells', {}).get('mean', 0) / 10000,
    ]
    
    x = range(len(metrics))
    width = 0.35
    
    bars1 = ax6.bar([i - width/2 for i in x], whole_vals, width, label='Whole Map', color=colors['whole'])
    bars2 = ax6.bar([i + width/2 for i in x], tiles_vals, width, label='Tile-Based', color=colors['tiles'])
    
    ax6.set_ylabel('Value')
    ax6.set_title('Summary Comparison')
    ax6.set_xticks(x)
    ax6.set_xticklabels(metrics)
    ax6.legend()
    ax6.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, val in zip(bars1, whole_vals):
        ax6.annotate(f'{val:.1f}', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                     ha='center', va='bottom', fontsize=8)
    for bar, val in zip(bars2, tiles_vals):
        ax6.annotate(f'{val:.1f}', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                     ha='center', va='bottom', fontsize=8)
    
    # Add legend for tile switches
    if tile_switch_times:
        orange_line = mpatches.Patch(color='orange', alpha=0.5, label=f'Tile Switches ({len(tile_switch_times)})')
        fig.legend(handles=[orange_line], loc='lower center', ncol=1, fontsize=10)
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.06)
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Compare navigation benchmarks')
    parser.add_argument('whole', help='Whole map benchmark JSON file')
    parser.add_argument('tiles', help='Tile-based benchmark JSON file')
    parser.add_argument('--output', '-o', help='Output image path (optional)')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')
    args = parser.parse_args()
    
    # Load results
    try:
        whole = load_results(args.whole)
        tiles = load_results(args.tiles)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    # Print comparison
    print_comparison(whole, tiles)
    
    # Plot comparison
    if not args.no_plot and HAS_MATPLOTLIB:
        plot_comparison(whole, tiles, args.output)


if __name__ == '__main__':
    main()
