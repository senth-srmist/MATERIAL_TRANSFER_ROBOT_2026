#!/usr/bin/env python3
"""
Benchmark Visualization

Generates comparison plots from benchmark CSV files.

Usage:
    python3 benchmark_visualizer.py --tiled /path/to/tiled.csv --whole /path/to/whole.csv
    python3 benchmark_visualizer.py --tiled /path/to/tiled.csv --whole /path/to/whole.csv --switches /path/to/switches.csv
    python3 benchmark_visualizer.py --tiled /path/to/tiled.csv --whole /path/to/whole.csv -o /path/to/output/

Outputs:
    - memory_comparison.png
    - cpu_comparison.png
    - costmap_comparison.png
    - disk_io_comparison.png
    - summary_table.png
    - full_report.png (combined)
"""

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime


class BenchmarkVisualizer:
    def __init__(self, tiled_csv, whole_csv, switches_csv=None, output_dir=None):
        self.tiled = pd.read_csv(tiled_csv)
        self.whole = pd.read_csv(whole_csv)
        self.switches = pd.read_csv(switches_csv) if switches_csv else None
        
        self.output_dir = output_dir or os.path.dirname(tiled_csv)
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Style settings
        plt.style.use('seaborn-v0_8-whitegrid')
        self.colors = {
            'tiled': '#2196F3',  # Blue
            'whole': '#F44336',  # Red
            'switch': '#4CAF50',  # Green
        }
        
    def _add_switch_lines(self, ax):
        """Add vertical lines for tile switches"""
        if self.switches is not None:
            for idx, row in self.switches.iterrows():
                label = 'Tile Switch' if idx == 0 else None
                ax.axvline(x=row['timestamp'], color=self.colors['switch'], 
                          linestyle='--', alpha=0.7, label=label)

    def plot_memory(self, save=True):
        """Plot memory comparison"""
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Memory Usage Comparison', fontsize=14, fontweight='bold')
        
        # Map server memory over time
        ax1 = axes[0]
        ax1.plot(self.tiled['timestamp'], self.tiled['map_server_mem_mb'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax1.plot(self.whole['timestamp'], self.whole['map_server_mem_mb'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Memory (MB)')
        ax1.set_title('Map Server Memory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Total memory over time
        ax2 = axes[1]
        ax2.plot(self.tiled['timestamp'], self.tiled['total_mem_mb'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax2.plot(self.whole['timestamp'], self.whole['total_mem_mb'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Memory (MB)')
        ax2.set_title('Total Memory (map_server + tile_switcher)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'memory_comparison.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def plot_cpu(self, save=True):
        """Plot CPU comparison"""
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('CPU Usage Comparison', fontsize=14, fontweight='bold')
        
        # Smoothing window
        window = 10
        
        # Map server CPU
        ax1 = axes[0]
        tiled_smooth = self.tiled['map_server_cpu'].rolling(window=window, center=True).mean()
        whole_smooth = self.whole['map_server_cpu'].rolling(window=window, center=True).mean()
        ax1.plot(self.tiled['timestamp'], tiled_smooth, 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax1.plot(self.whole['timestamp'], whole_smooth, 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('CPU (%)')
        ax1.set_title(f'Map Server CPU (smoothed, window={window})')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Total CPU
        ax2 = axes[1]
        tiled_smooth = self.tiled['total_cpu'].rolling(window=window, center=True).mean()
        whole_smooth = self.whole['total_cpu'].rolling(window=window, center=True).mean()
        ax2.plot(self.tiled['timestamp'], tiled_smooth, 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax2.plot(self.whole['timestamp'], whole_smooth, 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('CPU (%)')
        ax2.set_title(f'Total CPU (smoothed, window={window})')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'cpu_comparison.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def plot_costmap(self, save=True):
        """Plot costmap update rate comparison"""
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Costmap Update Comparison', fontsize=14, fontweight='bold')
        
        # Update rate over time
        ax1 = axes[0]
        ax1.plot(self.tiled['timestamp'], self.tiled['costmap_update_rate_hz'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax1.plot(self.whole['timestamp'], self.whole['costmap_update_rate_hz'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Update Rate (Hz)')
        ax1.set_title('Costmap Update Rate')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Cumulative updates
        ax2 = axes[1]
        ax2.plot(self.tiled['timestamp'], self.tiled['costmap_update_count'], 
                label='Tiled (updates)', color=self.colors['tiled'], linewidth=1.5)
        ax2.plot(self.whole['timestamp'], self.whole['costmap_update_count'], 
                label='Whole (updates)', color=self.colors['whole'], linewidth=1.5)
        ax2.plot(self.tiled['timestamp'], self.tiled['costmap_full_count'], 
                label='Tiled (full)', color=self.colors['tiled'], linestyle='--', linewidth=1.5)
        ax2.plot(self.whole['timestamp'], self.whole['costmap_full_count'], 
                label='Whole (full)', color=self.colors['whole'], linestyle='--', linewidth=1.5)
        self._add_switch_lines(ax2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Count')
        ax2.set_title('Cumulative Costmap Updates')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'costmap_comparison.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def plot_disk_io(self, save=True):
        """Plot disk I/O comparison"""
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Disk I/O Comparison', fontsize=14, fontweight='bold')
        
        # Cumulative read
        ax1 = axes[0]
        ax1.plot(self.tiled['timestamp'], self.tiled['total_disk_read_mb'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax1.plot(self.whole['timestamp'], self.whole['total_disk_read_mb'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Cumulative Read (MB)')
        ax1.set_title('Disk Read')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Cumulative write
        ax2 = axes[1]
        ax2.plot(self.tiled['timestamp'], self.tiled['total_disk_write_mb'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax2.plot(self.whole['timestamp'], self.whole['total_disk_write_mb'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Cumulative Write (MB)')
        ax2.set_title('Disk Write')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'disk_io_comparison.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def plot_summary_bars(self, save=True):
        """Plot summary bar charts"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Summary Comparison', fontsize=14, fontweight='bold')
        
        # Memory comparison
        ax1 = axes[0, 0]
        categories = ['Map Server\n(Mean)', 'Map Server\n(Max)', 'Total\n(Mean)', 'Total\n(Max)']
        tiled_vals = [
            self.tiled['map_server_mem_mb'].mean(),
            self.tiled['map_server_mem_mb'].max(),
            self.tiled['total_mem_mb'].mean(),
            self.tiled['total_mem_mb'].max(),
        ]
        whole_vals = [
            self.whole['map_server_mem_mb'].mean(),
            self.whole['map_server_mem_mb'].max(),
            self.whole['total_mem_mb'].mean(),
            self.whole['total_mem_mb'].max(),
        ]
        x = np.arange(len(categories))
        width = 0.35
        bars1 = ax1.bar(x - width/2, tiled_vals, width, label='Tiled', color=self.colors['tiled'])
        bars2 = ax1.bar(x + width/2, whole_vals, width, label='Whole', color=self.colors['whole'])
        ax1.set_ylabel('Memory (MB)')
        ax1.set_title('Memory Usage')
        ax1.set_xticks(x)
        ax1.set_xticklabels(categories)
        ax1.legend()
        ax1.grid(True, alpha=0.3, axis='y')
        for bar in bars1 + bars2:
            height = bar.get_height()
            ax1.annotate(f'{height:.1f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8)
        
        # CPU comparison
        ax2 = axes[0, 1]
        categories = ['Map Server\n(Mean)', 'Map Server\n(Max)', 'Total\n(Mean)', 'Total\n(Max)']
        tiled_vals = [
            self.tiled['map_server_cpu'].mean(),
            self.tiled['map_server_cpu'].max(),
            self.tiled['total_cpu'].mean(),
            self.tiled['total_cpu'].max(),
        ]
        whole_vals = [
            self.whole['map_server_cpu'].mean(),
            self.whole['map_server_cpu'].max(),
            self.whole['total_cpu'].mean(),
            self.whole['total_cpu'].max(),
        ]
        x = np.arange(len(categories))
        bars1 = ax2.bar(x - width/2, tiled_vals, width, label='Tiled', color=self.colors['tiled'])
        bars2 = ax2.bar(x + width/2, whole_vals, width, label='Whole', color=self.colors['whole'])
        ax2.set_ylabel('CPU (%)')
        ax2.set_title('CPU Usage')
        ax2.set_xticks(x)
        ax2.set_xticklabels(categories)
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')
        for bar in bars1 + bars2:
            height = bar.get_height()
            ax2.annotate(f'{height:.1f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8)
        
        # Costmap comparison
        ax3 = axes[1, 0]
        categories = ['Update Rate\n(Mean Hz)', 'Total\nUpdates', 'Full\nPublishes']
        tiled_vals = [
            self.tiled['costmap_update_rate_hz'].mean(),
            self.tiled['costmap_update_count'].iloc[-1] / 100,  # Scale down for visibility
            self.tiled['costmap_full_count'].iloc[-1],
        ]
        whole_vals = [
            self.whole['costmap_update_rate_hz'].mean(),
            self.whole['costmap_update_count'].iloc[-1] / 100,
            self.whole['costmap_full_count'].iloc[-1],
        ]
        x = np.arange(len(categories))
        bars1 = ax3.bar(x - width/2, tiled_vals, width, label='Tiled', color=self.colors['tiled'])
        bars2 = ax3.bar(x + width/2, whole_vals, width, label='Whole', color=self.colors['whole'])
        ax3.set_ylabel('Value')
        ax3.set_title('Costmap Metrics (updates ÷100)')
        ax3.set_xticks(x)
        ax3.set_xticklabels(categories)
        ax3.legend()
        ax3.grid(True, alpha=0.3, axis='y')
        
        # Disk I/O comparison
        ax4 = axes[1, 1]
        categories = ['Total Read\n(MB)', 'Total Write\n(MB)']
        tiled_vals = [
            self.tiled['total_disk_read_mb'].iloc[-1],
            self.tiled['total_disk_write_mb'].iloc[-1],
        ]
        whole_vals = [
            self.whole['total_disk_read_mb'].iloc[-1],
            self.whole['total_disk_write_mb'].iloc[-1],
        ]
        x = np.arange(len(categories))
        bars1 = ax4.bar(x - width/2, tiled_vals, width, label='Tiled', color=self.colors['tiled'])
        bars2 = ax4.bar(x + width/2, whole_vals, width, label='Whole', color=self.colors['whole'])
        ax4.set_ylabel('Size (MB)')
        ax4.set_title('Disk I/O')
        ax4.set_xticks(x)
        ax4.set_xticklabels(categories)
        ax4.legend()
        ax4.grid(True, alpha=0.3, axis='y')
        for bar in bars1 + bars2:
            height = bar.get_height()
            ax4.annotate(f'{height:.1f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=8)
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'summary_bars.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def plot_tile_switches(self, save=True):
        """Plot tile switch times"""
        if self.switches is None or len(self.switches) == 0:
            print("No tile switch data available")
            return None
            
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle('Tile Switch Analysis', fontsize=14, fontweight='bold')
        
        # Switch times bar chart
        ax1 = axes[0]
        x = np.arange(len(self.switches))
        bars = ax1.bar(x, self.switches['switch_time_ms'], color=self.colors['switch'])
        ax1.set_xlabel('Switch #')
        ax1.set_ylabel('Time (ms)')
        ax1.set_title('Individual Switch Times')
        ax1.set_xticks(x)
        ax1.set_xticklabels([f"→ Tile {int(t)}" for t in self.switches['tile']])
        ax1.grid(True, alpha=0.3, axis='y')
        for bar in bars:
            height = bar.get_height()
            ax1.annotate(f'{height:.1f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom')
        
        # Statistics
        ax2 = axes[1]
        ax2.axis('off')
        switch_times = self.switches['switch_time_ms']
        stats_text = f"""
        TILE SWITCH STATISTICS
        ─────────────────────────
        
        Count:    {len(switch_times)}
        
        Min:      {switch_times.min():.2f} ms
        Max:      {switch_times.max():.2f} ms
        Mean:     {switch_times.mean():.2f} ms
        Std Dev:  {switch_times.std():.2f} ms
        
        ─────────────────────────
        
        Switches:
        """
        for idx, row in self.switches.iterrows():
            stats_text += f"\n          t={row['timestamp']:.1f}s → Tile {int(row['tile'])}: {row['switch_time_ms']:.1f} ms"
        
        ax2.text(0.1, 0.9, stats_text, transform=ax2.transAxes, fontsize=11,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        if save:
            path = os.path.join(self.output_dir, 'tile_switches.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def generate_summary_table(self, save=True):
        """Generate summary statistics table"""
        
        def calc_diff(t, w):
            diff = t - w
            pct = (diff / w) * 100 if w != 0 else 0
            return diff, pct
        
        # Collect statistics
        stats = []
        
        # Memory
        t, w = self.tiled['map_server_mem_mb'].mean(), self.whole['map_server_mem_mb'].mean()
        diff, pct = calc_diff(t, w)
        stats.append(('Map Server Memory (Mean)', f'{t:.2f} MB', f'{w:.2f} MB', f'{diff:+.2f} MB ({pct:+.1f}%)'))
        
        t, w = self.tiled['map_server_mem_mb'].max(), self.whole['map_server_mem_mb'].max()
        diff, pct = calc_diff(t, w)
        stats.append(('Map Server Memory (Max)', f'{t:.2f} MB', f'{w:.2f} MB', f'{diff:+.2f} MB ({pct:+.1f}%)'))
        
        t, w = self.tiled['total_mem_mb'].mean(), self.whole['total_mem_mb'].mean()
        diff, pct = calc_diff(t, w)
        stats.append(('Total Memory (Mean)', f'{t:.2f} MB', f'{w:.2f} MB', f'{diff:+.2f} MB ({pct:+.1f}%)'))
        
        # CPU
        t, w = self.tiled['map_server_cpu'].mean(), self.whole['map_server_cpu'].mean()
        diff, pct = calc_diff(t, w)
        stats.append(('Map Server CPU (Mean)', f'{t:.2f} %', f'{w:.2f} %', f'{diff:+.2f} %'))
        
        t, w = self.tiled['total_cpu'].mean(), self.whole['total_cpu'].mean()
        diff, pct = calc_diff(t, w)
        stats.append(('Total CPU (Mean)', f'{t:.2f} %', f'{w:.2f} %', f'{diff:+.2f} %'))
        
        # Costmap
        t, w = self.tiled['costmap_update_rate_hz'].mean(), self.whole['costmap_update_rate_hz'].mean()
        diff, pct = calc_diff(t, w)
        stats.append(('Costmap Update Rate', f'{t:.2f} Hz', f'{w:.2f} Hz', f'{diff:+.2f} Hz'))
        
        t, w = self.tiled['costmap_full_count'].iloc[-1], self.whole['costmap_full_count'].iloc[-1]
        diff, pct = calc_diff(t, w)
        stats.append(('Costmap Full Publishes', f'{int(t)}', f'{int(w)}', f'{int(diff):+d}'))
        
        # Disk I/O
        t, w = self.tiled['total_disk_read_mb'].iloc[-1], self.whole['total_disk_read_mb'].iloc[-1]
        diff, pct = calc_diff(t, w)
        stats.append(('Disk Read (Total)', f'{t:.2f} MB', f'{w:.2f} MB', f'{diff:+.2f} MB'))
        
        t, w = self.tiled['total_disk_write_mb'].iloc[-1], self.whole['total_disk_write_mb'].iloc[-1]
        diff, pct = calc_diff(t, w)
        stats.append(('Disk Write (Total)', f'{t:.2f} MB', f'{w:.2f} MB', f'{diff:+.2f} MB'))
        
        # Tile switches
        if self.switches is not None and len(self.switches) > 0:
            stats.append(('Tile Switches', f'{len(self.switches)}', 'N/A', '—'))
            stats.append(('Avg Switch Time', f'{self.switches["switch_time_ms"].mean():.2f} ms', 'N/A', '—'))
        
        # Create table figure
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.axis('off')
        
        table_data = [['Metric', 'Tiled', 'Whole', 'Difference']] + stats
        table = ax.table(cellText=table_data, loc='center', cellLoc='left',
                        colWidths=[0.35, 0.2, 0.2, 0.25])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 1.8)
        
        # Style header
        for i in range(4):
            table[(0, i)].set_text_props(fontweight='bold')
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(color='white')
        
        # Alternate row colors
        for i in range(1, len(table_data)):
            for j in range(4):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor('#f0f0f0')
        
        plt.title('Benchmark Comparison Summary', fontsize=14, fontweight='bold', pad=20)
        
        if save:
            path = os.path.join(self.output_dir, 'summary_table.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig, stats

    def generate_full_report(self, save=True):
        """Generate complete report with all plots"""
        fig = plt.figure(figsize=(16, 20))
        
        # Create grid
        gs = fig.add_gridspec(4, 2, hspace=0.3, wspace=0.2)
        
        # Title
        fig.suptitle('Tile Map vs Whole Map: Benchmark Report', fontsize=16, fontweight='bold', y=0.98)
        
        # 1. Memory over time
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(self.tiled['timestamp'], self.tiled['map_server_mem_mb'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax1.plot(self.whole['timestamp'], self.whole['map_server_mem_mb'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Memory (MB)')
        ax1.set_title('Map Server Memory')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # 2. CPU over time
        ax2 = fig.add_subplot(gs[0, 1])
        window = 10
        ax2.plot(self.tiled['timestamp'], self.tiled['total_cpu'].rolling(window=window).mean(), 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax2.plot(self.whole['timestamp'], self.whole['total_cpu'].rolling(window=window).mean(), 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('CPU (%)')
        ax2.set_title('Total CPU Usage')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        # 3. Costmap update rate
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(self.tiled['timestamp'], self.tiled['costmap_update_rate_hz'], 
                label='Tiled', color=self.colors['tiled'], linewidth=1.5)
        ax3.plot(self.whole['timestamp'], self.whole['costmap_update_rate_hz'], 
                label='Whole', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax3)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Rate (Hz)')
        ax3.set_title('Costmap Update Rate')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        # 4. Disk I/O
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(self.tiled['timestamp'], self.tiled['total_disk_read_mb'], 
                label='Tiled Read', color=self.colors['tiled'], linewidth=1.5)
        ax4.plot(self.whole['timestamp'], self.whole['total_disk_read_mb'], 
                label='Whole Read', color=self.colors['whole'], linewidth=1.5)
        self._add_switch_lines(ax4)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Cumulative (MB)')
        ax4.set_title('Disk Read')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        
        # 5. Summary bars - Memory
        ax5 = fig.add_subplot(gs[2, 0])
        categories = ['Map Server\nMean', 'Map Server\nMax', 'Total\nMean']
        tiled_vals = [self.tiled['map_server_mem_mb'].mean(), 
                     self.tiled['map_server_mem_mb'].max(),
                     self.tiled['total_mem_mb'].mean()]
        whole_vals = [self.whole['map_server_mem_mb'].mean(),
                     self.whole['map_server_mem_mb'].max(),
                     self.whole['total_mem_mb'].mean()]
        x = np.arange(len(categories))
        width = 0.35
        ax5.bar(x - width/2, tiled_vals, width, label='Tiled', color=self.colors['tiled'])
        ax5.bar(x + width/2, whole_vals, width, label='Whole', color=self.colors['whole'])
        ax5.set_ylabel('Memory (MB)')
        ax5.set_title('Memory Summary')
        ax5.set_xticks(x)
        ax5.set_xticklabels(categories)
        ax5.legend()
        ax5.grid(True, alpha=0.3, axis='y')
        
        # 6. Tile switch times (if available)
        ax6 = fig.add_subplot(gs[2, 1])
        if self.switches is not None and len(self.switches) > 0:
            x = np.arange(len(self.switches))
            ax6.bar(x, self.switches['switch_time_ms'], color=self.colors['switch'])
            ax6.set_xlabel('Switch #')
            ax6.set_ylabel('Time (ms)')
            ax6.set_title('Tile Switch Times')
            ax6.set_xticks(x)
            ax6.set_xticklabels([f"→T{int(t)}" for t in self.switches['tile']])
            ax6.grid(True, alpha=0.3, axis='y')
        else:
            ax6.text(0.5, 0.5, 'No tile switches\n(Whole map mode)', 
                    ha='center', va='center', fontsize=12)
            ax6.set_title('Tile Switch Times')
        
        # 7-8. Summary table
        ax7 = fig.add_subplot(gs[3, :])
        ax7.axis('off')
        
        # Build table data
        table_data = [
            ['Metric', 'Tiled', 'Whole', 'Δ (Tiled - Whole)'],
            ['Map Server Mem (Mean)', f"{self.tiled['map_server_mem_mb'].mean():.2f} MB", 
             f"{self.whole['map_server_mem_mb'].mean():.2f} MB",
             f"{self.tiled['map_server_mem_mb'].mean() - self.whole['map_server_mem_mb'].mean():+.2f} MB"],
            ['Total Memory (Mean)', f"{self.tiled['total_mem_mb'].mean():.2f} MB",
             f"{self.whole['total_mem_mb'].mean():.2f} MB",
             f"{self.tiled['total_mem_mb'].mean() - self.whole['total_mem_mb'].mean():+.2f} MB"],
            ['Total CPU (Mean)', f"{self.tiled['total_cpu'].mean():.2f} %",
             f"{self.whole['total_cpu'].mean():.2f} %",
             f"{self.tiled['total_cpu'].mean() - self.whole['total_cpu'].mean():+.2f} %"],
            ['Costmap Rate (Mean)', f"{self.tiled['costmap_update_rate_hz'].mean():.2f} Hz",
             f"{self.whole['costmap_update_rate_hz'].mean():.2f} Hz",
             f"{self.tiled['costmap_update_rate_hz'].mean() - self.whole['costmap_update_rate_hz'].mean():+.2f} Hz"],
            ['Disk Read (Total)', f"{self.tiled['total_disk_read_mb'].iloc[-1]:.2f} MB",
             f"{self.whole['total_disk_read_mb'].iloc[-1]:.2f} MB",
             f"{self.tiled['total_disk_read_mb'].iloc[-1] - self.whole['total_disk_read_mb'].iloc[-1]:+.2f} MB"],
        ]
        
        if self.switches is not None and len(self.switches) > 0:
            table_data.append(['Tile Switches', f"{len(self.switches)}", 'N/A', '—'])
            table_data.append(['Avg Switch Time', f"{self.switches['switch_time_ms'].mean():.2f} ms", 'N/A', '—'])
        
        table = ax7.table(cellText=table_data, loc='center', cellLoc='left',
                         colWidths=[0.3, 0.2, 0.2, 0.3])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 1.6)
        
        for i in range(4):
            table[(0, i)].set_text_props(fontweight='bold')
            table[(0, i)].set_facecolor('#2196F3')
            table[(0, i)].set_text_props(color='white')
        
        if save:
            path = os.path.join(self.output_dir, 'full_report.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            print(f"Saved: {path}")
        return fig

    def generate_all(self):
        """Generate all visualizations"""
        print("\nGenerating benchmark visualizations...")
        print("=" * 50)
        
        self.plot_memory()
        self.plot_cpu()
        self.plot_costmap()
        self.plot_disk_io()
        self.plot_summary_bars()
        self.plot_tile_switches()
        self.generate_summary_table()
        self.generate_full_report()
        
        print("=" * 50)
        print(f"All visualizations saved to: {self.output_dir}")


def main():
    parser = argparse.ArgumentParser(description='Visualize benchmark results')
    parser.add_argument('--tiled', '-t', required=True, help='Path to tiled benchmark CSV')
    parser.add_argument('--whole', '-w', required=True, help='Path to whole map benchmark CSV')
    parser.add_argument('--switches', '-s', help='Path to tile switches CSV (optional)')
    parser.add_argument('--output', '-o', help='Output directory (default: same as tiled CSV)')
    
    args = parser.parse_args()
    
    visualizer = BenchmarkVisualizer(
        tiled_csv=args.tiled,
        whole_csv=args.whole,
        switches_csv=args.switches,
        output_dir=args.output
    )
    
    visualizer.generate_all()


if __name__ == "__main__":
    main()
