# Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

import os
import json
import pandas as pd
import re
import argparse
from bokeh.plotting import figure, show, save
from bokeh.layouts import column
from bokeh.io import output_file
from bokeh.models import ColumnDataSource, Legend, CustomJSTickFormatter, Range1d
from bokeh.palettes import Category10

def extract_msg_size(data):
    """Extract message size from msg_name field."""
    pattern = r'Array(\d+(?:[km])?)'
    match = re.search(pattern, data.get('msg_name', ''))
    if match:
        size_str = match.group(1)
        if size_str.endswith('k'):
            return int(size_str[:-1]) * 1024
        elif size_str.endswith('m'):
            return int(size_str[:-1]) * 1024 * 1024
        else:
            return int(size_str)
    return None

def parse_performance_file(filepath):
    """Parse a performance test JSON file and return relevant metrics."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        results_df = pd.DataFrame(data['analysis_results'])
        avg_latency = results_df['latency_mean'].mean()
        
        return {
            'rmw_implementation': data.get('rmw_implementation', ''),
            'avg_latency': avg_latency,
            'msg_size': extract_msg_size(data)
        }
    except Exception as e:
        print(f"Error processing {filepath}: {str(e)}")
        raise

def create_performance_plots(data_directory, output_file_path=None):
    """Create performance comparison plots."""
    if not os.path.exists(data_directory):
        raise FileNotFoundError(f"Directory not found: {data_directory}")

    results = []
    for filename in os.listdir(data_directory):
        if filename.endswith('.json'):
            filepath = os.path.join(data_directory, filename)
            try:
                result = parse_performance_file(filepath)
                if result['msg_size'] is not None:
                    results.append(result)
                    print(f"Successfully processed {filename}")
                else:
                    print(f"Skipped {filename} - couldn't determine message size")
            except Exception as e:
                print(f"Error processing {filename}: {str(e)}")
    
    if not results:
        raise ValueError(f"No valid JSON files found in {data_directory}")

    df = pd.DataFrame(results)
    
    p = figure(
        title='Latency vs Message Size',
        x_axis_label='Message Size',
        y_axis_label='Average Latency',
        x_axis_type='log',
        y_axis_type='log',
        width=800,
        height=600
    )
    
    x_ticks = [
        32,             # 32 B
        64,             # 64 B
        128,            # 128 B
        256,            # 256 B
        512,            # 512 B
        1024,           # 1 KB
        2*1024,         # 2 KB
        4*1024,         # 4 KB
        8*1024,         # 8 KB
        16*1024,        # 16 KB
        32*1024,        # 32 KB
        64*1024,        # 64 KB
        128*1024,       # 128 KB
        256*1024,       # 256 KB
        512*1024,       # 512 KB
        1024*1024,      # 1 MB
        2*1024*1024,    # 2 MB
        4*1024*1024     # 4 MB
    ]
    y_ticks = [
        1e-9,        # 1 ns
        1e-8,        # 10 ns
        1e-7,        # 100 ns
        1e-6,        # 1 µs
        1e-5,        # 10 µs
        1e-4,        # 100 µs
        1e-3,        # 1 ms
        1e-2,        # 10 ms
        1e-1,        # 100 ms
        1.0          # 1 s
    ]
    
    x_formatter = CustomJSTickFormatter(code="""
        if (tick < 1024) {
            return tick.toString() + ' B';
        } else if (tick < 1048576) {
            return (tick/1024).toFixed(0) + ' KB';
        } else {
            return (tick/1048576).toFixed(0) + ' MB';
        }
    """)
    y_formatter = CustomJSTickFormatter(code="""
        if (tick < 1e-6) {
            return (tick * 1e9).toFixed(0) + ' ns';
        } else if (tick < 1e-3) {
            return (tick * 1e6).toFixed(0) + ' µs';
        } else if (tick < 1) {
            return (tick * 1e3).toFixed(0) + ' ms';
        }
        return tick.toFixed(0) + ' s';
    """)
    
    p.xaxis.ticker = x_ticks
    p.xaxis.formatter = x_formatter
    p.yaxis.ticker = y_ticks
    p.yaxis.formatter = y_formatter
    
    p.x_range = Range1d(20, 8*1024*1024)
    p.y_range = Range1d(1e-9, 1.0)
    
    p.grid.grid_line_color = "#CCCCCC"
    p.grid.grid_line_alpha = 0.8
    p.xgrid.grid_line_dash = 'dotted'
    p.ygrid.grid_line_dash = 'dotted'
    
    # Remove plot outline
    p.outline_line_color = None
    p.toolbar.autohide = True
    
    # Construct plot
    colors = Category10[10]
    
    legend_items = []
    for i, (rmw, group) in enumerate(df.groupby('rmw_implementation')):
        color = colors[i % len(colors)]
        group = group.sort_values('msg_size')
        
        source = ColumnDataSource(group)
        line = p.line('msg_size', 'avg_latency', line_color=color, line_width=2, source=source)
        scatter = p.scatter('msg_size', 'avg_latency', size=8, color=color, source=source)
        
        legend_items.append((rmw, [line, scatter]))
    
    # Add legend
    legend = Legend(
        items=legend_items,
        location="center",
        orientation="horizontal",
        click_policy="hide"
    )
    p.add_layout(legend, 'above')
    
    # Configure hover tool
    p.hover.tooltips = [
        ('RMW', '@rmw_implementation'),
        ('Message Size', '@msg_size{0,0} bytes'),
        ('Average Latency', '@avg_latency{0.0000} ms')
    ]
    
    if output_file_path is None:
        output_file_path = os.path.join(data_directory, 'performance_comparison.html')
    
    output_file(output_file_path)
    show(p)

def main():
    parser = argparse.ArgumentParser(description='Generate performance comparison plots from ROS 2 performance test results.')
    parser.add_argument('data_dir', help='Directory containing the performance test JSON files')
    parser.add_argument('--output', '-o', help='Output HTML file path (default: <data_dir>/performance_comparison.html)')
    
    args = parser.parse_args()
    
    try:
        create_performance_plots(args.data_dir, args.output)
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
