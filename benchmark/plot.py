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
from bokeh.palettes import Category10, Spectral6

def extract_msg_size(filename):
    """Extract message size from filename."""
    pattern = r'array(\d+(?:[km])?)'
    match = re.search(pattern, filename.lower())
    if match:
        size_str = match.group(1)
        if size_str.endswith('k'):
            return int(size_str[:-1]) * 1024
        elif size_str.endswith('m'):
            return int(size_str[:-1]) * 1024 * 1024
        else:
            return int(size_str)
    return None

def parse_performance_file(filepath, config_type):
    """Parse a performance test JSON file and return relevant metrics."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        results_df = pd.DataFrame(data['analysis_results'])
        avg_latency = results_df['latency_mean'].mean()
        throughput = results_df['num_samples_received'].mean()
        
        return {
            'rmw_implementation': data.get('rmw_implementation', ''),
            'config_type': config_type,
            'avg_latency': avg_latency,
            'throughput': throughput,
            'is_zero_copy': data.get('is_zero_copy_transfer', False),
            'unbounded': data.get('msg_name', '') == 'UnboundedSequence'
        }
    except Exception as e:
        print(f"Error processing {filepath}: {str(e)}")
        raise

def create_performance_plots(config_dirs, output_file_path=None):
    """Create performance comparison plots."""
    results = []
    
    # Process each configuration directory
    for config_name, directory in config_dirs.items():
        if not os.path.exists(directory):
            print(f"Warning: Directory not found: {directory}")
            continue
            
        for filename in os.listdir(directory):
            if filename.endswith('.json'):
                filepath = os.path.join(directory, filename)
                try:
                    result = parse_performance_file(filepath, config_name)
                    msg_size = extract_msg_size(filename)
                    if msg_size is not None:
                        result['msg_size'] = msg_size
                        results.append(result)
                        print(f"Successfully processed {filename} from {config_name}")
                    else:
                        print(f"Skipped {filename} - couldn't determine message size")
                except Exception as e:
                    print(f"Error processing {filename}: {str(e)}")
    
    if not results:
        raise ValueError(f"No valid JSON files found in any of the provided directories")

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
        32, 64, 128, 256, 512,                          # Bytes
        1024, 2*1024, 4*1024, 8*1024, 16*1024,         # KB range
        32*1024, 64*1024, 128*1024, 256*1024, 512*1024, # Larger KB
        1024*1024, 2*1024*1024, 4*1024*1024            # MB range
    ]
    y_ticks = [1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0]
    
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
    
    # Create a different color for each RMW implementation and configuration combination
    colors = {}
    color_palettes = [Category10[10], Spectral6]
    all_colors = [color for palette in color_palettes for color in palette]
    
    # Create unique combinations of RMW and config type
    combinations = df.groupby(['rmw_implementation', 'config_type'])
    
    # Assign colors to each combination
    for i, ((rmw, config), _) in enumerate(combinations):
        colors[(rmw, config)] = all_colors[i % len(all_colors)]
    
    # Construct plot
    legend_items = []
    
    # Sort the DataFrame by msg_size within each group
    for (rmw, config), group in combinations:
        group = group.sort_values('msg_size')
        color = colors[(rmw, config)]
        
        # Create label
        label = f"{rmw} ({config})"
        
        source = ColumnDataSource(group)
        line = p.line('msg_size', 'avg_latency', line_color=color, line_width=2, source=source)
        scatter = p.scatter('msg_size', 'avg_latency', size=8, color=color, source=source)
        
        legend_items.append((label, [line, scatter]))
    
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
        ('Configuration', '@config_type'),
        ('Message Size', '@msg_size{0,0} bytes'),
        ('Average Latency', '@avg_latency{0.0000} ms')
    ]
    
    if output_file_path is None:
        output_file_path = 'performance_comparison.html'
    
    output_file(output_file_path)
    show(p)

def main():
    parser = argparse.ArgumentParser(description='Generate performance comparison plots from ROS 2 performance test results.')
    parser.add_argument('directories', nargs='+', help='Directories containing test results. Directory names will be used as labels.')
    parser.add_argument('--output', '-o', help='Output HTML file path (default: performance_comparison.html)')
    
    args = parser.parse_args()
    
    # Create dictionary of directories using their base names as labels
    config_dirs = {os.path.basename(d): d for d in args.directories}
    
    try:
        create_performance_plots(config_dirs, args.output)
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
