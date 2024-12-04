# Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

ARRAY_SIZES = [
    "Array32",
    "Array64",
    "Array128",
    "Array256",
    "Array512",
    "Array1k",
    "Array4k",
    "Array16k",
    "Array32k",
    "Array64k",
    "Array256k",
    "Array1m",
    "Array2m",
    "Array4m",
]

def format_time(seconds: float) -> str:
    """Format seconds into a human-readable string."""
    return str(timedelta(seconds=int(seconds)))

def run_performance_tests(rmw_name: str, install_dir: Path, use_zero_copy: bool, runtime: int):
    """Run performance tests for all array sizes sequentially."""
    perf_test_path = install_dir / "performance_test/lib/performance_test/perf_test"
    
    # Locate performance_test binary
    if not perf_test_path.exists():
        raise FileNotFoundError(f"Performance test executable not found at: {perf_test_path}")
    
    # Prepare command
    base_cmd = [
        str(perf_test_path),
        "-c", "rclcpp-single-threaded-executor",
        "--max-runtime", str(runtime),              
        "--ignore", "5",                            # warmup time
        "--rate", "0"                               # as fast as possible
    ]
    if use_zero_copy:
        base_cmd.append("--zero-copy")

    # Create output directory if it doesn't exist
    Path("results").mkdir(exist_ok=True)

    # Track test runs
    total_start_time = time.time()
    total_tests = len(ARRAY_SIZES)
    tests_completed = 0

    # Run tests for each array size
    for array_size in ARRAY_SIZES:
        size_suffix = array_size.lower()
        test_start_time = time.time()
        
        # Construct output filename
        prefix = "zero-copy" if use_zero_copy else "regular"
        output_file = f"results/{rmw_name}-performance-{prefix}-{size_suffix}.json"
        
        # Build complete command with provided arguments
        cmd = base_cmd + ["--msg", array_size, "--logfile", output_file]
        
        # Print test details
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Starting test {tests_completed + 1}/{total_tests}: {array_size}")
        print(f"Output will be saved to: {output_file}")
        print(f"Command: {' '.join(cmd)}")
        
        # Run tests
        try:
            # Run test process
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    print(output.strip())
            return_code = process.poll()
            
            if return_code != 0:
                stderr_output = process.stderr.read()
                raise subprocess.CalledProcessError(return_code, cmd, stderr=stderr_output)
            
            # Calculate progress and estimated time remaining
            test_duration = time.time() - test_start_time
            tests_completed += 1

            elapsed_total = time.time() - total_start_time
            avg_test_time = elapsed_total / tests_completed
            remaining_tests = total_tests - tests_completed
            estimated_remaining = avg_test_time * remaining_tests
            
            # Notify user of test progression
            print(f"Successfully completed test for {array_size}")
            print(f"Test duration: {format_time(test_duration)}")
            print(f"Progress: {tests_completed}/{total_tests} tests completed")
            if remaining_tests > 0:
                print(f"Estimated time remaining: {format_time(estimated_remaining)}")
            
        except subprocess.CalledProcessError as e:
            print(f"Error running test for {array_size}:", file=sys.stderr)
            print(f"Exit code: {e.returncode}", file=sys.stderr)
            print(f"stdout: {e.stdout}", file=sys.stderr)
            print(f"stderr: {e.stderr}", file=sys.stderr)
            continue
        except KeyboardInterrupt:
            total_duration = time.time() - total_start_time
            print(f"\nTest interrupted by user after {format_time(total_duration)}")
            print(f"Completed {tests_completed}/{total_tests} tests")
            sys.exit(1)

    total_duration = time.time() - total_start_time
    print(f"\nAll tests completed in {format_time(total_duration)}")

def main():
    parser = argparse.ArgumentParser(description='Run ROS 2 performance tests for multiple message sizes.')
    parser.add_argument('rmw_name', help='Name of the RMW implementation, to be used as a label')
    parser.add_argument('install_dir', type=Path, help='Path to the ROS 2 installation directory')
    parser.add_argument('--zero-copy', action='store_true', help='Enable zero-copy transfer')
    parser.add_argument('--runtime', type=int, default=35, help='Test runtime in seconds (default: 35)')
    args = parser.parse_args()
    
    # Estimate total expected duration
    total_runtime = (args.runtime + 5) * len(ARRAY_SIZES)  # runtime + ignore time per test
    
    # Provide test overview
    print(f"Starting performance tests for RMW: {args.rmw_name}")
    print(f"Using installation directory: {args.install_dir}")
    print(f"Zero-copy enabled: {args.zero_copy}")
    print(f"Runtime per test: {args.runtime} seconds (plus 5 seconds ignore time)")
    print(f"Total number of tests to run: {len(ARRAY_SIZES)}")
    print(f"Estimated total duration: {format_time(total_runtime)}")
    print(f"Started at: {datetime.now().strftime('%H:%M:%S')}")
    
    try:
        run_performance_tests(args.rmw_name, args.install_dir, args.zero_copy, args.runtime)
    except FileNotFoundError as e:
        print(f"\nError: {e}", file=sys.stderr)
        print("Ensure that the installation path is correct.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\nError running tests: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
