#!/usr/bin/env python3
# Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

"""Passes benchmark output through and summarizes the reports of repeated runs."""

import re
import statistics
import sys

ANSI = re.compile(r'\x1b\[[0-9;]*m')
UNITS = {'ns': 1e-3, 'µs': 1.0, 'ms': 1e3, 's': 1e6}
REPORT = re.compile(r'REPORT · (.*)')
PERCENTILE = re.compile(r'\s+(p50|p99)\s+([\d.]+)\s*(ns|µs|ms|s)\b')
VALUE = re.compile(r'^([a-z ]+ per [a-z ]+): ([\d.]+)( MB)?$')
ALIVE = re.compile(r'^([a-z]+) alive: (\d+)/(\d+)$')


def micros(value):
    if value >= 1000:
        return f'{value / 1000:.2f} ms'
    if value >= 1:
        return f'{value:.2f} µs'
    return f'{value * 1000:.0f} ns'


def main():
    percentiles = {}
    values = {}
    report = None
    level = ''
    for line in sys.stdin:
        sys.stdout.write(line)
        plain = ANSI.sub('', line.rstrip('\n'))
        match = REPORT.search(plain)
        if match:
            report = match.group(1).strip()
            continue
        match = PERCENTILE.match(plain)
        if match and report:
            entry = percentiles.setdefault(report, {'p50': [], 'p99': []})
            entry[match.group(1)].append(float(match.group(2)) * UNITS[match.group(3)])
            continue
        match = ALIVE.match(plain.strip())
        if match:
            level = f', {match.group(3)} requested'
            values.setdefault(f'{match.group(1)} alive{level}', ([], ''))[0].append(float(match.group(2)))
            continue
        match = VALUE.match(plain.strip())
        if match:
            values.setdefault(match.group(1) + level, ([], match.group(3) or ''))[0].append(float(match.group(2)))
    runs = max([len(entry['p50']) for entry in percentiles.values()] + [len(v[0]) for v in values.values()] + [0])
    if runs < 2:
        return
    print(f'\nSUMMARY of {runs} runs: median p50 (range of p50), median p99')
    for name, entry in percentiles.items():
        p50, p99 = entry['p50'], entry['p99']
        print(f'  {name}: {micros(statistics.median(p50))} ({micros(min(p50))} to {micros(max(p50))})'
              + (f', {micros(statistics.median(p99))}' if p99 else ''))
    for name, (samples, unit) in values.items():
        print(f'  {name}: {statistics.median(samples):.1f}{unit} ({min(samples):.1f} to {max(samples):.1f})')


if __name__ == '__main__':
    main()
