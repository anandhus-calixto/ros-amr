#!/usr/bin/env python3
import time

def read_cpu_lines():
    lines = {}
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("cpu") and line[3].isdigit():
                parts = line.split()
                name = parts[0]
                vals = list(map(int, parts[1:]))
                lines[name] = vals
    return lines

a = read_cpu_lines()
time.sleep(2)
b = read_cpu_lines()

for core in sorted(a.keys()):
    va, vb = a[core], b[core]
    d = [y - x for x, y in zip(va, vb)]
    idle = d[3] + d[4]  # idle + iowait
    total = sum(d)
    busy_pct = 100.0 * (total - idle) / total if total > 0 else 0.0
    print(f"{core}: {busy_pct:.1f}% busy")
