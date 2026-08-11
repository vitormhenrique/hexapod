#!/usr/bin/env python3
import json
n = {}
ts = []
for line in open("data/sessions/straight_walk.jsonl"):
    r = json.loads(line)
    n[r["stream"]] = n.get(r["stream"], 0) + 1
    ts.append(r["t"])
print(n, "span", round(ts[-1] - ts[0], 1), "s")
