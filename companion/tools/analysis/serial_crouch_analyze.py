#!/usr/bin/env python3
"""Analyze serial_crouch_test.jsonl: goal-vs-present tracking per phase."""
import json
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "data/sessions/serial_crouch_test.jsonl"

phases = {}  # name -> (t_start, t_end)
rows = []
cur = None
for line in open(path):
    r = json.loads(line)
    if r["stream"] == "marker":
        ph = r["data"]["phase"]
        if ph.endswith("_end"):
            name = ph[:-4]
            if name in phases:
                phases[name] = (phases[name][0], r["t"])
        else:
            phases[ph] = (r["t"], None)
        continue
    rows.append(r)

print("phases:", {k: (round(v[1] - v[0], 1) if v[1] else None) for k, v in phases.items()})

TICKS_PER_DEG = 4096.0 / 360.0  # 11.377


def goal_servo_id(leg: int, joint: int) -> int:
    # Config schema v10 front/back rotation: logical leg L drives the servo
    # trio of the opposite corner.
    return ((leg + 3) % 6) * 3 + joint + 1


def goal_tick(angle_centideg: int) -> int:
    # Stock config: sign +1, trim 0, center 2048.
    t = round(2048 + (angle_centideg / 100.0) * TICKS_PER_DEG)
    return max(0, min(4095, t))


for name, (t0, t1) in phases.items():
    if t1 is None:
        continue
    goals = defaultdict(list)    # id -> [(t, tick)]
    present = defaultdict(list)  # id -> [(t, pos, err, torque)]
    for r in rows:
        if not (t0 <= r["t"] <= t1):
            continue
        d = r["data"]
        if r["stream"] == "servo_goals":
            for g in d.get("goals", []):
                sid = goal_servo_id(g["leg"], g["joint"])
                goals[sid].append((r["t"], goal_tick(g["angle_centideg"])))
        elif r["stream"] == "servo_status":
            for s in d.get("servos", []):
                present[s["id"]].append(
                    (r["t"], s.get("position"),
                     s.get("hardware_error", 0),
                     s.get("torque_enabled")))
    print(f"\n=== phase {name} ===")
    print("id  goal[min..max] amp   present[min..max] amp   max|g-p|  hw_err")
    for sid in sorted(goals.keys()):
        gt = [tk for _, tk in goals[sid]]
        pv = [p for _, p, _, _ in present.get(sid, []) if p is not None and p >= 0]
        errs = {e for _, _, e, _ in present.get(sid, []) if e}
        if not pv:
            print(f"{sid:2d}  [{min(gt):4d}..{max(gt):4d}] {max(gt)-min(gt):4d}   (no status)")
            continue
        # max |goal - present| with nearest-time alignment
        max_err = 0
        gi = 0
        gl = goals[sid]
        for t, p, _, _ in present[sid]:
            if p is None or p < 0:
                continue
            while gi + 1 < len(gl) and gl[gi + 1][0] <= t:
                gi += 1
            max_err = max(max_err, abs(gl[gi][1] - p))
        gi = 0
        print(f"{sid:2d}  [{min(gt):4d}..{max(gt):4d}] {max(gt)-min(gt):4d}   "
              f"[{min(pv):4d}..{max(pv):4d}] {max(pv)-min(pv):4d}   {max_err:5d}    "
              f"{sorted(errs) if errs else '-'}")
