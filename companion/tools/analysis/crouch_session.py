#!/usr/bin/env python3
"""Analyze a crouch-walk session: goal vs present per joint + errors."""
import json
import sys
from pathlib import Path

sess_dir = Path(sys.argv[1])
path = sess_dir / "telemetry.jsonl"

states = {}
gate_open = 0
goals = []    # (t_ms, {id: tick})
status = []   # (t_ms, {id: dict})
rc_frames = rc_armed = rc_kill = rc_failsafe = 0
rc_sample = None
for line in open(path):
    r = json.loads(line)
    d = r["data"]
    t = r["robot_time_ms"]
    s = r["stream"]
    if s == "control_state":
        states[d["state"]] = states.get(d["state"], 0) + 1
        if d["motion_gate"]:
            gate_open += 1
    elif s == "rc_input":
        rc_frames += 1
        if rc_sample is None:
            rc_sample = d
        if d.get("armed"):
            rc_armed += 1
        if d.get("kill"):
            rc_kill += 1
        if d.get("failsafe"):
            rc_failsafe += 1
    elif s == "servo_goals" and d.get("goals"):
        goals.append((t, {g["id"]: g for g in d["goals"]}))
    elif s == "servo_status" and d.get("servos"):
        status.append((t, {v["id"]: v for v in d["servos"]}))

print("states seen:", states)
print("motion_gate open frames:", gate_open)
print("rc frames:", rc_frames, "armed:", rc_armed, "kill:", rc_kill,
      "failsafe:", rc_failsafe)
if rc_sample is not None:
    print("first rc_input:", json.dumps(rc_sample)[:400])
print("goal frames:", len(goals), " status frames:", len(status))
if not goals or not status:
    sys.exit(0)

# Per-servo: goal range, present range, tracking error, hardware errors.
ids = sorted(goals[-1][1].keys())
print("\nid   goal[min..max]   present[min..max]  max|goal-present|  hw_err torque_drop")
for sid in ids:
    gmin, gmax = 99999, -1
    pmin, pmax = 99999, -1
    hw = set()
    torque_drops = 0
    # Align: for each status frame, find latest goal frame before it.
    gi = 0
    max_err = 0
    for t, sv in status:
        if sid not in sv:
            continue
        v = sv[sid]
        p = v.get("present_position", -1)
        if p is None or p < 0:
            continue
        pmin, pmax = min(pmin, p), max(pmax, p)
        e = v.get("hardware_error", 0)
        if e:
            hw.add(e)
        if v.get("torque_enabled") is False:
            torque_drops += 1
        while gi + 1 < len(goals) and goals[gi + 1][0] <= t:
            gi += 1
        g = goals[gi][1].get(sid)
        if g:
            max_err = max(max_err, abs(g["tick"] - p))
    for t, gv in goals:
        if sid in gv:
            tk = gv[sid]["tick"]
            gmin, gmax = min(gmin, tk), max(gmax, tk)
    print(f"{sid:2d}  [{gmin:4d}..{gmax:4d}]      [{pmin:4d}..{pmax:4d}]        {max_err:5d}          "
          f"{sorted(hw) if hw else '-'}   {torque_drops}")
