#!/usr/bin/env python3
import json
import csv
import os
import glob

CHOREO_DIR = "src/main/deploy/choreo"
CHOR_FILE = os.path.join(CHOREO_DIR, "Rebuilt.chor")
OUTPUT_FILE = "choreo_export.csv"

with open(CHOR_FILE) as f:
    data = json.load(f)

trajectories = sorted(
    os.path.splitext(os.path.basename(p))[0]
    for p in glob.glob(os.path.join(CHOREO_DIR, "*.traj"))
)

expressions = data["variables"]["expressions"]
poses = data["variables"]["poses"]

with open(OUTPUT_FILE, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)

    # --- Trajectories ---
    writer.writerow(["TRAJECTORIES"])
    writer.writerow(["name"])
    for name in trajectories:
        writer.writerow([name])

    writer.writerow([])

    # --- Expression Variables ---
    writer.writerow(["VARIABLES (EXPRESSIONS)"])
    writer.writerow(["name", "dimension", "expression", "value"])
    for name, info in sorted(expressions.items()):
        writer.writerow([name, info["dimension"], info["var"]["exp"], info["var"]["val"]])

    writer.writerow([])

    # --- Pose Variables ---
    writer.writerow(["POSES"])
    writer.writerow(["name", "x_expression", "x_value (m)", "y_expression", "y_value (m)", "heading_expression", "heading_value (rad)"])
    for name, pose in sorted(poses.items()):
        writer.writerow([
            name,
            pose["x"]["exp"], pose["x"]["val"],
            pose["y"]["exp"], pose["y"]["val"],
            pose["heading"]["exp"], pose["heading"]["val"],
        ])

print(f"Exported to {OUTPUT_FILE}")
print(f"  {len(trajectories)} trajectories")
print(f"  {len(expressions)} expression variables")
print(f"  {len(poses)} pose variables")
