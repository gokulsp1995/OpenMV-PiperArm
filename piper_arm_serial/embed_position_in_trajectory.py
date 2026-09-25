#!/usr/bin/env python3
"""Embed a recorded position (positions/*.yaml) into a trajectory file.

Trajectory files (trajectory_play_press.py's format) are self-contained --
their own positions: dict plus a trajectory: sequence. A recorded single
position from positions/*.yaml lives in a different pool and isn't usable
by name until its joints/end_pose are copied in. This does that copy and
sequence insertion in one step, and writes a NEW file so the original is
never touched.

    # add 'up-down-new' to trajectory_2.yaml, right before point_3
    python3 embed_position.py \\
        --position up-down-new \\
        --trajectory trajectory/trajectory_2.yaml \\
        --insert-before point_3 \\
        --out trajectory/trajectory_2_new.yaml

    # same, but also insert a press action right after it
    python3 embed_position.py \\
        --position up-down-new \\
        --trajectory trajectory/trajectory_2.yaml \\
        --insert-before point_3 \\
        --add-press up \\
        --out trajectory/trajectory_2_new.yaml

    # just append at the end of the sequence
    python3 embed_position.py \\
        --position up-down-new \\
        --trajectory trajectory/trajectory_2.yaml \\
        --append \\
        --out trajectory/trajectory_2_new.yaml
"""

import argparse
import os
import sys

import yaml


def load_position(name: str, positions_dir: str) -> dict:
    """Load a recorded position, in the format position_recorder.py writes:
    top-level joints: {...} and end_pose: {...}."""
    path = os.path.join(positions_dir, name + ".yaml")
    if not os.path.exists(path):
        # also accept a full path if the caller passed one
        path = name if name.endswith((".yaml", ".yml")) else name + ".yaml"
    if not os.path.exists(path):
        raise FileNotFoundError(
            "Position '%s' not found (looked for %s)" % (name, path))

    with open(path) as f:
        data = yaml.safe_load(f)

    joints = data.get("joints")
    if not joints:
        raise ValueError("'%s' has no 'joints' section -- is this a valid "
                         "recorded position file?" % path)

    return {
        "joints": joints,
        "end_pose": data.get("end_pose", {}),
    }


def sanitize_key(name: str) -> str:
    """Trajectory point keys in existing files use plain identifiers
    (point_1, point_2). Recorded position names often have hyphens
    (up-down-new), which are valid YAML keys but inconsistent with the
    file's existing style -- normalise to underscores."""
    return name.replace("-", "_")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--position", required=True,
                    help="name of the recorded position (from positions/)")
    ap.add_argument("--positions-dir", default="positions",
                    help="directory recorded positions live in (default: positions)")
    ap.add_argument("--trajectory", required=True,
                    help="the trajectory YAML to embed into (not modified)")
    ap.add_argument("--out", required=True,
                    help="path to write the new trajectory file to")
    ap.add_argument("--key", default=None,
                    help="name to give the new point inside the trajectory's "
                         "positions: block (default: --position with '-' "
                         "replaced by '_')")

    where = ap.add_mutually_exclusive_group()
    where.add_argument("--insert-before", metavar="POINT",
                       help="insert immediately before this existing waypoint")
    where.add_argument("--insert-after", metavar="POINT",
                       help="insert immediately after this existing waypoint")
    where.add_argument("--append", action="store_true",
                       help="add at the end of the sequence (default if "
                            "no other placement given)")

    ap.add_argument("--add-press", metavar="TARGET_ID", default=None,
                    help="also insert {action: press, target: TARGET_ID} "
                         "immediately after the new waypoint")

    args = ap.parse_args()

    if not os.path.exists(args.trajectory):
        print("ERROR: trajectory file not found: %s" % args.trajectory)
        sys.exit(1)

    pos_data = load_position(args.position, args.positions_dir)
    key = args.key or sanitize_key(args.position)

    with open(args.trajectory) as f:
        traj = yaml.safe_load(f)

    positions = traj.setdefault("positions", {})
    sequence = traj.setdefault("trajectory", [])

    if key in positions:
        print("WARNING: '%s' already exists in this trajectory's positions "
              "-- overwriting with the current recorded values." % key)
    positions[key] = pos_data

    new_items = [key]
    if args.add_press:
        new_items.append({"action": "press", "target": args.add_press})

    if args.insert_before:
        if args.insert_before not in sequence:
            print("ERROR: '%s' not found in this trajectory's sequence. "
                  "Existing sequence: %s" % (args.insert_before, sequence))
            sys.exit(1)
        idx = sequence.index(args.insert_before)
        sequence[idx:idx] = new_items

    elif args.insert_after:
        if args.insert_after not in sequence:
            print("ERROR: '%s' not found in this trajectory's sequence. "
                  "Existing sequence: %s" % (args.insert_after, sequence))
            sys.exit(1)
        idx = sequence.index(args.insert_after) + 1
        sequence[idx:idx] = new_items

    else:
        # default: append
        sequence.extend(new_items)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        yaml.dump(traj, f, default_flow_style=False, sort_keys=False)

    print("Wrote %s" % args.out)
    print("  added position '%s' (from %s)" % (key, args.position))
    print("  sequence is now: %s"
          % [s if isinstance(s, str) else s for s in sequence])


if __name__ == "__main__":
    main()