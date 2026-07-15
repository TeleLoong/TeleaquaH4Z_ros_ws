#!/usr/bin/env python3
"""Run a controller-free, quasi-static Teleh4z free-surface sweep in Gazebo."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path


DIAGNOSTIC_COLUMNS = [
    "actual_z_m",
    "actual_roll_rad",
    "actual_pitch_rad",
    "actual_yaw_rad",
    "submersion_ratio_body",
    "submersion_ratio_propeller_0",
    "submersion_ratio_propeller_1",
    "submersion_ratio_propeller_2",
    "submersion_ratio_propeller_3",
    "buoyancy_z_N",
    "drag_z_N",
    "gravity_z_N",
    "net_force_z_N",
    "center_of_buoyancy_z",
]
FLOAT_RE = re.compile(r"data:\s*([-+0-9.eEnNaAiIfF]+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-name", default="teleh4z_zaxis_0")
    parser.add_argument("--world-name", default="playground")
    parser.add_argument("--z-start", type=float, default=-1.0)
    parser.add_argument("--z-stop", type=float, default=1.0)
    parser.add_argument("--z-step", type=float, default=0.01)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, nargs="+", default=[0.0, 10.0, -10.0])
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--settle-steps", type=int, default=20)
    parser.add_argument("--settle-step-s", type=float, default=0.005)
    parser.add_argument("--topic-timeout-s", type=float, default=3.0)
    parser.add_argument("--pose-retries", type=int, default=5)
    parser.add_argument("--pose-timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("experiments/static_free_surface/output")
        / f"teleh4z_static_sweep_{datetime.now():%Y%m%d_%H%M%S}.csv",
    )
    return parser.parse_args()


def quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def z_values(start: float, stop: float, step: float) -> list[float]:
    if step <= 0 or stop < start:
        raise ValueError("Require z-step > 0 and z-stop >= z-start")
    count = int(round((stop - start) / step))
    values = [start + i * step for i in range(count + 1)]
    if not math.isclose(values[-1], stop, abs_tol=1e-9):
        values.append(stop)
    return values


class StateReader:
    def __init__(self, topic: str):
        self._states: queue.Queue[list[float]] = queue.Queue()
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        assert self._proc.stdout is not None
        values: list[float] = []
        for line in self._proc.stdout:
            match = FLOAT_RE.search(line)
            if match:
                values.append(float(match.group(1)))
            elif line.strip() == "" and values:
                if len(values) == len(DIAGNOSTIC_COLUMNS):
                    self._states.put(values)
                values = []

    def drain(self) -> None:
        while True:
            try:
                self._states.get_nowait()
            except queue.Empty:
                return

    def next(self, timeout: float) -> list[float]:
        return self._states.get(timeout=timeout)

    def next_for_pose(
        self, z: float, pitch_rad: float, timeout: float, tolerance: float = 1e-4
    ) -> list[float]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise queue.Empty(
                    f"No diagnostic matched z={z:.6g}, pitch={pitch_rad:.6g}"
                )
            state = self.next(remaining)
            if (
                abs(state[0] - z) <= tolerance
                and abs(state[2] - pitch_rad) <= tolerance
            ):
                return state

    def close(self) -> None:
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._proc.kill()


def set_pose(args: argparse.Namespace, z: float, pitch_deg: float) -> None:
    r, p, y = map(math.radians, (args.roll_deg, pitch_deg, args.yaw_deg))
    qx, qy, qz, qw = quaternion(r, p, y)
    request = (
        f'name: "{args.model_name}" '
        f'position {{x: {args.x:.12g} y: {args.y:.12g} z: {z:.12g}}} '
        f'orientation {{x: {qx:.12g} y: {qy:.12g} z: {qz:.12g} w: {qw:.12g}}}'
    )
    last_error = ""
    for attempt in range(1, args.pose_retries + 1):
        try:
            result = subprocess.run(
                [
                    "gz", "service", "-s", f"/world/{args.world_name}/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", str(args.pose_timeout_ms), "--req", request,
                ],
                capture_output=True,
                text=True,
                timeout=args.pose_timeout_ms / 1000.0 + 3.0,
            )
            if result.returncode == 0 and "data: true" in result.stdout.lower():
                return
            last_error = result.stderr or result.stdout
        except subprocess.TimeoutExpired as error:
            last_error = str(error)
        print(
            f"set_pose attempt {attempt}/{args.pose_retries} failed; retrying...",
            flush=True,
        )
        time.sleep(min(0.2 * attempt, 1.0))
    raise RuntimeError(f"set_pose failed after {args.pose_retries} attempts: {last_error}")


def main() -> None:
    args = parse_args()
    if args.settle_steps < 1:
        raise ValueError("--settle-steps must be positive")
    if args.pose_retries < 1 or args.pose_timeout_ms < 1:
        raise ValueError("Pose retries and timeout must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    topic = f"/model/{args.model_name}/hydrodynamics_state"
    reader = StateReader(topic)
    fieldnames = [
        "source", "model_name", "arm_configuration",
        "roll_deg", "pitch_deg", "yaw_deg", "z_m",
        "actual_roll_deg", "actual_pitch_deg", "actual_yaw_deg",
        *DIAGNOSTIC_COLUMNS[4:],
    ]
    try:
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for pitch_deg in args.pitch_deg:
                for z in z_values(args.z_start, args.z_stop, args.z_step):
                    reader.drain()
                    set_pose(args, z, pitch_deg)
                    # The generated model is static. Settle steps therefore
                    # mean observation time, not repeated pose commands.
                    time.sleep(args.settle_steps * args.settle_step_s)
                    state = reader.next_for_pose(
                        z, math.radians(pitch_deg), args.topic_timeout_s
                    )
                    row = {
                        "source": "gazebo",
                        "model_name": args.model_name,
                        "arm_configuration": "deployed_fixed",
                        "roll_deg": args.roll_deg,
                        "pitch_deg": pitch_deg,
                        "yaw_deg": args.yaw_deg,
                        "z_m": state[0],
                        "actual_roll_deg": math.degrees(state[1]),
                        "actual_pitch_deg": math.degrees(state[2]),
                        "actual_yaw_deg": math.degrees(state[3]),
                    }
                    row.update(zip(DIAGNOSTIC_COLUMNS[4:], state[4:]))
                    writer.writerow(row)
                    stream.flush()
                    print(
                        f"pitch={pitch_deg:+g} deg z={state[0]:+.3f} "
                        f"ratio={state[4]:.4f}"
                    )
    finally:
        reader.close()
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
