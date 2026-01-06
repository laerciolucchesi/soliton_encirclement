"""Plot agent telemetry from agent_telemetry.csv.

Creates one figure per node_id with:
- u vs timestamp
- velocity_norm vs timestamp

Run:
    python plot_telemetry.py

By default, reads ./agent_telemetry.csv (same directory as this script).
"""

from __future__ import annotations

import os
import sys

import pandas as pd
import matplotlib.pyplot as plt


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "agent_telemetry.csv")

    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return 1

    df = pd.read_csv(csv_path)

    required_cols = {"node_id", "timestamp", "u", "velocity_norm"}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"Missing columns in CSV: {sorted(missing)}")
        return 1

    if df.empty:
        print("CSV is empty.")
        return 0

    # Ensure numeric types and sort by time.
    df = df.copy()
    df["node_id"] = pd.to_numeric(df["node_id"], errors="coerce").astype("Int64")
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["u"] = pd.to_numeric(df["u"], errors="coerce")
    df["velocity_norm"] = pd.to_numeric(df["velocity_norm"], errors="coerce")
    df = df.dropna(subset=["node_id", "timestamp", "u", "velocity_norm"]).sort_values("timestamp")

    node_ids = sorted(df["node_id"].unique())
    if len(node_ids) == 0:
        print("No valid rows to plot.")
        return 0

    for node_id in node_ids:
        df_node = df[df["node_id"] == node_id].sort_values("timestamp")

        fig, (ax_u, ax_v) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        fig.suptitle(f"Telemetry - node_id={int(node_id)}")

        ax_u.plot(df_node["timestamp"], df_node["u"], linewidth=1.0)
        ax_u.set_ylabel("u")
        ax_u.grid(True, alpha=0.3)

        ax_v.plot(df_node["timestamp"], df_node["velocity_norm"], linewidth=1.0)
        ax_v.set_ylabel("||v||")
        ax_v.set_xlabel("timestamp (s)")
        ax_v.grid(True, alpha=0.3)

        fig.tight_layout()

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
