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

    # Optional signals (newer telemetry): components and per-tick deltas.
    has_u_kdv = "u_kdv" in df.columns
    has_u_nom = "u_nom" in df.columns
    has_u_err = "u_err" in df.columns
    has_delta_u = "delta_u" in df.columns
    has_delta_u_kdv = "delta_u_kdv" in df.columns
    has_delta_u_nom = "delta_u_nom" in df.columns
    has_delta_u_err = "delta_u_err" in df.columns

    if df.empty:
        print("CSV is empty.")
        return 0

    # Ensure numeric types and sort by time.
    df = df.copy()
    df["node_id"] = pd.to_numeric(df["node_id"], errors="coerce").astype("Int64")
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["u"] = pd.to_numeric(df["u"], errors="coerce")
    if has_u_kdv:
        df["u_kdv"] = pd.to_numeric(df["u_kdv"], errors="coerce")
    if has_u_nom:
        df["u_nom"] = pd.to_numeric(df["u_nom"], errors="coerce")
    if has_u_err:
        df["u_err"] = pd.to_numeric(df["u_err"], errors="coerce")
    if has_delta_u:
        df["delta_u"] = pd.to_numeric(df["delta_u"], errors="coerce")
    if has_delta_u_kdv:
        df["delta_u_kdv"] = pd.to_numeric(df["delta_u_kdv"], errors="coerce")
    if has_delta_u_nom:
        df["delta_u_nom"] = pd.to_numeric(df["delta_u_nom"], errors="coerce")
    if has_delta_u_err:
        df["delta_u_err"] = pd.to_numeric(df["delta_u_err"], errors="coerce")
    df["velocity_norm"] = pd.to_numeric(df["velocity_norm"], errors="coerce")

    drop_subset = ["node_id", "timestamp", "u", "velocity_norm"]
    if has_u_kdv:
        drop_subset.append("u_kdv")
    if has_u_nom:
        drop_subset.append("u_nom")
    if has_u_err:
        drop_subset.append("u_err")
    if has_delta_u:
        drop_subset.append("delta_u")
    if has_delta_u_kdv:
        drop_subset.append("delta_u_kdv")
    if has_delta_u_nom:
        drop_subset.append("delta_u_nom")
    if has_delta_u_err:
        drop_subset.append("delta_u_err")
    df = df.dropna(subset=drop_subset).sort_values("timestamp")

    # Backward-compatible reconstruction (best-effort): if only one component exists, infer the other.
    # Note: with 3 components, missing pieces are inferred as "remainder".
    if has_u_kdv and not (has_u_nom or has_u_err):
        df["u_nom"] = df["u"] - df["u_kdv"]
        df["u_err"] = 0.0
        has_u_nom = True
        has_u_err = True
    elif has_u_nom and not (has_u_kdv or has_u_err):
        df["u_kdv"] = df["u"] - df["u_nom"]
        df["u_err"] = 0.0
        has_u_kdv = True
        has_u_err = True
    elif has_u_err and not (has_u_kdv or has_u_nom):
        df["u_kdv"] = 0.0
        df["u_nom"] = df["u"] - df["u_err"]
        has_u_kdv = True
        has_u_nom = True
    else:
        # If some (but not all) are present, fill missing as remainder/zero.
        if has_u_kdv and has_u_nom and not has_u_err:
            df["u_err"] = df["u"] - df["u_kdv"] - df["u_nom"]
            has_u_err = True
        if has_u_kdv and has_u_err and not has_u_nom:
            df["u_nom"] = df["u"] - df["u_kdv"] - df["u_err"]
            has_u_nom = True
        if has_u_nom and has_u_err and not has_u_kdv:
            df["u_kdv"] = df["u"] - df["u_nom"] - df["u_err"]
            has_u_kdv = True

    node_ids = sorted(df["node_id"].unique())
    if len(node_ids) == 0:
        print("No valid rows to plot.")
        return 0

    for node_id in node_ids:
        df_node = df[df["node_id"] == node_id].sort_values("timestamp")

        fig, (ax_u, ax_du, ax_v) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        fig.suptitle(f"Telemetry - node_id={int(node_id)}")

        ax_u.plot(df_node["timestamp"], df_node["u"], linewidth=1.2, label="u")
        if has_u_kdv:
            ax_u.plot(df_node["timestamp"], df_node["u_kdv"], linewidth=1.0, label="u_kdv")
        if has_u_nom:
            ax_u.plot(df_node["timestamp"], df_node["u_nom"], linewidth=1.0, label="u_nom")
        if has_u_err:
            ax_u.plot(df_node["timestamp"], df_node["u_err"], linewidth=1.0, label="u_err")
        ax_u.set_ylabel("u")
        ax_u.grid(True, alpha=0.3)
        ax_u.legend(loc="best")

        # Per-tick increments (dt * du_*). If not present, keep the subplot but annotate.
        if has_delta_u:
            ax_du.plot(df_node["timestamp"], df_node["delta_u"], linewidth=1.2, label="delta_u")
        if has_delta_u_kdv:
            ax_du.plot(df_node["timestamp"], df_node["delta_u_kdv"], linewidth=1.0, label="delta_u_kdv")
        if has_delta_u_nom:
            ax_du.plot(df_node["timestamp"], df_node["delta_u_nom"], linewidth=1.0, label="delta_u_nom")
        if has_delta_u_err:
            ax_du.plot(df_node["timestamp"], df_node["delta_u_err"], linewidth=1.0, label="delta_u_err")
        ax_du.set_ylabel("delta_u")
        ax_du.axhline(0.0, color="k", linewidth=0.8, alpha=0.4)
        ax_du.grid(True, alpha=0.3)
        if has_delta_u or has_delta_u_kdv or has_delta_u_nom or has_delta_u_err:
            ax_du.legend(loc="best")
        else:
            ax_du.text(
                0.5,
                0.5,
                "delta_u_* columns not found in CSV",
                transform=ax_du.transAxes,
                ha="center",
                va="center",
                fontsize=10,
                alpha=0.7,
            )

        ax_v.plot(df_node["timestamp"], df_node["velocity_norm"], linewidth=1.0)
        ax_v.set_ylabel("||v||")
        ax_v.set_xlabel("timestamp (s)")
        ax_v.grid(True, alpha=0.3)

        fig.tight_layout()

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
