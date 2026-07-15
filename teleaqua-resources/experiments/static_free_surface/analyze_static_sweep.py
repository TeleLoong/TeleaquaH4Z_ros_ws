#!/usr/bin/env python3
"""Plot Stonefish/Gazebo static sweeps and write Table 1 metrics."""

from __future__ import annotations

import argparse
import math
import os
import warnings
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-teleh4z-static")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ALIASES = {
    "z_m": ("z_m", "pos_z"),
    "pitch_deg": ("pitch_deg",),
    "submersion_ratio_body": ("submersion_ratio_body", "submersion_ratio"),
    "buoyancy_z_N": ("buoyancy_z_N", "buoy_z"),
    "net_force_z_N": ("net_force_z_N", "force_z_N"),
    "center_of_buoyancy_z": ("center_of_buoyancy_z", "cob_z"),
}

SOURCE_STYLE = {
    "Stonefish": {
        "color": "#0072B2",
        "linestyle": "-",
        "linewidth": 1.65,
        "zorder": 2,
    },
    "Gazebo": {
        "color": "#D55E00",
        "linestyle": "--",
        "linewidth": 1.65,
        "zorder": 3,
    },
}

def configure_publication_style() -> None:
    """Configure an IEEE/ICRA-friendly, colorblind-safe plotting style."""
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": [
                "Times New Roman",
                "Times",
                "Nimbus Roman",
                "STIXGeneral",
                "DejaVu Serif",
            ],
            "mathtext.fontset": "stix",
            "font.size": 8.0,
            "axes.labelsize": 8.0,
            "axes.titlesize": 8.5,
            "axes.titleweight": "semibold",
            "axes.linewidth": 0.8,
            "axes.edgecolor": "#333333",
            "xtick.labelsize": 7.5,
            "ytick.labelsize": 7.5,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.major.size": 3.0,
            "ytick.major.size": 3.0,
            "xtick.major.width": 0.7,
            "ytick.major.width": 0.7,
            "legend.fontsize": 8.0,
            "legend.frameon": False,
            "lines.solid_capstyle": "round",
            "grid.color": "#B8B8B8",
            "grid.linewidth": 0.45,
            "grid.alpha": 0.32,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.dpi": 400,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gazebo", type=Path, required=True)
    parser.add_argument(
        "--stonefish",
        type=Path,
        help="Optional Stonefish static-sweep CSV for cross-domain overlays.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("experiments/static_free_surface/output"))
    parser.add_argument("--stonefish-z-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    return parser.parse_args()


def normalize(path: Path, source: str, z_sign: float = 1.0) -> pd.DataFrame:
    frame = pd.read_csv(path, comment="#")
    result = pd.DataFrame(index=frame.index)
    for canonical, aliases in ALIASES.items():
        found = next((name for name in aliases if name in frame.columns), None)
        if found is not None:
            result[canonical] = pd.to_numeric(frame[found], errors="coerce")
    required = {"z_m", "submersion_ratio_body", "buoyancy_z_N"}
    missing = sorted(required - set(result.columns))
    if missing:
        raise ValueError(f"{source} CSV missing columns (or aliases): {missing}")
    result["z_m"] *= z_sign
    if "pitch_deg" not in result:
        if "pitch" in frame:
            result["pitch_deg"] = np.rad2deg(pd.to_numeric(frame["pitch"], errors="coerce"))
        elif "pitch_rad" in frame:
            result["pitch_deg"] = np.rad2deg(pd.to_numeric(frame["pitch_rad"], errors="coerce"))
        else:
            result["pitch_deg"] = 0.0
    result["pitch_deg"] = result["pitch_deg"].round(3)
    result["source"] = source
    numeric = [
        column
        for column in result.columns
        if column not in {"source", "pitch_deg", "z_m"}
    ]
    result = result.dropna(subset=["z_m", "submersion_ratio_body", "buoyancy_z_N"])
    return result.groupby(["source", "pitch_deg", "z_m"], as_index=False)[numeric].median()


def crossing(x: np.ndarray, y: np.ndarray, target: float) -> float:
    finite = np.isfinite(x) & np.isfinite(y)
    x, y = x[finite], y[finite]
    if len(x) == 0 or target < np.min(y) or target > np.max(y):
        return math.nan
    exact = np.flatnonzero(np.isclose(y, target, atol=1e-12))
    if exact.size:
        return float(x[exact[0]])
    for i in range(len(x) - 1):
        if (y[i] - target) * (y[i + 1] - target) < 0:
            return float(x[i] + (target - y[i]) * (x[i + 1] - x[i]) / (y[i + 1] - y[i]))
    return math.nan


def metrics(group: pd.DataFrame) -> dict[str, float | str]:
    group = group.sort_values("z_m")
    z = group["z_m"].to_numpy(float)
    ratio = group["submersion_ratio_body"].to_numpy(float)
    buoyancy = group["buoyancy_z_N"].to_numpy(float)
    z99, z50, z01 = (crossing(z, ratio, value) for value in (0.99, 0.50, 0.01))
    net = group.get("net_force_z_N", pd.Series(np.nan, index=group.index)).to_numpy(float)
    slopes = np.gradient(buoyancy, z) if len(z) >= 2 else np.array([math.nan])
    smoothness = float(np.nanmax(np.abs(np.diff(slopes)))) if len(slopes) >= 2 else math.nan
    return {
        "source": str(group["source"].iloc[0]),
        "pitch_deg": float(group["pitch_deg"].iloc[0]),
        "z_at_ratio_0.99": z99,
        "z_at_ratio_0.50": z50,
        "z_at_ratio_0.01": z01,
        "transition_width_z": z01 - z99 if np.isfinite(z01 + z99) else math.nan,
        "max_buoyancy_N": float(np.nanmax(buoyancy)),
        "equilibrium_z": crossing(z, net, 0.0),
        "smoothness_dFdz": smoothness,
    }


def save_publication_figure(fig: plt.Figure, output_stem: Path) -> None:
    output_stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        output_stem.with_suffix(".pdf"),
        bbox_inches="tight",
        pad_inches=0.02,
    )
    fig.savefig(
        output_stem.with_suffix(".png"),
        dpi=400,
        bbox_inches="tight",
        pad_inches=0.02,
    )
    plt.close(fig)


def format_table_value(value: object, decimals: int = 3) -> str:
    if pd.isna(value):
        return "\N{EM DASH}"
    return f"{float(value):.{decimals}f}"


def plot_metrics_table(table: pd.DataFrame, output_stem: Path) -> None:
    """Render Table 1 using a compact IEEE/booktabs-inspired layout."""
    configure_publication_style()
    source_rank = {"Stonefish": 0, "Gazebo": 1}
    display = table.copy()
    display["_source_rank"] = display["source"].map(source_rank).fillna(99)
    display = display.sort_values(["pitch_deg", "_source_rank"]).reset_index(drop=True)

    headers = [
        "Source",
        r"$\theta$ ($^\circ$)",
        r"$z_{0.99}$" + "\n(m)",
        r"$z_{0.50}$" + "\n(m)",
        r"$z_{0.01}$" + "\n(m)",
        r"$\Delta z$" + "\n(m)",
        r"$F_{B,\max}$" + "\n(N)",
        r"$z_{\mathrm{eq}}$" + "\n(m)",
        r"$S_{dF/dz}$" + "\n(N/m)",
    ]
    columns = [
        "source",
        "pitch_deg",
        "z_at_ratio_0.99",
        "z_at_ratio_0.50",
        "z_at_ratio_0.01",
        "transition_width_z",
        "max_buoyancy_N",
        "equilibrium_z",
        "smoothness_dFdz",
    ]
    decimals = [None, 0, 6, 6, 6, 6, 6, 6, 6]
    widths = np.array([1.05, 0.55, 0.84, 0.84, 0.84, 0.74, 0.90, 0.82, 1.10])
    widths = widths / widths.sum()
    edges = np.concatenate(([0.0], np.cumsum(widths)))
    centers = 0.5 * (edges[:-1] + edges[1:])

    row_count = len(display)
    fig_height = max(2.05, 0.95 + 0.20 * row_count)
    fig, ax = plt.subplots(figsize=(7.16, fig_height))
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")

    title_y = 0.965
    table_top = 0.85
    header_bottom = 0.70
    table_bottom = 0.1
    row_height = (header_bottom - table_bottom) / max(row_count, 1)

    ax.text(
        0.5,
        title_y,
        "TABLE 1. Static sweep consistency metrics",
        ha="center",
        va="center",
        fontsize=9.0,
        fontweight="semibold",
    )

    for x, label in zip(centers, headers):
        ax.text(
            x,
            0.5 * (table_top + header_bottom),
            label,
            ha="center",
            va="center",
            fontsize=7.0,
            fontweight="semibold",
            linespacing=1.15,
        )

    pitch_values = list(display["pitch_deg"])
    for row_index, (_, row) in enumerate(display.iterrows()):
        row_top = header_bottom - row_index * row_height
        row_bottom = row_top - row_height
        y = 0.5 * (row_top + row_bottom)
        for column_index, (x, column, precision) in enumerate(
            zip(centers, columns, decimals)
        ):
            value = row[column]
            text = str(value) if precision is None else format_table_value(
                value, precision
            )
            color = "#000000"
            weight = "semibold" if column == "source" else "normal"
            ax.text(
                x,
                y,
                text,
                ha="center",
                va="center",
                fontsize=7.0,
                color=color,
                fontweight=weight,
            )

        is_group_end = (
            row_index == row_count - 1
            or pitch_values[row_index + 1] != pitch_values[row_index]
        )
        if is_group_end and row_index != row_count - 1:
            ax.hlines(row_bottom, 0.0, 1.0, color="#999999", linewidth=0.35)

    # Booktabs-style rules: no vertical grid, only structural horizontal rules.
    ax.hlines(table_top, 0.0, 1.0, color="black", linewidth=1.05)
    ax.hlines(header_bottom, 0.0, 1.0, color="black", linewidth=0.72)
    ax.hlines(table_bottom, 0.0, 1.0, color="black", linewidth=1.05)
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.025, top=0.985)
    save_publication_figure(fig, output_stem)


def plot_pitch(frame: pd.DataFrame, pitch: float, output_stem: Path) -> None:
    specs = [
        (
            "submersion_ratio_body",
            "(a) Body submersion",
            r"Submersion ratio, $r_b$",
        ),
        (
            "buoyancy_z_N",
            r"(b) Vertical buoyancy",
            r"$F_{B,z}$ (N)",
        ),
        (
            "net_force_z_N",
            r"(c) Vertical net force",
            r"$F_{\mathrm{net},z}$ (N)",
        ),
        (
            "center_of_buoyancy_z",
            r"(d) Center of buoyancy",
            r"$z_B$ (m)",
        ),
    ]
    configure_publication_style()
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 4.9), sharex=True)
    local = frame[np.isclose(frame["pitch_deg"], pitch)]
    source_order = [
        source for source in ("Stonefish", "Gazebo")
        if source in set(local["source"])
    ]
    source_order.extend(
        source for source in local["source"].unique()
        if source not in source_order
    )
    for ax, (column, title, ylabel) in zip(axes.flat, specs):
        plotted = False
        for source in source_order:
            group = local[local["source"] == source]
            if column not in group or not group[column].notna().any():
                continue
            group = group.sort_values("z_m")
            style = SOURCE_STYLE.get(
                source,
                {
                    "color": "#4D4D4D",
                    "linestyle": "-.",
                    "linewidth": 1.5,
                    "zorder": 1,
                },
            )
            ax.plot(group["z_m"], group[column], label=source, **style)
            plotted = True
        if not plotted:
            ax.text(
                0.5,
                0.5,
                "Not available",
                transform=ax.transAxes,
                ha="center",
                va="center",
                color="#666666",
            )
        ax.axvline(0.0, color="#555555", linestyle=":", linewidth=0.75, zorder=0)
        if column in {"buoyancy_z_N", "net_force_z_N", "center_of_buoyancy_z"}:
            ax.axhline(0.0, color="#777777", linewidth=0.55, alpha=0.45, zorder=0)
        ax.set_title(title, loc="left", pad=4.0)
        ax.set_ylabel(ylabel)
        ax.grid(True, which="major")
        ax.tick_params(top=True, right=True)
        ax.margins(x=0.0)
    axes[0, 0].set_ylim(-0.035, 1.035)
    for ax in axes[-1]:
        ax.set_xlabel(r"Vertical position, $z$ (m)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(
            handles,
            labels,
            loc="lower center",
            bbox_to_anchor=(0.53, 0.08),
            ncol=2,
            borderaxespad=0.0,
            handlelength=2.8,
            columnspacing=2.2,
            handletextpad=0.8,
            frameon=True,
            fancybox=False,
            framealpha=1.0,
            facecolor="white",
            edgecolor="black",
        )
        legend = fig.legends[-1]
        legend.get_frame().set_linewidth(0.8)
    fig.suptitle(
        rf"Static free-surface sweep ($\theta={pitch:+g}^\circ$)",
        x=0.53,
        y=0.975,
        fontsize=9.5,
        fontweight="semibold",
    )
    fig.subplots_adjust(
        left=0.105,
        right=0.975,
        bottom=0.205,
        top=0.89,
        wspace=0.31,
        hspace=0.31,
    )
    save_publication_figure(fig, output_stem)


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    sources = [normalize(args.gazebo, "Gazebo")]
    if args.stonefish is not None:
        sources.insert(
            0, normalize(args.stonefish, "Stonefish", args.stonefish_z_sign)
        )
    frame = pd.concat(sources, ignore_index=True, sort=False)
    rows = [metrics(group) for _, group in frame.groupby(["source", "pitch_deg"])]
    table = pd.DataFrame(rows).sort_values(["pitch_deg", "source"])
    for _, row in table.iterrows():
        for name in ("z_at_ratio_0.99", "z_at_ratio_0.50", "z_at_ratio_0.01", "equilibrium_z"):
            if pd.isna(row[name]):
                warnings.warn(f"{row['source']} pitch={row['pitch_deg']}: {name} unavailable")
    table_path = args.output_dir / "table1_static_sweep_consistency.csv"
    table.to_csv(table_path, index=False)
    plot_metrics_table(
        table,
        args.output_dir / "table1_static_sweep_consistency",
    )
    for pitch in sorted(frame["pitch_deg"].dropna().unique()):
        suffix = f"{pitch:+07.3f}".replace("+", "p").replace("-", "m").replace(".", "p")
        plot_pitch(
            frame,
            float(pitch),
            args.output_dir / f"fig1_static_sweep_pitch_{suffix}",
        )
    print(
        f"Wrote {table_path} and {frame['pitch_deg'].nunique()} figure set(s) "
        "plus Table 1 (PDF + PNG)"
    )


if __name__ == "__main__":
    main()
