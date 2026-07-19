"""Envelope sample JSON: load/save and derived joint/Cartesian summaries.

Samples tagged ``in`` / ``out`` are collected by ``map_envelope.py``. Joint
min/max and Cartesian ground-plane / reach stats are computed from ``in``
samples only; ``out`` points are retained for later validation.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from mt4_jog.kinematics import JointAnglesDeg
from mt4_jog.status import Mt4Status

DEFAULT_ENVELOPE_PATH = Path("envelope_samples.json")
ENVELOPE_VERSION = 1
NOTES = (
    "In = reachable/allowed; out = past limit or forbidden. "
    "Cartesian restriction expected: ground plane (min Z among in)."
)

Label = Literal["in", "out"]
JOINT_KEYS = ("j1", "j2", "j3", "j4")


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def empty_doc(*, created: str | None = None) -> dict[str, Any]:
    ts = created or _utc_now_iso()
    return {
        "version": ENVELOPE_VERSION,
        "created": ts,
        "updated": ts,
        "notes": NOTES,
        "samples": [],
        "summary": recompute_summary([]),
    }


def load_doc(path: Path) -> dict[str, Any]:
    """Load an envelope JSON file, or return a fresh empty document."""
    if not path.is_file():
        return empty_doc()
    with path.open(encoding="utf-8") as f:
        doc = json.load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: expected a JSON object")
    doc.setdefault("version", ENVELOPE_VERSION)
    doc.setdefault("notes", NOTES)
    doc.setdefault("samples", [])
    if "created" not in doc:
        doc["created"] = _utc_now_iso()
    doc["summary"] = recompute_summary(doc["samples"])
    return doc


def save_doc(path: Path, doc: dict[str, Any]) -> None:
    doc = dict(doc)
    doc["updated"] = _utc_now_iso()
    doc["summary"] = recompute_summary(doc.get("samples") or [])
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    tmp.replace(path)


def sample_from_status(status: Mt4Status, label: Label) -> dict[str, Any] | None:
    """Build one sample dict from a ``?`` status reply, or None if incomplete."""
    if status.tcp is None or not status.joints:
        return None
    steps = {k: int(status.joints[k]) for k in JOINT_KEYS if k in status.joints}
    if len(steps) != len(JOINT_KEYS):
        return None
    q = JointAnglesDeg.from_steps(
        (steps["j1"], steps["j2"], steps["j3"], steps["j4"])
    )
    tcp = status.tcp
    return {
        "label": label,
        "t": _utc_now_iso(),
        "tcp": {
            "x": round(tcp.x, 3),
            "y": round(tcp.y, 3),
            "z": round(tcp.z, 3),
            "j4": round(tcp.j4, 3),
            "grip": round(tcp.grip, 1),
        },
        "joints_steps": steps,
        "joints_deg": {
            "j1": round(q.j1, 3),
            "j2": round(q.j2, 3),
            "j3": round(q.j3, 3),
            "j4": round(q.j4, 3),
        },
    }


def append_sample(doc: dict[str, Any], sample: dict[str, Any]) -> dict[str, Any]:
    """Assign the next id, append, and refresh summary. Returns the sample."""
    samples = doc.setdefault("samples", [])
    next_id = 1
    if samples:
        next_id = max(int(s.get("id", 0)) for s in samples) + 1
    sample = dict(sample)
    sample["id"] = next_id
    samples.append(sample)
    doc["summary"] = recompute_summary(samples)
    return sample


def undo_last_sample(doc: dict[str, Any]) -> dict[str, Any] | None:
    """Pop the last sample; return it, or None if empty."""
    samples = doc.setdefault("samples", [])
    if not samples:
        doc["summary"] = recompute_summary([])
        return None
    removed = samples.pop()
    doc["summary"] = recompute_summary(samples)
    return removed


def _minmax(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {"min_in": min(values), "max_in": max(values)}


def recompute_summary(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Derive joint ranges and Cartesian stats from ``in`` samples only."""
    ins = [s for s in samples if s.get("label") == "in"]
    outs = [s for s in samples if s.get("label") == "out"]

    joints_deg: dict[str, dict[str, float] | None] = {}
    joints_steps: dict[str, dict[str, float] | None] = {}
    for key in JOINT_KEYS:
        joints_deg[key] = _minmax(
            [float(s["joints_deg"][key]) for s in ins if "joints_deg" in s]
        )
        joints_steps[key] = _minmax(
            [float(s["joints_steps"][key]) for s in ins if "joints_steps" in s]
        )

    cartesian: dict[str, Any] = {
        "ground_z_mm": None,
        "max_reach_xy_mm": None,
        "min_r_xy_mm": None,
        "in_bbox_mm": None,
    }
    if ins:
        xs = [float(s["tcp"]["x"]) for s in ins]
        ys = [float(s["tcp"]["y"]) for s in ins]
        zs = [float(s["tcp"]["z"]) for s in ins]
        rs = [math.hypot(x, y) for x, y in zip(xs, ys)]
        cartesian["ground_z_mm"] = min(zs)
        cartesian["max_reach_xy_mm"] = max(rs)
        cartesian["min_r_xy_mm"] = min(rs)
        cartesian["in_bbox_mm"] = {
            "xmin": min(xs),
            "xmax": max(xs),
            "ymin": min(ys),
            "ymax": max(ys),
            "zmin": min(zs),
            "zmax": max(zs),
        }

    return {
        "counts": {"in": len(ins), "out": len(outs)},
        "joints_deg": joints_deg,
        "joints_steps": joints_steps,
        "cartesian": cartesian,
    }


def format_sample_line(sample: dict[str, Any], *, verb: str = "recorded") -> str:
    tcp = sample["tcp"]
    label = sample["label"]
    sid = sample.get("id", "?")
    return (
        f"{verb} {label} #{sid}: "
        f"tcp ({tcp['x']:.1f}, {tcp['y']:.1f}, {tcp['z']:.1f})  "
        f"j2={sample['joints_deg']['j2']:.1f}° j3={sample['joints_deg']['j3']:.1f}°"
    )


def format_counts(doc: dict[str, Any]) -> str:
    c = doc.get("summary", {}).get("counts", {})
    return f"in={c.get('in', 0)} out={c.get('out', 0)}"
