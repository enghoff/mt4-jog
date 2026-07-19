"""Unit tests for envelope summary math (no hardware)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from mt4_jog.envelope import (
    append_sample,
    empty_doc,
    load_doc,
    recompute_summary,
    sample_from_status,
    save_doc,
    undo_last_sample,
)
from mt4_jog.status import Mt4Status, TcpPose


def _sample(
    label: str,
    *,
    x: float,
    y: float,
    z: float,
    j1: float = 0.0,
    j2: float = 90.0,
    j3: float = 10.0,
    j4: float = 0.0,
    s1: int = 0,
    s2: int = 0,
    s3: int = 0,
    s4: int = 0,
) -> dict:
    return {
        "label": label,
        "t": "2026-01-01T00:00:00Z",
        "tcp": {"x": x, "y": y, "z": z, "j4": 0.0, "grip": 140},
        "joints_steps": {"j1": s1, "j2": s2, "j3": s3, "j4": s4},
        "joints_deg": {"j1": j1, "j2": j2, "j3": j3, "j4": j4},
    }


class RecomputeSummaryTests(unittest.TestCase):
    def test_empty(self) -> None:
        s = recompute_summary([])
        self.assertEqual(s["counts"], {"in": 0, "out": 0})
        self.assertIsNone(s["cartesian"]["ground_z_mm"])
        self.assertIsNone(s["joints_deg"]["j2"])

    def test_in_only_drives_ranges(self) -> None:
        samples = [
            _sample("in", x=200, y=0, z=160, j2=80, j3=5, s2=-100, s3=50),
            _sample("in", x=300, y=100, z=150, j2=100, j3=40, s2=200, s3=400),
            # Out sample must not expand the allowed box.
            _sample("out", x=400, y=0, z=100, j2=120, j3=60, s2=999, s3=999),
        ]
        s = recompute_summary(samples)
        self.assertEqual(s["counts"], {"in": 2, "out": 1})
        self.assertEqual(s["joints_deg"]["j2"], {"min_in": 80.0, "max_in": 100.0})
        self.assertEqual(s["joints_steps"]["j2"], {"min_in": -100.0, "max_in": 200.0})
        self.assertAlmostEqual(s["cartesian"]["ground_z_mm"], 150.0)
        self.assertAlmostEqual(s["cartesian"]["max_reach_xy_mm"], (300**2 + 100**2) ** 0.5)
        self.assertAlmostEqual(s["cartesian"]["min_r_xy_mm"], 200.0)
        bbox = s["cartesian"]["in_bbox_mm"]
        self.assertEqual(bbox["xmin"], 200.0)
        self.assertEqual(bbox["xmax"], 300.0)
        self.assertEqual(bbox["zmin"], 150.0)
        self.assertEqual(bbox["zmax"], 160.0)


class DocRoundTripTests(unittest.TestCase):
    def test_append_undo_save_load(self) -> None:
        doc = empty_doc(created="2026-01-01T00:00:00Z")
        a = append_sample(doc, _sample("in", x=220, y=10, z=155, j2=85))
        self.assertEqual(a["id"], 1)
        b = append_sample(doc, _sample("out", x=350, y=0, z=140, j2=110))
        self.assertEqual(b["id"], 2)
        self.assertEqual(doc["summary"]["counts"], {"in": 1, "out": 1})

        removed = undo_last_sample(doc)
        assert removed is not None
        self.assertEqual(removed["id"], 2)
        self.assertEqual(doc["summary"]["counts"], {"in": 1, "out": 0})

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "envelope_samples.json"
            save_doc(path, doc)
            loaded = load_doc(path)
            self.assertEqual(len(loaded["samples"]), 1)
            self.assertEqual(loaded["summary"]["counts"]["in"], 1)
            raw = json.loads(path.read_text(encoding="utf-8"))
            self.assertIn("summary", raw)
            self.assertIn("cartesian", raw["summary"])


class SampleFromStatusTests(unittest.TestCase):
    def test_builds_sample(self) -> None:
        status = Mt4Status(
            homed=True,
            joints={"j1": 0, "j2": -350, "j3": 100, "j4": 0},
            tcp=TcpPose(x=250.0, y=20.0, z=160.0, j4=1.0, grip=140.0, speed=1524),
        )
        sample = sample_from_status(status, "in")
        assert sample is not None
        self.assertEqual(sample["label"], "in")
        self.assertEqual(sample["tcp"]["x"], 250.0)
        self.assertEqual(sample["joints_steps"]["j2"], -350)
        self.assertIn("j2", sample["joints_deg"])

    def test_incomplete_returns_none(self) -> None:
        self.assertIsNone(sample_from_status(Mt4Status(homed=True), "in"))


if __name__ == "__main__":
    unittest.main()
