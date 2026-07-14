"""Unit tests for jog_keyboard's focus-detection helper (no hardware).

Run: python tests/test_jog_keyboard.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jog_keyboard import _pid_shares_ancestry


def test_target_is_this_process():
    parents = {300: 200, 200: 100, 100: 0}
    assert _pid_shares_ancestry(parents, 300, 300) is True


def test_target_is_an_ancestor():
    # terminal(100) -> shell(200) -> python(300): the terminal window itself
    # being foreground should count as this process's terminal being focused.
    parents = {300: 200, 200: 100, 100: 0}
    assert _pid_shares_ancestry(parents, 300, 100) is True


def test_target_is_unrelated_process():
    parents = {300: 200, 200: 100, 100: 0}
    assert _pid_shares_ancestry(parents, 300, 999) is False


def test_cyclic_parent_map_does_not_hang():
    """A malformed/cyclic pid->parent map must not infinite-loop."""
    parents = {1: 2, 2: 1}
    assert _pid_shares_ancestry(parents, 1, 999) is False


def run() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"ok  {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {exc}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(run())
