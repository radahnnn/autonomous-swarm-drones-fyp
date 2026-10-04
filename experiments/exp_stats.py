"""
Shared statistics helpers for the experiment scripts.

- NUM_SEEDS: seeds per condition (default 20, override with SWARM_NUM_SEEDS).
- ci95(): half-width of the two-sided 95% confidence interval of the mean
  (Student t), so "mean +/- ci95" is a proper confidence interval, not a std-dev.
- record_trials(): decorator that logs every trial call (arguments + scalar results)
  and writes them to experiments/results/<name>_raw.csv at interpreter exit, so every
  reported number can be re-derived from raw per-seed data.
"""

import atexit
import csv
import os
from functools import wraps
from typing import Any, Dict, List

import numpy as np
from scipy import stats as _st

NUM_SEEDS = int(os.environ.get("SWARM_NUM_SEEDS", "20"))
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def ci95(values) -> float:
    """95% CI half-width of the mean (Student t). 0.0 for fewer than 2 samples."""
    x = np.asarray(values, dtype=np.float64)
    n = len(x)
    if n < 2:
        return 0.0
    sem = float(np.std(x, ddof=1)) / np.sqrt(n)
    return float(_st.t.ppf(0.975, n - 1) * sem)


def _flatten(d: Dict[str, Any], prefix: str = "") -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flatten(v, f"{prefix}{k}."))
        elif isinstance(v, (int, float, bool, np.integer, np.floating)):
            out[f"{prefix}{k}"] = float(v)
    return out


def record_trials(name: str):
    """Log each call of a trial function to a raw CSV written at exit."""
    rows: List[Dict[str, Any]] = []

    def _write() -> None:
        if not rows:
            return
        os.makedirs(RESULTS_DIR, exist_ok=True)
        fields: List[str] = []
        for r in rows:
            fields += [k for k in r if k not in fields]
        with open(os.path.join(RESULTS_DIR, f"{name}_raw.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

    atexit.register(_write)

    def deco(fn):
        @wraps(fn)
        def wrapper(*args, **kwargs):
            res = fn(*args, **kwargs)
            row: Dict[str, Any] = {k: v for k, v in kwargs.items() if isinstance(v, (int, float, str, bool))}
            for i, a in enumerate(args):
                if isinstance(a, (int, float, str, bool)):
                    row[f"arg{i}"] = a
            if isinstance(res, dict):
                row.update(_flatten(res))
            rows.append(row)
            return res

        return wrapper

    return deco
