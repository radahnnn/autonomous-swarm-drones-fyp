"""
Lightweight Pandas DataFrame drop-in replacement for offline environments.
Zero external dependencies beyond numpy.
"""

import csv
from typing import Dict, List, Optional, Union, Any
import numpy as np


class Series:
    def __init__(self, values: Any):
        self._values = np.asarray(values)

    @property
    def values(self) -> np.ndarray:
        return self._values

    def __array__(self, dtype=None) -> np.ndarray:
        return np.asarray(self._values, dtype=dtype)

    def __len__(self) -> int:
        return len(self._values)

    def __iter__(self):
        return iter(self._values)

    def __getitem__(self, item):
        res = self._values[item]
        if isinstance(res, np.ndarray):
            return Series(res)
        return res

    def dropna(self) -> "Series":
        if np.issubdtype(self._values.dtype, np.floating):
            mask = ~np.isnan(self._values)
            return Series(self._values[mask])
        return Series(self._values.copy())

    def copy(self) -> "Series":
        return Series(self._values.copy())

    # Arithmetic
    def __add__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values + val)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values - val)

    def __rsub__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(val - self._values)

    def __mul__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values * val)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values / val)

    def __rtruediv__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(val / self._values)

    def __pow__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values ** val)

    def __neg__(self):
        return Series(-self._values)

    # Comparisons
    def __eq__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values == val)

    def __ne__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values != val)

    def __lt__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values < val)

    def __gt__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values > val)

    def __le__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values <= val)

    def __ge__(self, other):
        val = other.values if isinstance(other, Series) else other
        return Series(self._values >= val)

    def __repr__(self):
        return f"Series({self._values})"


class DataFrame:
    def __init__(self, data: Optional[Union[List[Dict[str, Any]], Dict[str, Any]]] = None):
        self._data: Dict[str, np.ndarray] = {}
        if data is None:
            return

        if isinstance(data, list):
            if len(data) > 0:
                keys = list(data[0].keys())
                for k in keys:
                    vals = [d[k] for d in data]
                    self._data[k] = np.array(vals)
        elif isinstance(data, dict):
            for k, v in data.items():
                if isinstance(v, Series):
                    self._data[k] = v.values.copy()
                else:
                    self._data[k] = np.asarray(v)

    def __len__(self) -> int:
        if not self._data:
            return 0
        return len(next(iter(self._data.values())))

    @property
    def columns(self) -> List[str]:
        return list(self._data.keys())

    def __getitem__(self, key: Union[str, Series, np.ndarray, list]) -> Any:
        if isinstance(key, str):
            if key not in self._data:
                raise KeyError(key)
            return Series(self._data[key])
        elif isinstance(key, (Series, np.ndarray, list)):
            mask = key.values if isinstance(key, Series) else np.asarray(key)
            if mask.dtype == bool:
                sub_data = {col: arr[mask] for col, arr in self._data.items()}
                return DataFrame(sub_data)
            else:
                # List of column names
                sub_data = {col: self._data[col] for col in key}
                return DataFrame(sub_data)
        raise KeyError(key)

    def __setitem__(self, key: str, value: Any):
        if isinstance(value, Series):
            arr = value.values
        elif isinstance(value, np.ndarray):
            arr = value
        elif isinstance(value, (list, tuple)):
            arr = np.array(value)
        else:
            arr = np.full(len(self), value)
        self._data[key] = arr

    def copy(self) -> "DataFrame":
        return DataFrame({k: v.copy() for k, v in self._data.items()})

    def to_csv(self, filepath: str, index: bool = False):
        if not self._data:
            with open(filepath, "w") as f:
                pass
            return
        keys = list(self._data.keys())
        n = len(self)
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(keys)
            for i in range(n):
                writer.writerow([self._data[k][i] for k in keys])

    def __repr__(self):
        return f"DataFrame(shape=({len(self)}, {len(self._data)}), cols={list(self._data.keys())})"


def read_csv(filepath: str) -> DataFrame:
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        records = []
        for row in reader:
            parsed = {}
            for k, v in row.items():
                if v == "" or v.lower() == "nan":
                    parsed[k] = np.nan
                else:
                    try:
                        parsed[k] = float(v)
                    except ValueError:
                        parsed[k] = v
            records.append(parsed)
    return DataFrame(records)


def merge(df1: DataFrame, df2: DataFrame, on: Union[str, List[str]]) -> DataFrame:
    on_keys = [on] if isinstance(on, str) else list(on)
    # Build lookup index for df2
    df2_lookup = {}
    for i in range(len(df2)):
        key = tuple(
            round(float(df2._data[k][i]), 6) if isinstance(df2._data[k][i], (float, np.floating)) else df2._data[k][i]
            for k in on_keys
        )
        df2_lookup[key] = i

    # Identify non-overlapping columns
    df2_other_cols = [c for c in df2.columns if c not in on_keys]

    merged_records = []
    for i in range(len(df1)):
        key = tuple(
            round(float(df1._data[k][i]), 6) if isinstance(df1._data[k][i], (float, np.floating)) else df1._data[k][i]
            for k in on_keys
        )
        if key in df2_lookup:
            rec = {col: df1._data[col][i] for col in df1.columns}
            j = df2_lookup[key]
            for col in df2_other_cols:
                rec[col] = df2._data[col][j]
            merged_records.append(rec)

    return DataFrame(merged_records)
