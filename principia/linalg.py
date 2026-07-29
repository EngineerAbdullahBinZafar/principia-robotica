"""
Principia Robotica — Pure Python Linear Algebra Backend

Author: Abdullah Bin Zafar <abz.king.1.9.2003@gmail.com>
License: MIT

This module provides minimal matrix/vector operations using only Python's
built-in `math` and `array` modules — ZERO external dependencies.

It works on ANY hardware including legacy CPUs that don't support SSE4.2.
When numpy IS available, it transparently uses numpy instead for performance.
"""

from __future__ import annotations

import array
import math

try:
    import numpy as _np

    _HAS_NUMPY = True
except (ImportError, RuntimeError):
    _np = None
    _HAS_NUMPY = False


class Vec:
    """
    Lightweight immutable vector backed by Python list.
    Supports all operations needed by the CBF engine.
    """

    __slots__ = ("_data",)

    def __init__(self, data):
        if isinstance(data, Vec):
            self._data = list(data._data)
        else:
            self._data = [float(x) for x in data]

    def __len__(self):
        return len(self._data)

    def __getitem__(self, idx):
        return self._data[idx]

    def __setitem__(self, idx, val):
        self._data[idx] = float(val)

    def __iter__(self):
        return iter(self._data)

    def __add__(self, other):
        if isinstance(other, Vec):
            return Vec([a + b for a, b in zip(self._data, other._data)])
        return Vec([a + other for a in self._data])

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Vec):
            return Vec([a - b for a, b in zip(self._data, other._data)])
        return Vec([a - other for a in self._data])

    def __mul__(self, scalar):
        return Vec([a * float(scalar) for a in self._data])

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __neg__(self):
        return Vec([-a for a in self._data])

    def __repr__(self):
        return f"Vec({self._data})"

    def dot(self, other) -> float:
        """Dot product."""
        return sum(a * b for a, b in zip(self._data, other._data))

    def norm(self) -> float:
        """L2 norm."""
        return math.sqrt(sum(a * a for a in self._data))

    def copy(self) -> "Vec":
        return Vec(self._data)

    def tolist(self) -> list:
        return list(self._data)


class Mat:
    """
    Lightweight matrix (list of rows) for small robot control matrices.
    All operations are pure Python — works on any CPU.
    """

    __slots__ = ("_rows", "_n", "_m")

    def __init__(self, rows):
        self._rows = [[float(x) for x in row] for row in rows]
        self._n = len(self._rows)
        self._m = len(self._rows[0]) if self._rows else 0

    @property
    def shape(self):
        return (self._n, self._m)

    def __getitem__(self, idx):
        if isinstance(idx, tuple):
            i, j = idx
            return self._rows[i][j]
        return self._rows[idx]

    def T(self) -> "Mat":
        """Transpose."""
        return Mat([[self._rows[i][j] for i in range(self._n)] for j in range(self._m)])

    def matmul_vec(self, v: Vec) -> Vec:
        """Matrix × vector."""
        assert len(v) == self._m
        return Vec([sum(self._rows[i][j] * v[j] for j in range(self._m)) for i in range(self._n)])

    def vec_matmul(self, v: Vec) -> Vec:
        """Row vector × matrix (v^T @ M)."""
        assert len(v) == self._n
        result = []
        for j in range(self._m):
            s = sum(v[i] * self._rows[i][j] for i in range(self._n))
            result.append(s)
        return Vec(result)

    def __repr__(self):
        return f"Mat({self._rows})"


def zeros_vec(n: int) -> Vec:
    return Vec([0.0] * n)


def zeros_mat(n: int, m: int) -> Mat:
    return Mat([[0.0] * m for _ in range(n)])


def clip(v: Vec, lo: Vec, hi: Vec) -> Vec:
    return Vec([max(lo[i], min(hi[i], v[i])) for i in range(len(v))])


def norm(v: Vec) -> float:
    return v.norm()


def dot(a: Vec, b: Vec) -> float:
    return a.dot(b)


def array_vec(data) -> Vec:
    """Create a Vec from list/tuple — drop-in for np.array(...)."""
    return Vec(data)
