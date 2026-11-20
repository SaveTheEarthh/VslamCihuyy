#!/usr/bin/env python3
"""Runtime parser for common/rtl/vslam_cfg_pkg.sv as the single parameter source."""

import os
import re
from types import SimpleNamespace

DEFAULT_CFG_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "../rtl/vslam_cfg_pkg.sv")
)

_LOCALPARAM_RE = re.compile(
    r"^\s*localparam\s+(?:(?:int|real|string|logic\s*\[[^\]]+\])\s+)?([A-Za-z_]\w*)\s*=\s*(.+?)\s*;\s*$"
)
_BASED_HEX_RE = re.compile(r"^\d+'h([0-9a-fA-F_]+)$")
_BASED_DEC_RE = re.compile(r"^\d+'d([0-9_]+)$")
_BASED_BIN_RE = re.compile(r"^\d+'b([01_]+)$")
_INT_RE = re.compile(r"^[+-]?[0-9][0-9_]*$")
_FLOAT_RE = re.compile(r"^[+-]?(?:\d[0-9_]*(?:\.[0-9_]*)?|\.[0-9_]+)(?:[eE][+-]?\d+)?$")


def _strip_line_comment(line: str) -> str:
    """Remove trailing // comments while preserving // inside double-quoted strings."""
    in_str = False
    escaped = False
    i = 0
    n = len(line)
    while i < n:
        ch = line[i]
        if escaped:
            escaped = False
        elif ch == "\\" and in_str:
            escaped = True
        elif ch == '"':
            in_str = not in_str
        elif not in_str and ch == "/" and i + 1 < n and line[i + 1] == "/":
            return line[:i]
        i += 1
    return line


def _parse_sv_literal(raw: str):
    """Parse a SystemVerilog string, sized integer, integer, or real literal."""
    s = raw.strip()
    if len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]

    m = _BASED_HEX_RE.match(s)
    if m:
        return int(m.group(1).replace("_", ""), 16)

    m = _BASED_DEC_RE.match(s)
    if m:
        return int(m.group(1).replace("_", ""), 10)

    m = _BASED_BIN_RE.match(s)
    if m:
        return int(m.group(1).replace("_", ""), 2)

    if _INT_RE.match(s):
        return int(s.replace("_", ""), 10)

    if _FLOAT_RE.match(s) and ("." in s or "e" in s or "E" in s):
        return float(s.replace("_", ""))

    raise ValueError(f"Unrecognized SystemVerilog localparam literal: {raw!r}")


class VslamConfig(SimpleNamespace):
    """Configuration namespace supporting attribute, dict-key, and dict export access."""

    def __getitem__(self, key: str):
        return getattr(self, key)

    def as_dict(self) -> dict:
        """Return a shallow dictionary copy of all loaded parameters."""
        return dict(vars(self))


def load_cfg(path: str = DEFAULT_CFG_PATH) -> VslamConfig:
    """Load all localparam definitions from a SystemVerilog configuration package."""
    parsed = {}
    with open(path, "r", encoding="utf-8") as f:
        for lineno, raw_line in enumerate(f, start=1):
            clean = _strip_line_comment(raw_line).strip()
            if not clean:
                continue
            if clean.startswith("localparam"):
                m = _LOCALPARAM_RE.match(clean)
                if not m:
                    raise ValueError(f"{path}:{lineno}: malformed localparam line: {raw_line.rstrip()}")
                name, val_str = m.group(1), m.group(2)
                parsed[name] = _parse_sv_literal(val_str)

    if not parsed:
        raise ValueError(f"{path}: no localparam entries found")
    return VslamConfig(**parsed)


CFG = load_cfg()
