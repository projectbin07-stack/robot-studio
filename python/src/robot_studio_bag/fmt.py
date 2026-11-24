"""Number formatting that matches Robot Studio's recording writer (JavaScript ``toFixed``)."""

from __future__ import annotations

import math
import struct
from decimal import ROUND_HALF_UP, Decimal


def fixed(v: float, digits: int) -> str:
    """``v`` with ``digits`` decimals as JavaScript's ``toFixed`` writes it.

    The exact binary value is rounded half away from zero (``toFixed`` picks the larger
    magnitude on a tie, Python's ``format`` the even digit); ``-0.000`` is written ``0.000``;
    NaN and infinities are ``nan``.
    """
    if not math.isfinite(v):
        return "nan"
    q = Decimal(v).quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP)
    text = f"{q:.{digits}f}"
    if text.startswith("-") and set(text[1:]) <= {"0", "."}:
        text = text[1:]
    return text


def float32(v: float) -> float:
    """``v`` rounded to the nearest float32 (what ``Math.fround`` does)."""
    if not math.isfinite(v):
        return v
    try:
        result: float = struct.unpack("<f", struct.pack("<f", v))[0]
    except OverflowError:
        return math.copysign(math.inf, v)
    return result


def short32(v: float) -> str:
    """Shortest decimal text that reads back as the same float32 (for float32 fields)."""
    if not math.isfinite(v):
        return "nan"
    target = float32(v)
    for digits in range(1, 10):
        text = f"{target:.{digits}g}"
        if float32(float(text)) == target:
            return _plain(text)
    return _plain(repr(target))


def _plain(text: str) -> str:
    """No exponent: JSON and CSV readers take either, but plain text is easier to read."""
    if "e" not in text and "E" not in text:
        return text
    d = Decimal(text)
    out = format(d, "f")
    return out
