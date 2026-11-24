"""Check ``rosbag-inspect export --arrays`` of Robot Studio's /scan against the recording.

Every beam must be in the CSV, no-return beams as ``inf``, the others equal to the recording's
range rounded to float32 (the LaserScan field type).

    python scripts/check_scan_export.py scan.csv run/scans.jsonl
"""

from __future__ import annotations

import csv
import json
import struct
import sys


def f32(v: float) -> float:
    return float(struct.unpack("<f", struct.pack("<f", v))[0])


def main() -> int:
    with open(sys.argv[1], encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    with open(sys.argv[2], encoding="utf-8") as f:
        scans = [json.loads(line) for line in f.read().splitlines()[1:]]
    assert len(rows) == len(scans), (len(rows), len(scans))
    beams = inf = 0
    for r, s in zip(rows, scans, strict=True):
        got = [float(v) for v in r["ranges"].split(" ")]
        assert len(got) == len(s["ranges"]) == int(r["ranges.length"])
        for g, e in zip(got, s["ranges"], strict=True):
            beams += 1
            if e is None:
                assert g == float("inf"), g
                inf += 1
            else:
                assert g == f32(e), (g, e)
    print(
        f"rosbag-inspect export --arrays: {len(rows)} scans, {beams} beams, all kept; "
        f"{inf} no-return beams are inf; every other beam equals the recording as float32"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
