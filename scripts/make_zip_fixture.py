"""Write tests/fixtures/description.zip with Python's zipfile module.

The zip reader is tested against an archive written by an independent
implementation: one stored entry, deflated entries and a directory entry.
Run once; the output is committed.
"""

import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "tests" / "fixtures" / "description.zip"

PACKAGE = """<?xml version="1.0"?>
<package format="3">
  <name>zip_demo</name>
  <version>0.0.0</version>
  <description>Zip reader fixture.</description>
  <maintainer email="noreply@example.com">fixture</maintainer>
  <license>MIT</license>
</package>
"""

ROBOT = """<?xml version="1.0"?>
<robot name="zip_demo" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="s" value="0.1"/>
  <link name="base_link">
    <visual>
      <geometry><mesh filename="package://zip_demo/meshes/cube.stl" scale="${s} ${s} ${s}"/></geometry>
    </visual>
  </link>
</robot>
"""

STL = "solid cube\n" + "".join(
    f"facet normal 0 0 1\nouter loop\nvertex {a}\nvertex {b}\nvertex {c}\nendloop\nendfacet\n"
    for a, b, c in [("0 0 0", "1 0 0", "1 1 0"), ("0 0 0", "1 1 0", "0 1 0")]
) + "endsolid cube\n"


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w") as z:
        z.writestr(zipfile.ZipInfo("zip_demo/"), "")
        z.writestr("zip_demo/package.xml", PACKAGE, compress_type=zipfile.ZIP_DEFLATED)
        z.writestr("zip_demo/urdf/robot.urdf.xacro", ROBOT, compress_type=zipfile.ZIP_DEFLATED)
        z.writestr("zip_demo/meshes/cube.stl", STL, compress_type=zipfile.ZIP_STORED)
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
