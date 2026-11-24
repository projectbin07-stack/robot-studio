// Bundled example models. studio_mecanum was written for this project and
// is illustrative, not modelled on a commercial robot. The others are
// unmodified third-party robot descriptions whose licences allow
// redistribution; tests/corpus/SOURCES.md records where each came from.

import { VirtualFs, isTextFile } from "./vfs.js";

/**
 * @typedef {object} ExampleSource
 * @property {string} dir folder in this repository (URL path relative to the site root)
 * @property {string} mount where it appears in the virtual file system
 * @property {string[]} files paths relative to dir
 */
/**
 * @typedef {object} Example
 * @property {string} id
 * @property {string} title
 * @property {string} note shown under the example picker
 * @property {string | null} attribution licence and source, null for our own models
 * @property {string} entry file to open, a path in the virtual file system
 * @property {Record<string, string>} packages package name -> mounted folder (when it differs from package.xml)
 * @property {ExampleSource[]} sources
 */

const TB3 = "examples/third_party/turtlebot3_description";
const RC = "examples/third_party/ros2_control_demos";

/** @type {Example[]} */
export const EXAMPLES = [
  {
    id: "mecanum",
    title: "studio_mecanum: mecanum base with LiDAR, IMU and depth camera (xacro)",
    note: "Written for Robot Studio; illustrative, not a real product. Four xacro files, macros with block parameters, a recursive roller macro, STL and OBJ meshes. Try the arguments below.",
    attribution: null,
    entry: "/studio_mecanum_description/urdf/studio_mecanum.urdf.xacro",
    packages: {},
    sources: [
      {
        dir: "examples/studio_mecanum",
        mount: "/studio_mecanum_description",
        files: [
          "package.xml",
          "urdf/studio_mecanum.urdf.xacro",
          "urdf/inertia.xacro",
          "urdf/mecanum_wheel.xacro",
          "urdf/sensors.xacro",
          "meshes/lidar.stl",
          "meshes/camera.obj",
        ],
      },
    ],
  },
  {
    id: "turtlebot3",
    title: "TurtleBot3 Burger (ROBOTIS, Apache-2.0)",
    note: "Unmodified turtlebot3_description from ROBOTIS-GIT/turtlebot3 (noetic branch): xacro includes via $(find), STL meshes.",
    attribution:
      "turtlebot3_description by ROBOTIS Co., Ltd., Apache License 2.0, https://github.com/ROBOTIS-GIT/turtlebot3 (commit 4ae959e).",
    entry: "/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro",
    packages: {},
    sources: [
      {
        dir: TB3,
        mount: "/turtlebot3_description",
        files: [
          "LICENSE",
          "package.xml",
          "urdf/common_properties.xacro",
          "urdf/turtlebot3_burger.urdf.xacro",
          "urdf/turtlebot3_burger.gazebo.xacro",
          "urdf/turtlebot3_waffle.urdf.xacro",
          "urdf/turtlebot3_waffle.gazebo.xacro",
          "urdf/turtlebot3_waffle_pi.urdf.xacro",
          "urdf/turtlebot3_waffle_pi.gazebo.xacro",
          "meshes/bases/burger_base.stl",
          "meshes/sensors/lds.stl",
          "meshes/wheels/left_tire.stl",
          "meshes/wheels/right_tire.stl",
        ],
      },
    ],
  },
  {
    id: "r2d2",
    title: "R2D2-style tutorial robot (ros/urdf_tutorial, BSD-3-Clause)",
    note: "Unmodified 08-macroed.urdf.xacro from the ROS URDF tutorial: macros, COLLADA (DAE) gripper meshes with textures.",
    attribution:
      "urdf_tutorial by the ROS project contributors, BSD 3-Clause License, https://github.com/ros/urdf_tutorial (commit 050f1e4).",
    entry: "/urdf_tutorial/urdf/08-macroed.urdf.xacro",
    packages: {},
    sources: [
      {
        dir: "examples/third_party/urdf_tutorial",
        mount: "/urdf_tutorial",
        files: [
          "LICENSE",
          "package.xml",
          "urdf/08-macroed.urdf.xacro",
          "meshes/l_finger.dae",
          "meshes/l_finger_color.png",
          "meshes/l_finger_tip.dae",
          "meshes/l_finger_tip_color.png",
        ],
      },
    ],
  },
  {
    id: "rrbot",
    title: "RRBot with ros2_control (ros2_control_demos, Apache-2.0)",
    note: "Unmodified example_1 of ros-controls/ros2_control_demos: two packages, xacro:arg, block parameters and a <ros2_control> tag.",
    attribution:
      "ros2_control_demos by the ros2_control contributors, Apache License 2.0, https://github.com/ros-controls/ros2_control_demos (commit 9f21994).",
    entry: "/ros2_control_demo_example_1/urdf/rrbot.urdf.xacro",
    packages: { ros2_control_demo_example_1: "/ros2_control_demo_example_1" },
    sources: [
      {
        dir: `${RC}/ros2_control_demo_description`,
        mount: "/ros2_control_demo_description",
        files: [
          "package.xml",
          "rrbot/urdf/rrbot_description.urdf.xacro",
          "rrbot/urdf/rrbot.materials.xacro",
        ],
      },
      {
        dir: `${RC}/example_1/description`,
        mount: "/ros2_control_demo_example_1",
        files: ["urdf/rrbot.urdf.xacro", "ros2_control/rrbot.ros2_control.xacro"],
      },
      {
        dir: RC,
        mount: "/licenses/ros2_control_demos",
        files: ["LICENSE", "example_1/package.xml"],
      },
    ],
  },
];

/** @param {string} id */
export const findExample = (id) => EXAMPLES.find((e) => e.id === id) ?? null;

/**
 * Build the virtual file system of an example. Text files are read now,
 * meshes and images when first needed.
 * @param {Example} ex
 * @param {{ text: (url: string) => Promise<string>, bytes: (url: string) => Promise<Uint8Array> }} read
 *   loaders for repository-relative URLs (fetch in the browser, fs in tests)
 * @returns {Promise<VirtualFs>}
 */
export async function exampleFs(ex, read) {
  const fs = new VirtualFs();
  for (const [name, dir] of Object.entries(ex.packages)) fs.setPackage(name, dir);
  const jobs = [];
  for (const src of ex.sources)
    for (const f of src.files) {
      const url = `${src.dir}/${f}`;
      const path = `${src.mount}/${f}`;
      if (isTextFile(path) || f === "LICENSE")
        jobs.push(read.text(url).then((t) => fs.addText(path, t)));
      else fs.addLazy(path, () => read.bytes(url));
    }
  await Promise.all(jobs);
  return fs;
}
