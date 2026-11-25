# Robot Studio documentation

Robot Studio runs at <https://projectbin07-stack.github.io/robot-studio/>.
On a first visit it shows what it does and offers a guided example; the
pages below cover each feature. The screenshots under `img/live/` were taken
from the live site by the Pages workflow's `screenshots` job (artifact
`docs-screenshots`) and checked by eye before they were committed.

Validation tiers, everywhere in these pages: **software tests** (unit,
integration and browser tests in CI), **simulation** (output of the studio's
own models, always labelled simulated) and **hardware** (only what was run
on a real robot or read from its bag; so far nothing).

| Page                                           | What it covers                                                                                                    |
| ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| [Getting started](getting-started.md)          | The first-run panel and the four-step guided example                                                              |
| [Model](model.md)                              | Opening URDF / xacro / folders / zips, the 3D view, tree, sliders, findings, links; the performance budget        |
| [Builder](builder.md)                          | Templates, editing links and joints, inertia, exports (URDF, xacro, TOML, ROS 2 package)                          |
| [Simulation](simulation.md)                    | Drive, sensors, estimators, worlds, recordings, cross-checks, performance                                         |
| [ROS 2](ros2.md)                               | Live robot over rosbridge, driving behind the dead-man switch, bags, replays, reaching a robot from the live site |
| [Command line](cli.md)                         | `xacro`, `check`, `new`, `export`, `sim`, `bag`, `replay`, `compare`, `studio`                                    |
| [Keyboard and accessibility](accessibility.md) | Keys, what axe-core found before and after, what is not checked                                                   |
| [xacro support](xacro.md)                      | The xacro subset, compared with the official implementation                                                       |

For contributors: [HANDOFF.md](HANDOFF.md) (state and module map),
[decisions.md](decisions.md), [ROADMAP.md](ROADMAP.md),
[upstream-requests.md](upstream-requests.md), [prior-art.md](prior-art.md).
