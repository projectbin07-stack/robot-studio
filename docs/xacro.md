# xacro in Robot Studio

Robot Studio expands xacro itself, in the browser and in the CLI, with no
Python and no ROS install. The implementation (`src/core/xacro/`) was
written from the xacro documentation and checked against the behaviour of
the official implementation:

- xacro wiki (ROS 2 / current): https://github.com/ros/xacro/wiki
- ROS 1 wiki page, still the most complete feature description: http://wiki.ros.org/xacro
- Official source used as the reference for edge cases: `xacro` 2.1.1 from
  PyPI (`xacro/__init__.py`, `xacro/substitution_args.py`,
  https://github.com/ros/xacro/tree/ros2), BSD licence.

## How it is verified

`tests/corpus/cases.json` lists 38 inputs. For each, the official `xacro`
2.1.1 output (or error) is stored in `tests/corpus/golden/`. CI installs
xacro from PyPI, regenerates every golden and fails if one changes
(`python scripts/xacro_golden.py --check`), then compares Robot Studio's
output with the goldens (`tests/xacro-golden.test.js`).

Result at v0.1.0 (`node scripts/golden-report.js`):

| Inputs                                                                                                                                                                                                                                              | Outcome                                                                                                                                                                                            |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 16 that the official xacro expands: 8 feature fixtures, TurtleBot3 Burger / Waffle / Waffle Pi, the ROS URDF tutorial's `08-macroed.urdf.xacro`, ros2_control_demos RRBot (two argument sets), the bundled studio_mecanum model (two argument sets) | 14 identical to the official output **byte for byte** (comments and layout included); 2 (studio_mecanum, two argument sets) identical except that 4 numbers each differ by 1 ulp in the last digit |
| 21 that the official xacro rejects (undefined names, bad conditions, missing or unused blocks, circular properties, syntax errors, ...)                                                                                                             | Robot Studio rejects all 21                                                                                                                                                                        |
| 1 that the official xacro accepts and Robot Studio rejects on purpose (`lambda`)                                                                                                                                                                    | Rejected with "lambda expressions are not supported"                                                                                                                                               |

The comparison is also canonical (`src/core/canonical.js`: element order,
names, attributes and text must match; comments and attribute order may
differ). Numbers may differ by up to 4 units in the last place, and the
report says when they do: the roller positions of studio_mecanum use
`sin`/`cos`, and Python on Linux (glibc) rounds `sin(4.18879...)` one bit
differently from the JavaScript engine (V8 uses fdlibm), e.g.
`-0.027712812921102028` vs `-0.02771281292110203`. Every other successful
case is byte-identical, which the test also asserts.

The goldens are generated on Linux, as in CI. Regenerating them on Windows
gives the Windows C library's rounding for those 4 numbers, so
`--check` reports the two studio_mecanum goldens as changed there.

This is a corpus of 38 inputs, not a proof. Robot descriptions in the wild
may use something the corpus does not cover; please open an issue with the
input and the official output if you find a difference.

## Supported

| Feature                                        | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| ---------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `xacro:property` with `value`                  | Lazy evaluation by default (evaluated at first use, in its own scope), literal typing as xacro does (`"0.1"` is a float, `"3"` an int, `"true"` a bool, `'quoted'` a string, `"1_0"` stays text)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Property blocks                                | `<xacro:property name="b">...</xacro:property>` inserted with `xacro:insert_block`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Property attributes                            | `default`, `remove`, `scope="parent"` / `"global"`, `lazy_eval="false"`, computed names (`name="${prefix}_r"`)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `xacro:arg`, `$(arg x)`                        | Defaults, overrides from the CLI (`--arg x=1` or `x:=1`) or the app's argument form; args are global and set in document order, as in xacro                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `xacro:macro`                                  | Parameters with `:=` or `=` defaults (evaluated at call time), `^` and `^                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | default`inheritance from the calling scope,`*block`and`**block` parameters, nested and local macros, recursion, redefinition, typed parameters |
| `xacro:insert_block`                           | For block parameters and block properties                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `xacro:if`, `xacro:unless`                     | xacro's rules: `true`/`True`/`false`/`False`, integer strings, or any `${}` value by Python truthiness; anything else is an error, as in xacro (including `"1.0"`)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `xacro:include`                                | Relative to the including file, `$(find pkg)`, `ns="name"` namespaces (`${name.prop}`, `<xacro:name.macro/>`), glob patterns (`*`, `?`, `[...]`), `optional="true"`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `xacro:element`, `xacro:attribute`             | Dynamic element and attribute names                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `xacro:call`                                   | `<xacro:call macro="${name}" .../>`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `xacro:targetNamespace`, `xacro:eval-comments` | As in xacro                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `${...}` expressions                           | A Python expression subset evaluated by our own parser and evaluator (no `eval`, no `Function`): numbers with Python's int/float semantics (`6/3` is `2.0`, `7//2` is `3`, `-7%3` is `2`, ints are arbitrary precision), floats printed like Python (`1e-05`, `2.0`), strings and `%` formatting, `str.format` and common string methods, lists, tuples, `dict(...)`, indexing and slicing, comparisons (chained), `and`/`or`/`not`, `a if c else b`, `in`, `is`                                                                                                                                                                                                                            |
| Names in expressions                           | `True False None`, `list dict map len str float int bool min max round`, `sorted` and `range` (with xacro's deprecation warning), `python.*` (`abs all any sum tuple zip enumerate reversed ord repr divmod sorted range` and the above), every common `math` function and constant both bare and as `math.*` (`pi e tau inf nan sin cos tan asin acos atan atan2 sinh cosh tanh asinh acosh atanh sqrt cbrt exp expm1 log log10 log2 log1p pow fabs floor ceil trunc fmod copysign hypot degrees radians isclose isnan isinf isfinite factorial gcd`), `xacro.arg xacro.tokenize xacro.message xacro.warning xacro.error xacro.fatal xacro.print_location xacro.abs_filename xacro.dotify` |
| `$(...)` substitutions                         | `$(arg)`, `$(find)`, `$(env)` and `$(optenv)` (the CLI reads the shell environment like xacro; the web app has none, so there `$(env X)` fails and `$(optenv X d)` gives `d`), `$(eval ...)`, `$(cwd)`, `$(dirname)`; `$$` escapes                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| Errors                                         | file:line of the element being processed, plus "when instantiating macro" and "included from" lines, like xacro's own trace                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Warnings                                       | Unknown attributes on directives, redefined globals, deprecated direct calls, ignored include children, unmatched globs                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |

## Not supported (honest list)

- **`xacro.load_yaml()`** (and `load_yaml`): there is no YAML reader yet.
  Descriptions that read joint limits or calibration from YAML (for example
  Universal Robots' `ur_description`) stop with "xacro.load_yaml() is not
  supported". Planned; see [ROADMAP.md](ROADMAP.md).
- **Python constructs outside the subset**: `lambda`, list/dict/set
  comprehensions and generator expressions, f-strings, `*args` / `**kwargs`
  in calls, set literals, bytes and complex numbers. The official xacro
  evaluates these with Python's `eval`; Robot Studio rejects them with a
  message naming the construct.
- **Uncommon builtins**: `python.isinstance`, `type`, `vars`, `hash`,
  `filter`, `slice`, `set`, `frozenset`, `complex`; `math` functions not
  listed above (e.g. `math.prod`, `math.dist`, `math.comb`).
- **`map()` returns a list**, not an iterator; `str(map(...))` therefore
  differs from xacro (`list(map(...))`, the usual form, is identical).
- **Large integers beyond 2^53** in float contexts and **last-ulp
  differences in `sin`/`cos`/`exp`**: ints are exact (BigInt), but float
  math uses the JavaScript engine's functions, which may differ from the C
  library in the last bit for some inputs. None of the corpus inputs shows
  a difference.
- **`$(dirname)`**: the PyPI xacro 2.1.1 fails on it ("Cannot substitute
  $(dirname), no file/directory information available"); Robot Studio
  returns the folder of the current file, as the wiki documents.
- **`xacro:attribute` with a non-text value**: the official xacro crashes
  when writing such a document (`argument of type 'float' is not a
container`); Robot Studio converts the value to text.
- **Comment removal before directives** follows xacro's rule for the
  common cases (comments directly above a property, macro, include, arg,
  `if`/`unless` or macro call are dropped) but not its exact handling of
  blank-line markers; comments never affect the robot model.
- **`--deps`, `--root-dir`, `-q/-v`** and other options of the xacro
  command are not offered by `robot-studio xacro`.

## File resolution

- `$(find pkg)` and `package://pkg/...` map to the folder that holds the
  `package.xml` whose `<name>` is `pkg`. In the browser, that is a folder
  you dropped or a folder inside the zip; with no `package.xml`, a
  top-level folder named `pkg` is used. The CLI finds packages next to the
  input file's package (the usual `src/<pkg>` workspace layout) or from
  `--package name=dir`.
- ament_cmake packages often install a source folder into their share
  directory (`install(DIRECTORY description/ ...)`), so `$(find pkg)/urdf/x`
  refers to `<pkg>/description/urdf/x` in the source tree. When a file is
  missing inside a package and exactly one file exists at the same path one
  folder deeper, Robot Studio uses it and reports a `file-fallback` note.
- In the official xacro, `$(find)` goes through `ament_index_python`. The
  golden generator replaces only that lookup (see
  [`tests/corpus/SOURCES.md`](../tests/corpus/SOURCES.md)).
