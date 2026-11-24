# Security policy

## Supported versions

Only the latest release and the version deployed on GitHub Pages receive fixes.

## Reporting a vulnerability

Please do not open a public issue for a security problem. Use the private
vulnerability reporting of this repository on GitHub (Security tab, "Report a
vulnerability"):
https://github.com/projectbin07-stack/robot-studio/security/advisories/new

Include what you found, how to reproduce it and the version or commit. This is
a spare-time project maintained by one person, so there is no guaranteed
response time.

## Scope

The web page is static. Apart from its own files (HTML, CSS, JavaScript, the
vendored urdf-inspector and three.js code and the bundled examples) it makes
no network requests and sends no data anywhere. Files, folders and zips you
drop are read inside the browser and never uploaded; textures a COLLADA file
names but that are not among your files are replaced by a built-in pixel, not
fetched. xacro `${}` expressions are parsed and evaluated by Robot Studio's own
interpreter for a fixed Python subset: there is no `eval` or `Function`, names
starting with `__` and attributes starting with `_` are rejected. Model names
and values are inserted as text, never as HTML. URL parameters are matched
against fixed lists or strict patterns; a dropped model is never put in a URL.

The XML readers accept only the five predefined entities and character
references and reject DTD internal subsets, so entity-expansion attacks do not
apply. The zip reader rejects ZIP64, encrypted entries and unknown methods,
and checks sizes and CRC-32. A very large or deeply recursive xacro file can
still make the page slow; it runs only in your own browser tab.

The CLI reads the files you name and the packages it finds next to them,
writes a file only with `-o`, reads the environment for `$(env)` like xacro,
and opens no network connections. `scripts/serve.js` is a development server
bound to 127.0.0.1; do not expose it to a network.

## Not a safety component

A clean report means none of the documented checks fired, not that the model
is correct. Verify joint limits, masses and frames on the real robot.
