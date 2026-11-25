# Keyboard and accessibility

What was checked in milestone 5, how, and what is still not checked.

## Keyboard

Everything in the page can be reached with Tab and operated with the
keyboard; the focused control always shows an outline (2 px, the accent
colour, in both themes).

| Where            | Keys                                                                                                    |
| ---------------- | ------------------------------------------------------------------------------------------------------- |
| Kinematic tree   | one Tab stop; ↑/↓ previous/next entry, ← parent, → first child, Home/End, Enter or Space selects        |
| Joint sliders    | ←/→ (or ↓/↑) one step (1/400 of the range), Page Up/Down, Home/End                                      |
| View toolbar     | Tab between the buttons and check boxes; Enter or Space                                                 |
| Panels           | Enter or Space on a panel's title opens or closes it                                                    |
| Simulation       | W/S or arrows forward and back, A/D turn, Q/E strafe (mecanum), Space stop, when no field has the focus |
| ROS 2 driving    | the same keys, only after the explicit opt-in; letting go or leaving the page stops the robot           |
| Builder          | Ctrl+Z, Ctrl+Y / Ctrl+Shift+Z, Delete removes the selected link                                         |
| Start here panel | Tab between its buttons; Escape closes it                                                               |

The 3D view itself (orbit, pan, pick) needs a pointer; everything it shows
can also be reached from the tree, the Details panel and the sliders.

## How it is checked in CI

The browser job (`ci.yml`, headless Chrome) runs, on every push:

- **axe-core 4.14.0** (vendored, MPL-2.0, `vendor/axe-core`) with the WCAG
  2.1 A and AA rules after the default page in both themes, at phone width,
  the builder, the simulation, the ROS 2 live view against the fake
  rosbridge server, a replay with the comparison, the guided example and
  three keyboard runs. Any violation fails the job; the reports are in the
  `browser-screenshots` artifact (`axe-*.json`).
- **Keyboard only** (`browser-smoke.mjs --keyboard`, real key events over
  DevTools): Tab to the tree, arrows and Enter select a link, ← goes to the
  parent, the arrow keys move a joint slider, Enter on Collision switches
  the view, Tab leaves the toolbar, Enter opens the Simulate panel and
  starts the simulation, the ROS 2 panel takes an address and reaches
  Connect. After each step the focused element must show an outline. Run in
  the light and dark themes and on the R2D2 model.

## What axe-core found, before and after

Measured on the development machine with headless Edge 158 and the same
smoke script, on v0.4.0 (before) and on the milestone 5 code (after);
"nodes" are elements that failed a rule.

| Page state                       | v0.4.0: rules / nodes                                            | after: rules / nodes |
| -------------------------------- | ---------------------------------------------------------------- | -------------------- |
| Default page, light theme        | 2 / 15 (color-contrast 14, label 1)                              | 0 / 0                |
| Default page, dark theme         | 2 / 3 (color-contrast 2, label 1)                                | 0 / 0                |
| Simulation running, light        | 2 / 16 (color-contrast 15, label 1)                              | 0 / 0                |
| Simulation running, dark         | 2 / 5 (color-contrast 4, label 1)                                | 0 / 0                |
| Builder, a wheel selected, light | 2 / 14 (color-contrast 13, label 1)                              | 0 / 0                |
| Replay with comparison, dark     | 3 / 4 (color-contrast 2, label 1, scrollable-region-focusable 1) | 0 / 0                |

What was fixed: the accent (links, pressed buttons) and warning colours and
the purple "simulated" badge were below 4.5:1 in the light theme; white
text on the light blue accent in the dark theme was 2.7:1 (now dark text,
7.0:1); joint types in the tree were drawn at 80 % opacity (3.7:1); the
template's robot name field had no label; the comparison table's scroll
region could not be reached with the keyboard. The keyboard runs also found
that the folder and world-file inputs were unreachable (hidden inputs), and
that closing the simulation left the 3D view empty (the camera stayed where
the robot had driven). All fixed.

## Not checked

- No screen reader has been used (NVDA, JAWS, VoiceOver, TalkBack). The
  tree has WAI-ARIA tree roles and the status lines are live regions, but
  how they are announced has not been heard.
- axe-core checks what it can decide automatically; it cannot judge whether
  a label is meaningful or whether the 3D view's content is conveyed.
- The canvas plots and the 3D view have no text alternative beyond the HUD,
  the tables and the Details panel.
- Colour contrast was checked by axe-core in the page states listed above,
  not in every state (for example every finding severity colour on every
  background).
