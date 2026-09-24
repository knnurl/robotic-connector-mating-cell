---
name: operator-hmi
description: Design rules for operator consoles that command machines (robot cells, test rigs, process panels) - ISA-101 high-performance HMI, IEC 60073 colour meanings, a software STOP that does not pose as an E-stop, stable numerics, pending feedback, colour-blind safety, three text sizes. Use when building or reviewing any GUI that moves hardware or shows machine state, in any toolkit (Qt, Tk, web); these rules override generic web or frontend design guidance wherever the two conflict.
---

# Operator HMI rules

Built from the FR3 Cell Control build (`tools/fr3/cell/`, PySide6),
which applied every rule below and was checked screen by screen. An operator
console is not a website: it exists so that one person can see the machine's
state and stop it. Everything below serves those two jobs.

## The rules

1. **ISA-101 (High Performance HMI).** Use a muted, low-saturation base; a
   dark neutral grey is fine for a dim lab, pure black is not. Saturated
   colour is reserved for two things: abnormal states, and the next step
   the operator can take. Normal, idle and completed elements are neutral,
   so a healthy screen is almost colourless and a fault stands out.
2. **IEC 60073 colour meanings, one meaning per colour.**
   - red: emergency or danger
   - yellow/amber: abnormal or warning
   - green: normal, active and healthy
   - blue: mandatory action (use it for the next available step)
   - grey: neutral, done or blocked

   No colour may carry two meanings. That includes data: a blue plot line
   breaks the rule if blue means "next step".
3. **A software STOP is red, and it is the only red control.** Never style
   it red on yellow: that is the IEC 60204-1 hardware emergency-stop look,
   and it implies a safety rating the button does not have. Say so in its
   tooltip: a software stop, not an E-stop, with the hardware E-stop named
   as the safety function. Opposite-direction actions such as RELEASE or
   RESET must not be red.
4. **Numbers.**
   - Use a monospace or tabular-figure font so values do not jitter.
   - Give each quantity a fixed number of decimals.
   - Always show units.
   - Show data age where it matters (for example "vision 42 ms"), and
     turn it amber when stale.
   - Show a missing value as an em dash, never as `nan` or `None`.
5. **Layout (Fitts's law).** STOP is large and stays in one fixed place in
   every state. Never put opposing actions (HOLD and RELEASE, STOP and
   AUTO-RUN) side by side at the same size. Opening a drawer or a view must
   not move the control buttons.
6. **Feedback.** Acknowledge every press at once with a visible pending
   state, and keep it until the command succeeds or fails. A button never
   looks idle while its command is in flight. Failures surface in the
   banner, not only in the log. A press on a blocked control says why; it
   is never silently ignored.
7. **Colour-blind safety.** Red and green never carry meaning alone. Pair
   every state colour with a shape and a word: ● normal, ▲ warning,
   ■ fault, plus a text value.
8. **Typography.** Use at most three text sizes: banner, section and body.
   Section headers are quiet (small, muted). Live values are the most
   prominent text on screen, so they share the largest size with the
   banner title.

## How to apply (checklist)

**Structure first**
- [ ] The status bar is the same set of chips in every mode and state;
      only values and colours change.
- [ ] Exactly one banner, showing the single highest-priority fault from a
      fixed, documented order, or READY plus the mode.
- [ ] Enable logic is one pure function, state → {control: (enabled,
      reason)}, driven by state rather than by the order of presses and
      unit tested. Banner priority is another pure function.
- [ ] Controls whose preconditions fail are visibly blocked and carry the
      reason in a tooltip.
- [ ] At most one blue control at a time, chosen by a pure
      `next_step(state)`.
- [ ] Machine callbacks never touch widgets. Marshal to the GUI thread
      (Qt queued signals; Tk `after()`). Check it structurally: the ROS-side
      modules must not even import the toolkit.

**Safety**
- [ ] STOP (plus a window-level Esc that works whatever has focus, popups
      included) is always enabled and never waits on a busy flag.
- [ ] Anything that raises speed or force past a threshold needs a confirm
      click. That includes values that carry over from another mode.
- [ ] Settings do not persist between sessions where persistence could
      surprise; start each launch at a low default.

**Verification**
- [ ] Offscreen screenshots of the key states, diffed against baselines.
- [ ] Measure the window's minimum size in every state, including
      worst-case strings, fault states and open drawers. It must fit the
      target screen.
- [ ] Walk every screen against rules 1-8 and report any rule you could
      not meet, with the reason.

## Gotchas (each one happened)

- **ISA-101 versus IEC 60073 on green.** ISA-101 wants healthy things grey;
  IEC 60073 says green means normal. Reconcile by drawing healthy chips
  grey, and reserving green for "a process is running and healthy" (the
  RUNNING/TRACKING activity indicator, an active recording).
- **Amber needs dark ink.** White or light text on an amber banner is
  unreadable. Pick the ink per background.
- **Qt style sheets: changing a property on a parent does not re-polish its
  children.** A rule like `QFrame#banner[level="warn"] QLabel {...}` stays
  stale. Put the property on each child and unpolish/polish it. Also, ID
  selectors (`QLabel#title`) outrank descendant selectors.
- **Disabled Qt widgets swallow clicks silently.** "Soft-disable" instead:
  style the control as blocked but keep it enabled, and make its click
  explain the reason (log it, and raise "PRE-FLIGHT NEEDED" when a torque
  control is pressed too early). That also guarantees the tooltip shows.
- **Esc debounce.** Synthetic key events have timestamp 0, so a debounce on
  timestamps drops them. An application-wide event filter that consumes the
  key already stops Qt propagating it to parents; ignore only
  `isAutoRepeat()`.
- **The minimum window size grows silently.** Long chip values under fault
  ("stale 9999 ms", "USER_STOPPED"), captions that do not wrap and an
  opened drawer each pushed the window past 1440×960. Use short state
  names, wrap captions, and move rarely changing chips to spare space.
- **Reference lines flatten the data.** A 40 N reflex line on a plot of
  3 N readings squashes the trace. Scale to the data and the lowest
  reference, and name the off-scale references with an arrow ("reflex
  40 ↑").
- **Shared sliders across modes.** A speed set to 100 % in position mode
  was written straight into torque mode on activation, bypassing the
  torque confirm. Cap at the threshold on mode entry and offer the confirm.
- **Pending must come from state as well as events.** If a "pending" event
  is missed, the busy command shows as blocked. Derive pending from the
  snapshot's busy command too.
- **Theme libraries are templates.** ttkbootstrap, sv-ttk and Bootstrap
  looks fight ISA-101. Hand-style from a small token set instead.
- **Separate what the operator commands from what another process owns.**
  While a tracking node owns gains and speed, show its values as "in
  force" rather than as pending edits, and lock the controls with a reason.
