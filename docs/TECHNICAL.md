# amt-raw technical architecture

This document describes the major implementation concepts in `amt-raw`. It is
intended for developers and technically curious users who want to understand the
state machines and data flow without reconstructing them from the source one
function at a time.

The end-user overview and installation instructions are in the project
[`README.md`](../README.md).

## Design goals and invariants

Several design choices constrain the whole project:

1. **The physical Magic Trackpad is not EVIOCGRABed.** `amt-raw` must coexist
   with independent gesture readers such as Touchégg.
2. **Contact classification uses real physical samples.** Predicted or
   interpolated pointer positions never feed tap, gesture, pressure, lift-off,
   or finger-count decisions.
3. **Pointer resampling happens after gain/precision processing.** Synthetic
   high-rate deltas are generated in logical pointer space rather than being fed
   back through a speed-dependent acceleration curve.
4. **Synthetic motion must preserve total logical displacement.** Fractional
   REL rounding is accumulated instead of discarded, and the preferred
   interpolation curve is anchored exactly to measured logical positions.
5. **Desktop routing is separate from the root daemon.** The system process
   owns raw input/uinput; a user-session helper owns compositor-specific device
   enable/disable policy.
6. **Normal pointer motion is not generally smoothed.** Filtering is narrowly
   scoped to known problems such as stationary coordinate wobble, precision
   speed estimation, and lift-off release slip.

## Process architecture

`amt-raw` consists of a root/system daemon plus an optional per-user desktop
routing helper.

```text
                        +----------------------------+
                        | external gesture recognizer|
                        | (Touchégg, etc.)            |
                        +-------------^--------------+
                                      |
                                      | physical evdev remains readable
                                      |
+----------------------+              |
| Apple Magic Trackpad |--------------+
| USB 05ac:0324        |
| BT  004c:0324        |
+----------+-----------+
           |
           v
+----------------------+       root / system service
| source.py            |
| Type-B MT -> RawFrame|
+----------+-----------+
           |
           v
+----------------------+
| selected backend     |
| native/touchpad/ptr  |
+----------+-----------+
           |
           v
+----------------------+       uinput
| AMT Raw ... devices  |
+----------+-----------+
           |
           v
+----------------------+       user session
| desktop compositor   |<------ KDE/GNOME/Cinnamon helper
+----------------------+
```

The routing helper never tells the root daemon how to interpret contacts. It
only prevents the desktop compositor from simultaneously consuming the physical
trackpad and the replacement device.

## Physical input decoding

Linux `hid-magicmouse` already exposes the USB-C Magic Trackpad as a Type-B
multitouch evdev device on both USB and Bluetooth. The transports use different
HID report formats internally, but the kernel normalizes them into the same evdev
slot/position/pressure/orientation/button model before amt-raw sees them.
`amt_raw/source.py` therefore reconstructs the same slot state and emits the same
`RawFrame` at each `SYN_REPORT` regardless of transport.

Source discovery accepts USB `05ac:0324` and Bluetooth `004c:0324`. USB discovery
prefers interface 01 when available; Bluetooth normally exposes one evdev MT
device and is selected by identity plus required multitouch capabilities.

Conceptually:

```python
@dataclass(frozen=True)
class Touch:
    finger_id: int
    x: int
    y: int
    pressure: int
    major: int
    minor: int
    orientation: int

@dataclass(frozen=True)
class RawFrame:
    timestamp: int
    touches: tuple[Touch, ...]
```

Two time domains are intentionally retained:

- the physical frame timestamp is used for input semantics such as tap duration
  and physical sample intervals;
- `time.monotonic_ns()` is used for host scheduling, output clocks, deadlines,
  timers, and session-level timing.

The configured `AMT_HARDWARE_REPORT_RATE_HZ` is only an initialization request.
The undocumented `0x99/0xf2` control is used only on USB; Bluetooth leaves its
transport cadence untouched. After source initialization, measured report timing
is authoritative on both transports.

## Pointer-mode pipeline

The pointer backend has a strict ordering:

```text
physical RawFrame
      |
      v
button-area contact classification
      |
      v
lift-off jitter filter
      |
      v
finger-count / tap / gesture arbitration
      |
      +---- 2 fingers ----> high-resolution scroll state machine
      |
      +---- 3+ fingers ---> suppress local pointer/scroll output
      |
      v
one-finger positional hysteresis
      |
      v
raw speed estimate
      |
      v
precision gain + normal gain
      |
      v
continuous logical dx/dy
      |
      v
fixed-rate pointer resampler
      |
      v
fractional integerization
      |
      v
REL_X / REL_Y
```

The key invariant is that the resampler sees only already accepted logical
motion. It cannot change whether a physical sample counted as a tap, drag,
gesture, or release.

## Preferred pointer resampling: delayed interpolation

### Why an independent output clock

The hardware input clock and desired pointer presentation clock are unrelated.
For example, tested hardware reports around 9–10 ms apart when 120 Hz is
requested, while a 120 Hz output clock is 8.333 ms and a 240 Hz output clock is
4.167 ms.

Therefore the resampler maintains its own absolute monotonic schedule rather
than emitting synthetic events relative to each arriving physical report.
Missed output deadlines are skipped rather than replayed in bursts.

### Measured-cadence delay

`interpolated` mode intentionally evaluates a known past trajectory rather than
predicting an unknown future one. The presentation time is approximately one
measured physical interval behind real time:

```text
physical:      P0---------P1---------P2---------P3
                              newest known ^

output clock:             evaluate known segment here
                          <---- delay ---->
```

The delay is not calculated from the configured hardware-rate request. The
resampler keeps a short history of actual measured input intervals and uses the
recent maximum. This absorbs normal 9/10 ms or 16/17 ms quantization while
avoiding a nominal-Hz assumption.

If the estimated delay grows, presentation time is never allowed to move
backwards; output temporarily holds until the delayed clock catches up.

### Position-anchored velocity interpolation

Each fully known segment has endpoints `P0` and `P1`. The current and previous
physical segments provide interval-average velocity estimates. From their
change, the resampler estimates a constant acceleration across the current
segment.

Ordinary linear position interpolation is:

```text
P(u) = P0 + (P1 - P0) * u
```

with `u` from 0 to 1.

For the smooth curve, the implementation adds a correction derived from
integrating a linearly changing velocity:

```text
P(u) = P0 + d*u + 0.5*a*T^2*(u^2 - u)
```

where:

- `d = P1 - P0`
- `T` is the physical segment duration
- `a` is estimated from the previous and current interval-average velocities

The correction term is zero at both ends:

```text
u = 0  -> correction = 0
u = 1  -> correction = 0
```

Therefore every reconstructed segment begins and ends at the exact measured
logical positions. There is no prediction debt to repay and no possibility that
resampling changes long-term pointer gain.

Starts, stops, reversals, or an estimated endpoint tangent that would point
backward fall back to ordinary position interpolation for that segment.

### Fractional REL output

The trajectory exists in floating-point logical coordinates, while Linux
`REL_X/REL_Y` events are integers. Rounding error is carried forward:

```text
float delta + previous fraction
             |
             +--> integer REL event
             |
             +--> leftover fraction saved for next tick
```

This preserves total displacement over time.

### Causal alternatives

Two lower-latency modes remain available:

- `latest`: constant-velocity extrapolation from the newest measured segment;
- `adaptive`: the same causal architecture with a conservative short-history
  acceleration estimate when consistency gates permit it.

Both are bounded by the most recent measured physical interval and an absolute
20 ms lookahead ceiling. They are useful references and may suit users who
prefer minimum latency over the extremely even spatial pacing of delayed
interpolation.

## Pointer gain and precision

The physical one-finger delta is converted into a raw speed using the physical
sample interval. `AMT_POINTER_GAIN` is the normal/high-speed scale.

The precision curve uses a filtered **speed estimate**, not filtered position.
Below `AMT_POINTER_PRECISION_SPEED`, the effective gain is:

```text
normal_gain * precision_gain
```

Between the configured speed and transition values, gain blends smoothly toward
normal. At or above `AMT_POINTER_PRECISION_TRANSITION`, normal gain applies.

This distinction is important: the speed filter stabilizes which gain is
selected while leaving the physical path itself unsmoothed.

## Positional hysteresis

### One-finger pointer motion

A small absolute-coordinate deadband suppresses stationary sensor wobble. The
accepted coordinate remains fixed while the raw coordinate stays within the
margin. Once it leaves the margin, the accepted coordinate follows at the edge
of the deadband rather than jumping to the raw point.

That gives a small initial noise margin without quantizing sustained one-unit
movement.

### Two-finger scrolling

The same idea is applied **per finger before centroid averaging**. Filtering only
the centroid allows independent finger noise to combine into a random walk;
filtering each tracking ID first suppresses that noise at the source while
preserving coherent slow motion.

## Tap, double-tap, and tap-and-drag state machine

### Landing grace and tap anchoring

A new one-finger contact is initially ambiguous: a second finger may still land,
and the first fingertip's reported center may move substantially while the pad
flattens against the surface.

For the first `AMT_POINTER_MULTITOUCH_ENTRY_GRACE_MS`:

- visible pointer motion is suppressed by continuously re-anchoring the pointer;
- raw landing travel is tracked separately;
- if the contact survives through the grace period, the tap anchor is
  re-centered at the settled position;
- only subsequent motion uses the normal tight `AMT_POINTER_TAP_MOVE_UNITS`
  threshold.

A very short contact that ends before re-centering instead uses the much larger
`AMT_POINTER_TAP_LANDING_MOVE_UNITS` guard. This prevents hidden fingertip
settling from causing missed taps without making the normal post-settle tap
threshold permissive.

### Physical tap duration

Tap duration is measured from the trackpad's physical frame timestamps and uses
the last frame in which the contact was actually active. The later
`TRACKING_ID=-1` release report does not add an extra report interval to a
borderline tap.

Host monotonic time remains appropriate for host-side deadlines such as the
tap-and-drag retouch window.

### Hold the release, not the first click

A recognized tap does not delay the button press. At tap resolution, the engine
emits:

```text
BTN_LEFT down
```

and delays only the corresponding `BTN_LEFT up` for
`AMT_POINTER_TAP_DRAG_WINDOW_MS`.

If no retouch arrives, a timer releases the button and the result is an ordinary
single click.

If a retouch arrives inside the window, the held first press is retained and no
second press is emitted yet. This avoids a desktop recognizing a double-click
before it is known whether the user intended to drag.

The retouch then has two outcomes:

```text
retouch releases inside slop
    -> release first held click
    -> emit second click

retouch moves past activation distance
or remains down past hold threshold
    -> keep BTN_LEFT held
    -> arm pointer motion as tap-and-drag
    -> release BTN_LEFT when finger lifts
```

The distance and time promotion paths are both important: the distance threshold
rejects ordinary retouch jitter, while the hold threshold lets a very slow
rolling drag start without leaving the pointer frozen indefinitely.

## Two-finger tap versus scroll

Two fingers get a longer landing/arbitration window than ordinary one-finger
entry grace. During that window:

- scroll output is held;
- the scroll anchors follow the contacts so landing movement is not later
  replayed as a jump;
- displacement from the original landing points is still measured;
- decisive motion cancels the tap candidate and starts scrolling immediately.

If both fingers lift without exceeding tap limits, pointer mode emits right
click. Once scrolling commits, the tap candidate is cancelled permanently for
that gesture.

## Three-or-more-finger gestures

When 3+ active non-button-area contacts are present, pointer mode suppresses
local pointer/scroll interpretation until the gesture resolves. This prevents a
3- or 4-finger gesture from leaking partial one/two-finger pointer motion while
fingers land and lift asynchronously.

An exactly three-finger session that remains within tap limits can emit middle
click. Larger motion remains available to external gesture readers through the
unchanged physical evdev stream.

## Lift-off jitter suppression

Release slip is handled separately from ordinary motion smoothing.

For each physical tracking ID, `LiftOffJitterFilter` keeps recent pressure and
raw-path history. A release candidate may engage when either pressure reaches
zero after positive pressure or a sufficiently long monotonic pressure fall
enters the low-pressure region.

A second gate checks recent **path speed**, not net displacement. Using path
length matters because a contact can wobble back toward its starting point while
still moving substantially.

When suppression engages:

```text
accepted physical position ---- anchor
                                  |
low/falling-pressure samples ----+---- coordinates frozen here
```

If the same tracking ID rebounds to `AMT_LIFTOFF_RECOVERY_PRESSURE`, or survives
beyond the release-tail timeout, the candidate is cancelled. An internal offset
keeps recovery from producing a catch-up jump.

Active motion above `AMT_LIFTOFF_MAX_PATH_SPEED` bypasses suppression even when
pressure falls, avoiding false catches during fast/light-pressure movement.

## High-resolution wheel output

Linux defines 120 `REL_WHEEL_HI_RES` units as one logical wheel detent. `amt-raw`
converts filtered two-finger displacement directly into fractional v120 motion.

Fractional conversion error is retained across reports. High-resolution values
are emitted immediately; legacy `REL_WHEEL`/`REL_HWHEEL` events are generated
only when the accumulated high-resolution total crosses a full 120-unit
boundary.

```text
raw touch delta
      |
      v
fractional v120 accumulator
      |
      +----> REL_WHEEL_HI_RES every fine step
      |
      +----> REL_WHEEL when +/-120 boundary is crossed
```

## Scroll-direction ownership

Scroll direction is deliberately owned by exactly one layer. When amt-raw emits
relative wheel events, `AMT_NATURAL_SCROLL` controls both vertical and horizontal
direction before high-resolution integerization and before kinetic velocity is
recorded. The kinetic coast therefore inherits the same sign automatically.

Conceptually:

```text
physical two-finger delta
        |
        v
centroid / hysteresis / axis arbitration
        |
        v
traditional wheel sign
        |
        +---- AMT_NATURAL_SCROLL=on ----> reverse both axes
        |
        v
high-resolution wheel output + kinetic history
```

This applies in pointer mode and to the `AMT Raw Touchpad Scroll` / `Kinetic`
companions in touchpad mode. It does not apply in native mode because no wheel
stream is synthesized there.

Desktop-side inversion is kept neutral for synthetic devices whenever the
public desktop API permits that. KDE's helper can set per-device
`naturalScroll=false` on the AMT wheel-producing devices while leaving the
physical trackpad untouched. GNOME and Cinnamon expose mouse natural scrolling
through a global setting, so their helpers intentionally do not rewrite it; the
README instructs users to keep that global mouse inversion disabled if
`AMT_NATURAL_SCROLL` should be authoritative.

## Kinetic scrolling

During live scrolling, recent signed raw displacement is recorded in a bounded
history. Release velocity is estimated from the last meaningful motion rather
than blindly anchoring the history window at finger lift; the Magic Trackpad can
spend several reports flattening/lifting after useful motion has already ended.

If the release speed exceeds `AMT_KINETIC_START_VELOCITY`, a small worker emits
an exponentially decaying sequence of high-resolution wheel events until:

- velocity falls below `AMT_KINETIC_STOP_VELOCITY`,
- `AMT_KINETIC_MAX_MS` is reached, or
- new pointer/scroll activity cancels the coast.

The total target coast distance is kept compatible with the logical detent
model, while the high-resolution implementation subdivides that distance into
small events for smooth presentation.

## Button-area contact isolation

Button-area classification is **sticky by tracking ID**. A contact that begins
inside the configured bottom strip remains a button contact until that tracking
ID ends, even if the finger later moves above the visual strip.

Reserved button contacts are excluded from pointer, scroll, and gesture finger
counts. This makes arrangements such as "thumb holds a button while another
finger moves the pointer" possible without the thumb contaminating motion.

In pointer mode with physical clicks disabled, a quick button-zone touch becomes
a click. A contact that remains down for a short hold, or is joined by an active
pointer finger, becomes a true held button before pointer motion is emitted.

## Touchpad mode

Touchpad mode clones accepted multitouch contacts into `AMT Raw Touchpad`. The
virtual touchpad deliberately uses the Apple USB `05ac:0324` identity even when
the physical source is connected over Bluetooth. This keeps libinput quirk
selection stable across transports, while source autodetection excludes all
device names beginning with `AMT Raw` to prevent a feedback loop.

The absolute touchpad continues to own normal libinput pointer/tap/gesture
semantics. Live scrolling is intentionally separated:

```text
AMT Raw Touchpad
    -> absolute MT pointer/tap/gestures

AMT Raw Touchpad Scroll
    -> live REL_WHEEL_HI_RES scrolling

AMT Raw Touchpad Kinetic
    -> post-release kinetic coast only
```

The scroll engine reuses the pointer backend's two-finger hysteresis,
tap-vs-scroll arbitration, high-resolution conversion, and kinetic physics. The
separate kinetic companion keeps live finger scrolling and post-release coast
independent.

## Desktop routing helpers

### KDE Plasma

KWin exposes per-device input objects over the user session D-Bus. The KDE
helper follows the actually live `AMT Raw` uinput devices and changes only
KWin's per-device `enabled` property.

Fail-safe ordering is:

```text
replacement appears
    -> enable replacement
    -> then disable physical Magic Trackpad in KWin

replacement disappears / daemon stops
    -> re-enable physical Magic Trackpad
```

The physical kernel evdev device remains available to other processes.

For touchpad mode the helper also disables KWin/libinput native two-finger/edge
scrolling on `AMT Raw Touchpad`, because live scrolling is owned by the separate
`AMT Raw Touchpad Scroll` device.

For every synthetic AMT device that actually emits wheel events, the helper also
reconciles KWin's `naturalScroll` property to `false`. This prevents the
compositor from applying a second direction inversion after amt-raw has already
applied `AMT_NATURAL_SCROLL`. The physical device's property is never changed.

### GNOME and Cinnamon

Their public settings expose touchpad `send-events` as a global logical-touchpad
setting rather than a reliable per-device switch. The respective helper therefore
supports pointer mode only:

```text
physical + AMT Raw Pointer present
    -> save exact current send-events value
    -> set send-events=disabled

pointer routing no longer needed
    -> restore exact saved value
```

The saved value is persisted under the user's XDG state directory so a later
helper start can repair state after an interrupted session.

Because the setting is global, other desktop-managed touchpads are also disabled
while pointer mode is active. Pretending this can safely route touchpad mode
would disable `AMT Raw Touchpad` too, so that combination is explicitly left
unsupported.

Their mouse natural-scrolling preferences are global too. The helpers therefore
do not alter them: doing so would unexpectedly change unrelated mice. In
pointer mode, users who want amt-raw to own scroll direction should keep the
desktop's global mouse natural-scroll inversion disabled and set their Magic
Trackpad preference with `AMT_NATURAL_SCROLL`.

## Source tree

```text
amt-raw.py                    CLI and argument validation
amt-raw.service               system/root daemon unit
amt-raw.sysconfig             end-user configuration
amt-raw-kde-sync.py           KWin per-device routing helper
amt-raw-gnome-sync.py         GNOME pointer-mode routing helper
amt-raw-cinnamon-sync.py      Cinnamon pointer-mode routing helper
amt_raw/
  frames.py                   immutable physical frame structures
  source.py                   evdev discovery, MT decoding, hardware rate setup
  runner.py                   reconnect/session loop
  liftoff.py                  release-slip filter
  clicks.py                   hardware click/button-area mapping
  pointer.py                  pointer/tap/scroll/kinetic state machines
  resample.py                 fixed-rate pointer trajectory renderer
  touchpad.py                 virtual MT touchpad + scroll companions
  native.py                   baseline backend
  __init__.py                 package version
tools/
  amt-liftoff-capture.py      development capture/replay diagnostics
```

## Debugging boundaries

When investigating a visible cursor problem, separate the layers:

```text
physical timestamps
    -> amt-raw logical trajectory
    -> uinput REL event timestamps/spacing
    -> libinput/compositor processing
    -> displayed cursor frames
```

A clean `AMT Raw Pointer` stream does not guarantee that every compositor will
present the hardware cursor at the same cadence. This was demonstrated on a
120 Hz GNOME/Mutter test system: the virtual pointer stream remained evenly
paced while the visible hardware cursor clumped; forcing Mutter's software
cursor path substantially improved presentation without changing `amt-raw`.

That distinction is important before changing the resampling algorithm to fix a
problem that may actually exist downstream.
