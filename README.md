# amt-raw

`amt-raw` is a Linux userspace input bridge for the **Apple Magic Trackpad USB-C**
connected over USB or Bluetooth. Its primary `pointer` mode replaces the
desktop's normal
trackpad pointer path with a configurable relative pointer designed for smooth,
precise motion and scrolling while still leaving the physical multitouch device
available to external gesture applications.

The project is currently pre-release software and has been developed and tested
primarily on Fedora with KDE Plasma, GNOME, and Cinnamon.

## Major features

### Pointer mode — default and recommended

Pointer mode is the main reason to install `amt-raw`.

- **Smooth, resampled cursor output at a configurable rate independent of the
  hardware.** The default 120 Hz `interpolated` mode reconstructs a smooth,
  evenly paced pointer trajectory from the measured hardware reports.
- **High-resolution two-finger scrolling** using standard Linux high-resolution
  wheel events, with legacy detent events emitted for compatibility.
- **Per-trackpad natural-scrolling preference** for amt-raw-generated scrolling,
  independent of the physical trackpad's native desktop setting.
- **Configurable kinetic scrolling** so a fast two-finger flick can coast after
  release without application-specific integration.
- **Configurable overall gain/speed control.**
- **Configurable low-speed precision gain**, including speed cutoff and smooth
  transition back to normal gain.
- **Tap, double-tap, and tap-and-drag handling** tuned for the Magic Trackpad.
- **Lift-off jitter suppression** to prevent the cursor from slipping as a
  finger leaves the surface.
- **Works alongside external gesture applications such as Touchégg.** The
  physical device is not `EVIOCGRAB`ed, and 3+ finger sessions are kept out of
  the custom pointer/scroll path.
- **Optional bottom button areas** with independent left/middle/right zones and
  physical-click or touch activation.
- **Configurable thresholds** for motion, taps, lift-off, scrolling, and kinetic
  behavior through `/etc/sysconfig/amt-raw`.

### Touchpad mode

Touchpad mode creates an enhanced virtual multitouch touchpad instead of a
relative pointer.

- Preserves the physical 16-slot multitouch stream for normal libinput touchpad
  pointer/tap/gesture behavior.
- Adds high-resolution two-finger scrolling through a companion scroll device.
- Adds optional kinetic/coasting scrolling after release.
- Keeps the physical device readable by external gesture applications.
- Supports optional bottom physical-click button areas on KDE/Plasma.

Touchpad mode intentionally leaves pointer acceleration/gain policy to libinput
and the desktop environment.

### Native mode

Native mode is a baseline/fallback mode. `amt-raw` opens and initializes the
trackpad but does not create a replacement pointer or touchpad; the stock Linux
input path remains authoritative.

## Supported hardware

The current implementation targets:

- Apple Magic Trackpad USB-C
- USB input ID `05ac:0324`
- Bluetooth input ID `004c:0324`
- Linux `hid-magicmouse` exposing the active multitouch stream as a Type-B
  multitouch evdev device

On USB, automatic discovery prefers the stable `*-if01-event-mouse` path when
present; interface 01 is the active multitouch event stream on tested hardware.
On Bluetooth there is normally a single evdev trackpad node, so discovery falls
back to VID/PID plus multitouch-capability probing. No Bluetooth-specific device
path needs to be configured.

The undocumented `AMT_HARDWARE_REPORT_RATE_HZ` override is currently **USB-only**.
When connected over Bluetooth, amt-raw leaves the device's Bluetooth report
cadence unchanged and the resampler adapts to the cadence actually measured from
the evdev stream.

The trackpad advertises 16 multitouch slots. `amt-raw` does not impose a
four-finger limit on the raw physical stream; limitations in external gesture
recognizers are downstream of `amt-raw`.

## How it fits into the desktop

The system daemon reads the physical evdev stream and creates the selected
uinput replacement device. A small per-user desktop helper prevents the desktop
compositor from also consuming the physical Magic Trackpad at the same time.

```text
physical Magic Trackpad
        |
        +--------------------> external gesture reader (optional)
        |
        v
    amt-raw system daemon
        |
        v
 AMT Raw Pointer / Touchpad
        |
        v
 desktop compositor
```

The daemon deliberately does **not** use `EVIOCGRAB`. Desktop routing is handled
at the compositor/session level instead.

For a deeper explanation of the architecture and algorithms, see
[`docs/TECHNICAL.md`](docs/TECHNICAL.md).

## Requirements

Common requirements:

```bash
sudo dnf install python3-evdev
```

The system daemon needs access to the physical input event device and
`/dev/uinput`; the supplied systemd unit therefore runs system-wide.

Desktop-specific requirements:

- **KDE Plasma:** install Qt 6 qdbus support:

  ```bash
  sudo dnf install qt6-qttools
  ```

  KDE has per-device input controls, so both `pointer` and `touchpad` modes are
  supported by the KDE helper.

- **GNOME:** the included helper uses `gsettings` and supports `pointer` mode.
  GNOME exposes touchpad enable/disable globally rather than per device, so the
  helper intentionally does not route `touchpad` mode.

- **Cinnamon:** the included helper likewise uses `gsettings` and supports
  `pointer` mode. Its public touchpad enable/disable setting is also global.

## Installation

From the extracted repository root:

```bash
sudo install -d /usr/local/libexec/amt-raw
sudo install -m 0755 amt-raw.py /usr/local/libexec/amt-raw/amt-raw.py
sudo install -m 0755 amt-raw-kde-sync.py /usr/local/libexec/amt-raw/amt-raw-kde-sync.py
sudo install -m 0755 amt-raw-gnome-sync.py /usr/local/libexec/amt-raw/amt-raw-gnome-sync.py
sudo install -m 0755 amt-raw-cinnamon-sync.py /usr/local/libexec/amt-raw/amt-raw-cinnamon-sync.py
sudo cp -a amt_raw /usr/local/libexec/amt-raw/

sudo install -m 0644 amt-raw.service /etc/systemd/system/amt-raw.service
sudo install -m 0644 amt-raw.sysconfig /etc/sysconfig/amt-raw
sudo install -m 0644 amt-raw-kde-sync.service /etc/systemd/user/amt-raw-kde-sync.service
sudo install -m 0644 amt-raw-gnome-sync.service /etc/systemd/user/amt-raw-gnome-sync.service
sudo install -m 0644 amt-raw-cinnamon-sync.service /etc/systemd/user/amt-raw-cinnamon-sync.service

# Optional classification fallback for AMT Raw Touchpad:
sudo install -m 0644 99-amt-raw-touchpad.rules /etc/udev/rules.d/99-amt-raw-touchpad.rules
sudo udevadm control --reload-rules

sudo systemctl daemon-reload
systemctl --user daemon-reload
```

The shipped configuration defaults to:

```ini
AMT_MODE=pointer
AMT_HARDWARE_REPORT_RATE_HZ=120
AMT_RESAMPLING_RATE_HZ=120
AMT_RESAMPLING_PREDICTOR=interpolated
AMT_NATURAL_SCROLL=off
```

Before starting the system daemon, enable **exactly one** desktop helper as the
normal desktop user.

### KDE Plasma

```bash
systemctl --user enable --now amt-raw-kde-sync.service
```

### GNOME

```bash
systemctl --user enable --now amt-raw-gnome-sync.service
```

### Cinnamon

```bash
systemctl --user enable --now amt-raw-cinnamon-sync.service
```

Then start the system daemon:

```bash
sudo systemctl enable --now amt-raw.service
```

Useful logs:

```bash
journalctl -u amt-raw.service -f
journalctl --user -u amt-raw-kde-sync.service -f
journalctl --user -u amt-raw-gnome-sync.service -f
journalctl --user -u amt-raw-cinnamon-sync.service -f
```

## Quick configuration

The main configuration file is:

```text
/etc/sysconfig/amt-raw
```

For synthesized scrolling, `amt-raw` owns the Magic Trackpad's scroll-direction
preference in `pointer` and `touchpad` modes:

```ini
AMT_NATURAL_SCROLL=off   # traditional scrolling
# AMT_NATURAL_SCROLL=on  # content follows finger motion
```

In `native` mode this setting is ignored; use the desktop environment's normal
physical-touchpad natural-scrolling setting instead. See **Scroll direction and
desktop settings** below for the required desktop-side configuration.

After changing it:

```bash
sudo systemctl restart amt-raw.service
```

The shipped sysconfig is the detailed configuration reference. Every exposed
variable includes its units, allowed or practical range, and default value.

For the normal pointer-mode setup, the defaults are a good starting point. The
most commonly adjusted controls are:

```ini
AMT_POINTER_GAIN=1.35
AMT_POINTER_PRECISION_GAIN=0.55
AMT_POINTER_PRECISION_SPEED=0.8
AMT_POINTER_PRECISION_TRANSITION=1.5

AMT_POINTER_SCROLL_UNITS_PER_DETENT=66
AMT_KINETIC_DECAY_MS=500
AMT_KINETIC_MAX_MS=1600
```

## Pointer motion and resampling

The preferred `interpolated` resampler evaluates pointer motion on an
independent output clock. Its output rate does not need to match the hardware
report rate:

```ini
AMT_HARDWARE_REPORT_RATE_HZ=120
AMT_RESAMPLING_RATE_HZ=120
AMT_RESAMPLING_PREDICTOR=interpolated
```

The hardware value is only a **request sent during initialization**. On tested
hardware, requesting 120 Hz produces an actual cadence of roughly 110 Hz. The
resampler never assumes the requested rate after initialization; it derives
trajectory timing from the measured physical report timestamps.

Reasonable resampling rates include 60, 120, 144, 165, and 240 Hz. The output
rate may be above or below the measured hardware cadence.

`interpolated` intentionally presents a fully known trajectory approximately
one measured hardware-report interval behind the physical stream. This produces
very even spacing and clean curves without predicting past the newest measured
position. The latency cost is approximately one physical report interval—about
9–10 ms at the trackpad's practical maximum rate, or about 16–17 ms when the
hardware is configured near 60 Hz.

Two causal alternatives remain available:

```ini
AMT_RESAMPLING_PREDICTOR=latest
AMT_RESAMPLING_PREDICTOR=adaptive
```

They reduce the deliberate interpolation delay but can be less spatially smooth,
especially when the hardware report rate is much lower than the output rate.
Set `AMT_RESAMPLING_RATE_HZ=0` to disable pointer resampling entirely.

## Pointer gain and precision control

`AMT_POINTER_GAIN` controls normal pointer speed. The precision controls reduce
effective gain during very slow motion and transition smoothly back to normal
gain as finger speed increases:

```ini
AMT_POINTER_GAIN
AMT_POINTER_PRECISION_GAIN
AMT_POINTER_PRECISION_SPEED
AMT_POINTER_PRECISION_TRANSITION
AMT_POINTER_PRECISION_FILTER_MS
```

The precision filter affects only the speed estimate that selects the gain; it
does not smooth or delay the pointer position itself.

For best results, configure the desktop's handling of `AMT Raw Pointer` as
flat/neutral so the compositor does not add a second acceleration curve.

## Taps, double taps, and tap-and-drag

Pointer mode implements its own one-, two-, and three-finger tap arbitration.
A short landing grace hides the contact-center movement that occurs while a
fingertip settles, after which the tap anchor is re-centered and a tighter
movement threshold applies.

A recognized one-finger tap presses `BTN_LEFT` immediately when the tap is
resolved at release, but briefly defers only the button-up. A quick retouch can
therefore become either a second click or a held tap-and-drag without adding
latency to the initial button press.

Relevant settings:

```ini
AMT_POINTER_MULTITOUCH_ENTRY_GRACE_MS
AMT_POINTER_TWO_FINGER_TAP_GRACE_MS
AMT_POINTER_MULTITOUCH_EXIT_GRACE_MS
AMT_POINTER_TAP_MAX_MS
AMT_POINTER_TAP_MOVE_UNITS
AMT_POINTER_TAP_LANDING_MOVE_UNITS
AMT_POINTER_TAP_DRAG_WINDOW_MS
AMT_POINTER_TAP_DRAG_ACTIVATION_UNITS
AMT_POINTER_TAP_DRAG_HOLD_MS
AMT_POINTER_THREE_FINGER_TAP
```

Two-finger taps generate right-click. An exactly three-finger tap can generate
middle-click. Larger 3+ finger motion remains reserved for external gesture
recognizers.

## Lift-off jitter suppression

The Magic Trackpad can move its reported contact center as pressure collapses at
finger release. Pointer mode detects the characteristic low/falling-pressure
release tail only when recent raw path speed is low, then freezes coordinates
until the contact disappears or clearly recovers.

This prevents release slip without applying a general position smoother to
normal motion.

Relevant settings:

```ini
AMT_LIFTOFF_JITTER_SUPPRESSION
AMT_LIFTOFF_PRESSURE_THRESHOLD
AMT_LIFTOFF_FALL_FRAMES
AMT_LIFTOFF_PRESSURE_DROP
AMT_LIFTOFF_RECOVERY_PRESSURE
AMT_LIFTOFF_RECOVERY_TIMEOUT_MS
AMT_LIFTOFF_MOTION_WINDOW_MS
AMT_LIFTOFF_MAX_PATH_SPEED
```

## High-resolution scrolling

Pointer mode applies a small deadband to each scrolling finger independently,
then computes the two-finger centroid. This suppresses independent stationary
sensor wobble without quantizing slow deliberate motion.

The resulting displacement is converted to standard
`REL_WHEEL_HI_RES`/`REL_HWHEEL_HI_RES` events. Legacy `REL_WHEEL` and
`REL_HWHEEL` events are emitted only when the accumulated high-resolution motion
crosses a full detent, so both modern and legacy applications remain usable.

Relevant settings:

```ini
AMT_POINTER_SCROLL_HYSTERESIS_UNITS
AMT_POINTER_SCROLL_START_UNITS
AMT_POINTER_SCROLL_UNITS_PER_DETENT
AMT_NATURAL_SCROLL
AMT_POINTER_SCROLL_AXIS_LOCK
AMT_POINTER_SCROLL_AXIS_LOCK_RATIO
```

## Scroll direction and desktop settings

`amt-raw` uses one clear ownership rule:

> When `amt-raw` synthesizes wheel events, `amt-raw` owns scroll direction.
> When the physical trackpad is used natively, the desktop owns scroll direction.

This avoids accidental double inversion. `AMT_NATURAL_SCROLL=on` reverses both
vertical and horizontal synthesized scrolling so content follows finger motion.
It applies to pointer mode and to the relative scroll/kinetic companion devices
in touchpad mode. Kinetic scrolling automatically follows the same direction.

| Mode | Scroll producer | Natural-scrolling control |
| --- | --- | --- |
| `pointer` | `AMT Raw Pointer` | `AMT_NATURAL_SCROLL` |
| `touchpad` | `AMT Raw Touchpad Scroll` / `Kinetic` | `AMT_NATURAL_SCROLL` |
| `native` | physical Magic Trackpad | desktop touchpad setting |

For synthesized modes, the desktop should **not** apply a second natural-scroll
inversion to the AMT virtual wheel device. The desktop helpers/settings below
keep that ownership model explicit.

## Kinetic scrolling

When a two-finger scroll is released with sufficient velocity, `amt-raw` can
continue emitting decaying high-resolution wheel motion. Release velocity is
estimated from recent real scroll movement, and the coast stops at the configured
velocity threshold or maximum duration.

Relevant settings:

```ini
AMT_POINTER_KINETIC_SCROLL
AMT_TOUCHPAD_KINETIC_SCROLL
AMT_KINETIC_HISTORY_MS
AMT_KINETIC_START_VELOCITY
AMT_KINETIC_STOP_VELOCITY
AMT_KINETIC_DECAY_MS
AMT_KINETIC_MAX_MS
```

In touchpad mode, disable `AMT_TOUCHPAD_KINETIC_SCROLL` if an application adds
its own finger-scroll inertia and the result double-coasts.

## External gestures

The physical Magic Trackpad is deliberately left readable by other processes.
In pointer mode, `amt-raw` suppresses its own pointer/scroll interpretation when
3 or more active gesture contacts are present, so applications such as Touchégg
can continue reading the physical multitouch stream independently.

This does not extend libinput or Touchégg's own gesture-count capabilities. In
particular, `amt-raw` does not synthesize a separate five-finger gesture
protocol.

## Physical clicks and button areas

The trackpad's hardware/force click can be forwarded normally or suppressed:

```ini
AMT_PHYSICAL_CLICK=on
```

Optional button areas reserve the bottom of the surface as equal
left/middle/right thirds:

```text
+-------------------+-------------------+-------------------+
|       LEFT        |      MIDDLE       |       RIGHT       |
+-------------------+-------------------+-------------------+
              bottom button area
```

Configuration:

```ini
AMT_BUTTON_AREAS
AMT_BUTTON_AREA_HEIGHT_MM
AMT_PHYSICAL_CLICK_RELEASE_PRESSURE
AMT_PHYSICAL_CLICK_RELEASE_DEBOUNCE_FRAMES
```

A contact that starts in the button strip remains classified as a button
contact for its tracking-ID lifetime and is excluded from pointer/scroll gesture
counting. With physical click disabled in pointer mode, a quick touch can act as
a click and a held zone can act as a held button while another finger moves the
pointer.

## Touchpad mode

Select:

```ini
AMT_MODE=touchpad
```

Touchpad mode creates `AMT Raw Touchpad` with the full multitouch stream. It
also creates separate relative companions:

- `AMT Raw Touchpad Scroll` — live high-resolution two-finger scrolling
- `AMT Raw Touchpad Kinetic` — post-release kinetic coast

The split keeps live scrolling and post-release coast independent while the
virtual touchpad continues to provide normal libinput pointer, tap, click, and
gesture semantics.

KDE/Plasma routing supports this mode. The GNOME and Cinnamon helpers currently
support pointer mode only because their public touchpad enable/disable setting is
global and would disable the replacement touchpad together with the physical
one.

## Desktop integration

### KDE Plasma

`amt-raw-kde-sync.service` is required for normal enhanced-mode use on Plasma.
It follows the **actually live** virtual devices rather than the desired mode in
the sysconfig file. The helper enables the replacement first, then disables the
physical Magic Trackpad only inside KWin. If the replacement disappears, it
restores the physical device.

This is important even if pointer motion appears normal without the helper:
letting KWin consume the physical and virtual streams together can produce
conflicting scrolling, taps, or other duplicate behavior.

The helper also forces KWin's per-device `naturalScroll` setting **off** for the
AMT synthetic wheel-producing devices. This prevents KWin from applying a second
inversion after `AMT_NATURAL_SCROLL`. The physical Magic Trackpad's own KDE
natural-scrolling preference is left untouched and remains authoritative in
`native` mode.

The helper requires Qt 6 `qdbus` (`qt6-qttools` on Fedora). A short fast-poll
window after evdev topology changes minimizes overlap during restart/reconnect;
steady-state reconciliation is slower and inexpensive.

Useful check:

```bash
systemctl --user status amt-raw-kde-sync.service
```

### GNOME

`amt-raw-gnome-sync.service` supports pointer mode. When both the physical
Magic Trackpad and `AMT Raw Pointer` are present, it saves the user's existing
GNOME touchpad `send-events` value, sets it to `disabled`, and restores the exact
saved value when routing is no longer needed.

Because GNOME's public setting is global, **other GNOME-managed touchpads are
also disabled while amt-raw pointer mode is active**. The helper logs a warning
when another touchpad is visible. Touchpad mode is intentionally unsupported.

Useful checks:

```bash
gsettings get org.gnome.desktop.peripherals.touchpad send-events
systemctl --user status amt-raw-gnome-sync.service
```

While pointer mode is active, `send-events` should be `'disabled'`.

GNOME's mouse natural-scrolling setting is global, so the helper does not change
it automatically. To let `AMT_NATURAL_SCROLL` be the only inversion applied to
`AMT Raw Pointer`, use neutral mouse scrolling and acceleration in GNOME:

```bash
gsettings set org.gnome.desktop.peripherals.mouse natural-scroll false
gsettings set org.gnome.desktop.peripherals.mouse accel-profile 'flat'
gsettings set org.gnome.desktop.peripherals.mouse speed 0
```

This affects other mouse-class devices too. If you deliberately keep GNOME's
mouse `natural-scroll` enabled, remember that it will invert amt-raw's synthetic
wheel stream a second time. The GNOME touchpad natural-scroll preference remains
relevant only to the physical trackpad when using native mode.

#### GNOME high-refresh cursor presentation

On a tested 120 Hz GNOME/Mutter system, the raw `AMT Raw Pointer` stream remained
clean and evenly paced at 120 Hz while Mutter's visible hardware cursor showed
some temporal clumping. Forcing Mutter's software-cursor path substantially
improved presentation without changing `amt-raw` output.

For the smoothest tested GNOME cursor presentation, this optional workaround may
be used:

```bash
mkdir -p ~/.config/environment.d
cat > ~/.config/environment.d/90-amt-raw-mutter-cursor.conf <<'EOF2'
MUTTER_DEBUG_DISABLE_HW_CURSORS=1
EOF2
```

Log out completely and log back in. The setting affects Mutter cursor rendering
globally, not just `amt-raw`, so the helper does not enable it automatically.

To remove it:

```bash
rm ~/.config/environment.d/90-amt-raw-mutter-cursor.conf
```

Then log out and back in again.

### Cinnamon

`amt-raw-cinnamon-sync.service` supports pointer mode using Cinnamon's global
`send-events` setting. It saves/restores the user's exact previous value using
the same fail-safe model as the GNOME helper.

The helper is important even if cursor motion appears correct without it:
testing showed that the physical and virtual streams could coexist without an
obvious pointer problem while still producing incorrect/conflicting scrolling.

Because Cinnamon's public setting is global, other Cinnamon-managed touchpads
are disabled while pointer mode is active, and touchpad mode is intentionally
unsupported.

For neutral downstream pointer/scroll processing:

```bash
gsettings set org.cinnamon.desktop.peripherals.mouse natural-scroll false
gsettings set org.cinnamon.desktop.peripherals.mouse accel-profile 'flat'
gsettings set org.cinnamon.desktop.peripherals.mouse speed 0
```

Cinnamon's mouse natural-scroll setting is global, so the helper does not change
it automatically. Keep it `false` if `AMT_NATURAL_SCROLL` should be the sole
scroll-direction control for `AMT Raw Pointer`. Cinnamon's touchpad natural-scroll
setting remains relevant to the physical trackpad in native mode.

Useful checks:

```bash
gsettings get org.cinnamon.desktop.peripherals.touchpad send-events
systemctl --user status amt-raw-cinnamon-sync.service
```

## Troubleshooting

### Pointer or scrolling appears duplicated/conflicted

Check the desktop helper first. The physical device and `AMT Raw Pointer` should
not both be active inside the compositor.

```bash
systemctl --user status amt-raw-kde-sync.service
systemctl --user status amt-raw-gnome-sync.service
systemctl --user status amt-raw-cinnamon-sync.service
```

Run only the helper appropriate for the current desktop session.

### Scroll direction is reversed or inverted twice

In pointer/touchpad modes, set the desired Magic Trackpad direction with
`AMT_NATURAL_SCROLL`. On GNOME/Cinnamon, also confirm the global **mouse**
`natural-scroll` setting is `false`; otherwise the desktop may invert the
synthetic wheel stream again. KDE is reconciled per-device by its helper.

In native mode, `AMT_NATURAL_SCROLL` is ignored and the desktop's ordinary
**touchpad** natural-scrolling preference owns the physical device.

### KDE helper reports missing qdbus

Install:

```bash
sudo dnf install qt6-qttools
systemctl --user restart amt-raw-kde-sync.service
```

### Hardware requested at 120 Hz but reports near 110 Hz

This is expected on USB with the tested Magic Trackpad. The request value is not
used as a runtime timing assumption; measured timestamps drive the resampler.

On Bluetooth, the hardware-rate override is intentionally skipped because the
validated `0x99/0xf2` control is USB-specific. Resampling still works normally
and follows the measured Bluetooth report cadence.

### GNOME cursor looks less even than KDE at the same rate

Confirm the mouse acceleration profile is flat, then see the GNOME software
cursor workaround above. A capture can still show a clean 120 Hz `AMT Raw
Pointer` stream even when Mutter's hardware-cursor presentation is less even.

### Detailed diagnostics

The development capture utility is available at:

```text
tools/amt-liftoff-capture.py
```

It requires read access to the physical evdev device; run it with `sudo` when
needed.

## Technical documentation

For implementation details, state machines, pipeline diagrams, and the
resampling mathematics, see:

- [`docs/TECHNICAL.md`](docs/TECHNICAL.md)

The heavily commented shipped sysconfig is the authoritative end-user tuning
reference:

- [`amt-raw.sysconfig`](amt-raw.sysconfig)

## License

See [`LICENSE`](LICENSE).
