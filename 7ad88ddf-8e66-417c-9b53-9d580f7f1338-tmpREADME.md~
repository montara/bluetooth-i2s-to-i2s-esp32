# ESP32 I2S Passthrough DAC with Bluetooth

Firmware for an ESP32-based DAC that relays a wired I2S input straight to an I2S
output DAC, and hands the DAC over to a Bluetooth A2DP stream when a phone
connects and starts playing. A 2.4" TFT with a rotary encoder shows what is
playing and lets you pin the source.

## Hardware

**ESP32-WROOM-32** (or any original-ESP32 module). This is not a preference:
A2DP rides on Bluetooth Classic, which only the original ESP32 has. The S3, C3
and C6 are BLE-only and cannot be an A2DP sink at all.

The design assumes a DAC that needs **no MCLK** (PCM5102A, MAX98357A and
similar derive everything from BCLK). That is what frees GPIO0/1/3 — the only
pins the ESP32 can emit MCLK on — and lets GPIO1/3 stay on the serial console.

| Function | GPIO | Notes |
|---|---|---|
| I2S in BCLK | 34 | input-only pin |
| I2S in WS | 35 | input-only; also feeds PCNT for rate detection |
| I2S in DATA | 36 | input-only (SENSOR_VP) |
| I2S out BCLK | 26 | |
| I2S out WS | 25 | |
| I2S out DATA | 22 | |
| TFT SCLK / MOSI | 18 / 23 | VSPI (SPI3) |
| TFT CS / DC / RST | 5 / 21 / 19 | |
| TFT backlight | 4 | on/off; LEDC-capable for dimming later |
| EC11 A / B | 32 / 33 | hardware quadrature via PCNT |
| EC11 push (select) | 27 | to ground, internal pull-up |
| Back button | 14 | to ground, internal pull-up |

The ESP32 is the **slave** on the I2S input — the external source drives BCLK
and WS. Buttons wire to ground; no external pull-ups needed.

## Building

Requires **ESP-IDF v5.5 or newer**. This is a hard floor, for one reason:
`i2s_channel_tune_rate()`, which the drift correction depends on entirely, was
added in v5.5 and does not exist in v5.4.

Builds clean on v5.5.5 — full project, both components' host tests, `idf.py
build` through to a linked, sized `.bin` — as of the commit that added this
line.

```bash
git clone -b v5.5.3 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
~/esp/esp-idf/install.sh esp32
. ~/esp/esp-idf/export.sh

idf.py set-target esp32
idf.py build flash monitor
```

If `export.sh` reports its Python virtual environment missing (a fresh
`~/.espressif` tree without one, or a system Python upgrade that outdated the
old one), re-run `~/esp/esp-idf/install.sh esp32` — it rebuilds just that venv
without touching the toolchain or any downloaded components.

Configurable bits live under `idf.py menuconfig` → *DAC user interface* (panel
controller, colour inversion, LVGL buffer size) and *Bluetooth audio* (device
name).

## How it works

```
 I2S in (I2S0 slave) ─┐
   + PCNT rate detect ├─► audio_router ─► elastic buffer ─► I2S1 master ─► DAC
 A2DP sink (BT) ──────┘        │              ▲
                               │              └── drift servo
                               ▼
                          esp_event bus ─► UI
```

`audio_out` owns the output channel and one elastic buffer; the router decides
which producer may fill it. Everything else communicates over an `esp_event`
bus, so the UI never touches the audio path and the audio path never knows the
UI exists.

### The clocking problem

This is the part that makes the project more than a memcpy loop.

The ESP32 is a slave on the input, so it does not know the incoming sample rate
and cannot ask. We recover it by counting WS edges — WS frequency *is* the
sample rate — with a PCNT unit fed from the same GPIO the I2S peripheral
listens on. The GPIO matrix fans one input pin out to both, so this costs no
extra wiring.

Worse, the input and output run on *different crystals*. Even at a few parts
per million apart, the buffer between them fills or empties without bound —
minutes to hours before it audibly breaks, which is exactly long enough for the
problem to be missed in testing.

So the output clock is servo'd to the input. Every 100 ms the buffer's fill
level is compared to its 50 % target and MCLK is nudged via
`i2s_channel_tune_rate()`, which works on a running channel. The control law is
proportional-**derivative** on fill rather than the PI you might expect: the
tuning call accumulates, so the clock is already an integrator, and buffer fill
integrates the rate error on top of that. Two integrators in series oscillate;
the damping has to come from the derivative term. The derivation is in
`components/audio_out/audio_out.c`.

The same servo handles Bluetooth, where the far clock belongs to the phone.

### Source switching

Auto by default: Bluetooth takes over when a phone actually starts streaming,
not merely when it connects, and the wired input comes back when it stops. The
UI can pin it to either. Transitions are debounced by 250 ms so a stuttering
A2DP stream cannot flap the route, and the choice persists in NVS.

## Bring-up

The serial console is the fastest way through this. Commands: `stats`, `src`,
`servo`, `bt`, `heap`.

Work in this order — each step depends on the one before actually working:

1. **Output alone.** Scope BCLK and WS for the right frequencies. Then, on a
   running stream, `servo off` followed by `servo nudge 5000`: the pitch should
   shift slightly and `stats` should show the MCLK offset. This is the single
   assumption the whole design rests on, so prove it before building on it.
2. **Input detection.** Feed a known 44.1 k and 48 k source; `stats` should
   report the right rate and flip present/absent when you pull the cable.
3. **Passthrough.** Run a tone through for 30+ minutes watching `stats`. Fill
   should hover near 50 % with the MCLK offset settling to a small constant. A
   slow monotonic crawl toward 0 % or 100 % means the servo gains need trimming
   — or, if it heads for a limit fast, that a sign is wrong.
4. **Bluetooth.** Pair, confirm 44.1 kHz, check `heap` still shows headroom.
   Disconnect mid-stream and confirm fallback within ~250 ms.
5. **UI.** Encoder navigation, source menu, back button, and that Now Playing
   tracks events when you pull the I2S cable or drop the BT connection.

### Expected trouble spots

- **ESP32 I2S slave mode is fussy about first-bit alignment.** Received data can
  land shifted by one BCLK. If a known tone arrives recognisable but distorted,
  or the channels are swapped, suspect `slot_cfg.bit_shift` / `left_align` in
  `components/i2s_in/i2s_in.c` before looking anywhere else.
- **RAM is the binding constraint**, not CPU — this one is confirmed, not just
  anticipated. The stock LVGL Kconfig defaults (`LV_USE_BUILTIN_MALLOC`) reserve
  a **static 64 KB pool in BSS**, sized for the worst case regardless of what
  the UI actually uses — on a WROOM-32 with Bluedroid also resident, that alone
  overflowed the DRAM segment at link time by 96 bytes. `sdkconfig.defaults`
  now sets `CONFIG_LV_USE_CLIB_MALLOC=y`, which routes LVGL's internal
  allocations through the general heap instead; the build now links with
  **65 KB of DRAM free (47.5 % used)**. This is *static* headroom for the heap,
  not a guarantee against runtime pressure — `heap` after a BT connect is still
  the number to watch, since Bluedroid takes 110–160 KB from that same pool
  once a phone is connected and streaming. If it does get tight, in order: drop
  `CONFIG_UI_LVGL_BUFFER_LINES` to 20, shorten `RING_FRAMES`, then reduce
  Bluedroid's ACL buffer counts.
- **A managed component (`idf_component.yml`) dependency used only inside a
  public header must be a public `REQUIRES`, not left to auto-injection.**
  `ui_input.h` includes `lvgl.h` and exposes `lv_indev_t`/`lv_group_t` in its
  API; the component manager auto-adds an `idf_component.yml` dependency as a
  *private* requirement of the component that declares it, which is enough for
  that component's own `.c` file but does not propagate to anything that merely
  includes its header. `main.c` calling `ui_input_init()` was one such
  consumer, and failed to compile with `lvgl.h: No such file or directory`
  until `components/ui_input/CMakeLists.txt` listed `lvgl__lvgl` in `REQUIRES`
  explicitly. The rule of thumb: if a public header pulls in a dependency's
  types, that dependency has to be public too.
- **The A2DP PCM callback is a legacy path.** `esp_a2d_sink_register_data_callback()`
  moved to `esp_a2dp_legacy_api.h` in v5.5 and Espressif has said the internal
  SBC decoder will eventually be removed. The migration, if it comes, is
  documented at the top of `components/bt_audio/bt_audio.c`.

## Tests

The two pieces with real logic and no hardware dependency run on the host — no
ESP-IDF, no board:

```bash
./components/audio_out/test/run_tests.sh   # elastic buffer: wrap, overflow, rollover
./components/i2s_in/test/run_tests.sh      # rate snapping: tolerance bands
```

Both build with `-Werror` under ASan and UBSan.

## Layout

| Component | Responsibility |
|---|---|
| `board` | pin map; header-only, depended on by everything |
| `audio_events` | event base and shared types; no logic, prevents dependency cycles |
| `audio_out` | I2S1 master TX, elastic buffer, drift servo |
| `i2s_in` | I2S0 slave RX, PCNT rate detection and presence |
| `bt_audio` | A2DP sink, AVRCP, absolute volume |
| `audio_router` | source state machine, NVS persistence |
| `ui` / `ui_input` | LVGL screens; EC11 and buttons as an LVGL input device |
| `console_cmds` | serial console for bring-up |
