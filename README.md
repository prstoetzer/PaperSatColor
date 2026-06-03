# PaperSatColor

A standalone amateur-radio satellite tracker for the **M5Paper Color** (ESP32-S3, 4" SPECTRA 6 color e-paper, 600×400). It shows where your selected satellite is in the sky right now, plots its current or next pass on a polar (az/el) map, and lists upcoming pass times — all on a low-power color e-ink display that stays readable with the power off.

This is a port of the original PaperSat (built for the monochrome, touchscreen M5Paper S3) to the newer color, button-only hardware. The display is rendered in portrait orientation and the entire interface is driven by the board's three physical buttons.

## Features

- **Live sky position** of the selected satellite, drawn on a polar plot (zenith at center, horizon at the edge), with a direction-of-travel arrow.
- **Pass prediction** using SGP4, showing AOS → LOS times and maximum elevation for the next several passes.
- **Color status cues** made possible by the SPECTRA 6 panel: the satellite marker and its name turn red while it is above the horizon, and the plotted pass arc is drawn in blue.
- **Orbital data over Wi-Fi** from the AMSAT daily bulletin, rebuilt into SGP4 elements on-device, with an offline cache so the tracker keeps working without a connection.
- **Flexible location entry**: Maidenhead grid square, raw latitude/longitude, or automatic geolocation from your public IP.
- **Time sync** via NTP, with a manual UTC entry fallback.

## Hardware

- **M5Paper Color ESP32S3 Dev Kit** — ESP32-S3R8, 16 MB flash, 8 MB PSRAM, 4" SPECTRA 6 color e-paper (600×400), three programmable buttons, 1250 mAh battery, Wi-Fi.

No additional wiring is required; the app uses only the built-in display, buttons, battery gauge, and Wi-Fi.

## Display orientation

The app runs in **portrait** (400 wide × 600 tall) via `setRotation(0)`. If your unit reads upside-down, change the rotation in `setup()` to `setRotation(2)`.

## Controls

The board has three buttons (A, B, C). There is no touchscreen. A hint bar at the bottom of every screen always shows what A / B / C do in the current context.

| Button | Short press | Long press (hold ~0.6 s) |
| --- | --- | --- |
| **A** | Move highlight up / previous | — |
| **B** | Select the highlighted item | Back / cancel |
| **C** | Move highlight down / next | — |

The currently highlighted item is drawn as a filled (inverted) block so it is easy to see on the slow-refreshing panel.

### Entering text (grid square, lat/lon, time)

Because three buttons can't drive a full keyboard, text entry uses a **character wheel**:

1. Press **A / C** to scroll through the available characters.
2. Press **B** to add the highlighted character to your entry.
3. Scroll to **`DEL`** and press **B** to erase the last character.
4. Scroll to **`OK`** and press **B** to confirm and save.

Long-pressing **B** at any time cancels and returns to the setup menu.

## Screens

- **Main** — satellite name, UTC clock, battery, the polar sky plot, current Az/El, the next three pass times, data status, and the action row: *Refresh*, *Sat List*, *Setup*.
- **Sat List** — pick a satellite from the AMSAT bulletin (paged), force a fresh data download (*Update GP*), or go back.
- **Setup** — enter a Maidenhead grid, enter latitude/longitude, open Wi-Fi configuration, auto-locate via Wi-Fi, or set the UTC time/date.

## How it works

On boot the app connects to saved Wi-Fi, syncs time over NTP, and downloads the AMSAT daily bulletin (`daily-bulletin.json`). For the selected satellite it reconstructs standard two-line elements from the bulletin's discrete orbital-element fields and feeds them to the SGP4 propagator. The bulletin is cached to the on-board flash (LittleFS) so the tracker continues to work offline using the last known elements. Your location, selected satellite, and cached elements are persisted in non-volatile storage and survive reboots.

The bulletin is refreshed roughly once a day (or on demand from *Sat List → Update GP*). On the main screen the position and pass list update about once a minute while a pass is in progress and about every five minutes otherwise — deliberately infrequent, because color e-paper takes roughly 15–19 seconds to perform its full-screen refresh and flashes while doing so. Button presses are always handled immediately.

## Building

This is an Arduino sketch. The `.ino` file must live in a folder of the same name (`PaperSatColor/PaperSatColor.ino`).

### Board support

Install the ESP32 boards package (Espressif Systems) via the Arduino Boards Manager and select the M5Paper Color / ESP32-S3 target. Enable PSRAM in the board options; the bulletin parser allocates a large JSON document that relies on it.

### Libraries

Install these through the Arduino Library Manager (or PlatformIO `lib_deps`):

- **M5Unified** and **M5GFX** — board, display, and button support.
- **WiFiManager** (tzapu) — captive-portal Wi-Fi setup.
- **ArduinoJson** (Benoit Blanchon) — parsing the AMSAT bulletin.
- **Sgp4** — the SGP4 orbital propagator (Hopkins' Arduino port).

`WiFi`, `HTTPClient`, `Preferences`, and `LittleFS` ship with the ESP32 core.

### Flashing

Connect the board over USB-C and flash from the Arduino IDE (or `pio run -t upload`). If the board does not enter download mode automatically, hold the side reset button while flashing.

## First-time setup

1. Flash and power on. The app starts trying to connect to any previously saved Wi-Fi.
2. Open **Setup → WiFi Configuration**. The board starts an access point named **`PaperSatColor-Setup`**. Join it from a phone or laptop and browse to `192.168.4.1` to enter your network credentials.
3. Once online, the app can set your location automatically (**Setup → Auto Location via WiFi**), or you can enter a Maidenhead grid or latitude/longitude manually.
4. Open **Sat List** to choose a satellite. The default is the ISS (NORAD 25544).

## Configuration defaults

| Setting | Default |
| --- | --- |
| Location | 38.8626, −77.0562 (Washington, DC area) |
| Satellite | ISS (NORAD 25544) |
| Orbital data source | AMSAT daily bulletin |
| Time base | UTC (NTP via `pool.ntp.org`) |

These can all be changed at runtime from the Setup and Sat List screens; your choices are saved automatically.

## Notes and limitations

- **Color e-paper is slow.** Expect a full-screen flash of roughly 15–19 seconds on each redraw. This is normal for SPECTRA 6 panels and is why the interface avoids frequent updates.
- **Pass times are in UTC.**
- The battery percentage assumes a single-cell LiPo (3.4 V empty, 4.2 V full); if your board's power reading differs, the percentage may need calibration.
- An on-board real-time clock (RX8130CE) is present on this hardware but is not yet used; time currently comes from NTP or manual entry.

## Credits

PaperSatColor is a hardware port of the original PaperSat project. Orbital data is courtesy of AMSAT. Satellite propagation uses the SGP4 model.
