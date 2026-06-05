# PaperSatColor

A four-satellite **next-pass dashboard** for the **M5Paper Color** (ESP32-S3, 4" SPECTRA 6 color e-paper, 600×400). The screen is a static 2×2 grid: each cell shows one satellite's next pass as a polar (azimuth/elevation) plot of the pass track, with the AOS/LOS times and the pass's maximum azimuth and elevation. It is built for a slow color e-ink panel — the display redraws only after a tracked pass has concluded (so the next pass can take its place), or when you change the configuration.

All configuration is done over Wi-Fi through a small web page the device serves; there are no on-device menus or buttons to operate.

## Why a static dashboard

Color SPECTRA 6 e-paper takes roughly 15–19 seconds to refresh and **cannot do partial updates** — rendering its colors requires a full-panel waveform. A live-tracking UI that redraws frequently is therefore unusable on this hardware. PaperSatColor instead treats the panel as a glanceable status board: it computes the next pass for four satellites, draws once, and then sits still until a pass event changes what should be shown.

## Features

- **Four satellites at once**, in a 2×2 grid, each with its own polar plot.
- **Next-pass polar plot** per satellite: the pass ground track in blue, a green dot where the satellite rises (AOS), a red dot where it sets (LOS), and a red marker at the highest point. North is up; the horizon is the outer circle, the zenith is the center, and a soft yellow fill marks the high-elevation (above 45°) zone.
- **Pass details** per satellite: AOS time and rise azimuth (green), LOS time and set azimuth (red), the maximum elevation, and the pass date. A cell currently in a pass is flagged "PASS IN PROGRESS."
- **Full six-color display** with a clean anti-aliased typeface. The text uses the GNU FreeSans font (FreeSansBold for satellite names) rather than the blocky built-in font. The SPECTRA 6 inks are used semantically: green = rise/AOS, red = set/LOS and peak, blue = ground track and headings, yellow = the good-elevation zone. The satellite name and the max-elevation figure are colored by pass quality (green = marginal, blue = good, red = excellent), so you can judge each pass at a glance.
- **Event-driven refresh**: the panel redraws only after a tracked pass ends (rolling that cell forward to its next pass), when you save new configuration, and once a day to pick up the latest orbital data. While a pass is in progress its cell stays on screen.
- **Wi-Fi-only configuration**: a built-in web page with four dropdowns populated from the AMSAT bulletin satellite list, plus station location fields (lat/lon, altitude, or a Maidenhead grid).
- **Orbital data over Wi-Fi** from the AMSAT daily bulletin, rebuilt into SGP4 elements on-device, with an offline LittleFS cache so passes keep computing without a connection.
- **RTC-backed UTC timekeeping** (RX8130CE) for accurate pass prediction immediately on power-up, even before Wi-Fi connects. The time is used internally only — it isn't shown on screen, since the panel refreshes far too infrequently for a clock to stay accurate.
- **Pass alerts (LED + sound).** The two onboard RGB LEDs hold a steady color showing the current phase of the nearest pass, and the speaker plays a short tone at each transition. The phases are: amber from 5 minutes before AOS, orange from 1 minute before AOS, green for the whole pass (AOS to LOS), and red for 30 seconds after LOS, then off. A distinct tone marks each boundary (two low beeps at T-5, three mid beeps at T-1, a rising two-tone at AOS, a falling two-tone at LOS). With four satellites the LED shows the most urgent phase across all of them (in progress beats imminent beats upcoming beats just-ended).

## Hardware

- **M5Paper Color ESP32S3 Dev Kit** — ESP32-S3R8, 16 MB flash, 8 MB PSRAM, 4" SPECTRA 6 color e-paper (600×400), RX8130CE RTC, 1250 mAh battery, Wi-Fi.

The three physical buttons are not used. No external wiring is required.

## Display orientation

The dashboard runs in **portrait** (400 wide × 600 tall) via `setRotation(0)`. If your unit reads upside-down, change it to `setRotation(2)` in `setup()`.

## What each cell shows

```
            SAT NAME            <- colored by pass quality
            (polar plot)
             N
           W + E                <- horizon circle, yellow 45 deg zone,
             S                     blue track, green AOS dot, red LOS dot,
                                   red peak marker
  AOS hh:mm  Az nnn   (green)
  LOS hh:mm  Az nnn   (red)
  Max El nn          MM/DD
```

The polar plot maps each sampled point of the pass to a radius of `(90 − elevation) / 90`, so a point at the zenith sits at the center and a point on the horizon sits on the outer circle; the angle is the azimuth measured clockwise from north. The green dot is where the satellite rises, the red dot is where it sets, and the red marker is the highest point of the pass. The text gives the rise time and azimuth, the set time and azimuth, and the maximum elevation.

## First-time setup

1. Flash and power on. On first boot the device starts a Wi-Fi access point named **`PaperSatColor-Setup`** (this is WiFiManager's captive portal). Join it from a phone or laptop and enter your network credentials.
2. Once the device is on your network, it shows its **IP address** in the header bar at the top of the screen.
3. Browse to that IP address. The setup page has four dropdowns — one per grid cell — populated from the AMSAT bulletin. Pick the satellite for each slot.
4. Set your station location: latitude/longitude and altitude, or just a Maidenhead grid square (which overrides lat/lon). If you leave the defaults, the device will try to estimate your location from your public IP on first run.
5. Press **Save & Refresh**. The device stores your choices, fetches fresh orbital elements for the chosen satellites, recomputes their next passes, and redraws once (about 20 seconds on this panel).

Your configuration persists across reboots.

## How it works

On boot the device connects to Wi-Fi, seeds its clock from the RTC, syncs time over NTP, and downloads the AMSAT daily bulletin (`daily-bulletin.json`). It reconstructs standard two-line elements for the four chosen satellites from the bulletin's discrete orbital-element fields, then runs the SGP4 propagator to find each satellite's next pass — capturing AOS, LOS, the peak elevation, the azimuth at peak, and a set of points along the pass for the polar plot. The bulletin is cached to on-board flash so the tracker keeps working offline using the last known elements.

The main loop is almost idle: it services the configuration web server, drives the alert LEDs/tones, and watches a single scheduled time — the soonest pass *end* (LOS) across all four cells. While a pass is in progress its cell stays on screen; only once the pass concludes does the device recompute all four next passes and redraw once, so the slow panel never refreshes mid-pass. A daily refresh also runs to pull the latest bulletin. Nothing else triggers a redraw, which keeps the panel quiet and the power draw low.

Time is kept in UTC, backed by the battery-powered RTC: the system clock is seeded from the RTC at startup, and once NTP provides a better time it is written back so the correct time survives a full power-off.

## Building

This is an Arduino sketch. The `.ino` file must live in a folder of the same name (`PaperSatColor/PaperSatColor.ino`).

### Board support

Install the ESP32 boards package (Espressif Systems) and select the M5Paper Color / ESP32-S3 target. Enable PSRAM in the board options — the bulletin parser allocates a large JSON document that relies on it.

### Libraries

Install through the Arduino Library Manager (or PlatformIO `lib_deps`):

- **M5Unified** and **M5GFX** — board, display, RTC, and battery support.
- **WiFiManager** (tzapu) — captive-portal Wi-Fi credential setup.
- **ArduinoJson** (Benoit Blanchon) — parsing the AMSAT bulletin.
- **Sgp4** — the SGP4 orbital propagator (Hopkins' Arduino port).
- **FastLED** — drives the two onboard RGB LEDs for pass alerts (M5Unified does not control the RGB LEDs itself).

`WiFi`, `WebServer`, `HTTPClient`, `Preferences`, and `LittleFS` ship with the ESP32 core.

> **LED data pin:** the sketch defines `LED_DATA_PIN` near the top (default 21). RGB-LED wiring varies between board revisions, so check this against your unit's GPIO map. If it's wrong, only the LED alerts are affected — the tones and the rest of the app still work.

### Flashing

Connect the board over USB-C and flash from the Arduino IDE (or `pio run -t upload`). If the board does not enter download mode automatically, hold the side reset button while flashing.

## Configuration defaults

| Setting | Default |
| --- | --- |
| Tracked satellites | ISS, FOX-1B, AO-7, FOX-1D |
| Location | 38.8626, -77.0562 (Washington, DC area), auto-located on first run |
| Orbital data source | AMSAT daily bulletin |
| Time base | UTC (NTP via `pool.ntp.org`, RTC-backed) |

All of these are changeable from the web page; your choices are saved automatically.

## Notes and limitations

- **Color e-paper is slow and cannot do partial refresh.** Every redraw is a full-panel flash of roughly 15–19 seconds. This is inherent to SPECTRA 6 and is the entire reason the dashboard is event-driven rather than live.
- **No microSD card is required.** The card slot is left unprobed at startup, and the orbital-data cache uses internal flash (LittleFS). If the flash filesystem can't be mounted the device still runs — it just re-downloads the bulletin each time instead of caching it.
- **All times are UTC.**
- A satellite with no pass in the prediction window shows "no pass found"; a satellite whose elements could not be built shows "no orbital data" (usually resolved after the next bulletin download).
- The battery percentage assumes a single-cell LiPo (3.4 V empty, 4.2 V full); if your board's power reading differs, the percentage may need calibration.
- The configuration web page is served unencrypted on your local network and is not password-protected; anyone on the network can change the tracked satellites.

## Troubleshooting

- **"GP parse failed" or "Download failed".** The device fetches the AMSAT bulletin over HTTPS using an insecure TLS client (no certificate is bundled on-device). If the download still fails it is almost always a network issue — confirm the device has internet access and that `newark192.amsat.org` is reachable from your network. Once a successful download is cached, the dashboard keeps working from that copy.
- **The screen shows "offline" where the IP should be.** Wi-Fi hasn't connected. Re-run the captive-portal setup by connecting to the `PaperSatColor-Setup` access point.

## Credits

Orbital data courtesy of AMSAT. Satellite propagation uses the SGP4 model.
