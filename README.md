# PaperSatColor

An **up-to-20-satellite next-pass dashboard** for the **M5Paper Color** (ESP32-S3, 4" SPECTRA 6 color e-paper, 600×400). It tracks up to twenty satellites across up to five pages of four, each shown in a 2×2 grid; each cell is a polar (azimuth/elevation) plot of that satellite's next pass with the AOS/LOS times and azimuths and the maximum elevation. You don't have to fill every slot: any slot you leave blank is simply left empty, and any page with no satellites at all is hidden — so if you only configure six satellites you get two pages, not five. It is built for a slow color e-ink panel — the display redraws only after a pass on the currently shown page has concluded (so the next pass can take its place), when you switch pages, when you manually refresh, or when you change the configuration.

All configuration is done over Wi-Fi through a small web page the device serves; there are no on-device menus or buttons to operate.

## Why a static dashboard

Color SPECTRA 6 e-paper takes roughly 15–19 seconds to refresh and **cannot do partial updates** — rendering its colors requires a full-panel waveform. A live-tracking UI that redraws frequently is therefore unusable on this hardware. PaperSatColor instead treats the panel as a glanceable status board: it computes the next pass for every configured satellite, draws the current page once, and then sits still until a pass on that page ends or you press a button.

## Features

- **Up to 20 satellites across up to 5 pages**, four per page in a 2×2 grid, each with its own polar plot. Blank slots are left empty and fully blank pages are skipped, so the number of pages matches how many satellites you actually configure. Press the up/down buttons to move between the occupied pages.
- **Physical buttons** (mapped per the board's published USER_KEY spec): **USER_KEY1** triggers a manual refresh (re-fetch the bulletin, recompute, redraw), or saves a screenshot to microSD when held; **USER_KEY2** and **USER_KEY3** page up and down through the occupied pages.
- **Screenshot to microSD**: hold **USER_KEY1** for about a second to save the current screen as a 24-bit BMP under `/screenshots` on the card. Files are auto-numbered so captures never overwrite each other, and the card is only accessed when you take a shot.
- **Next-pass polar plot** per satellite: the pass ground track in blue, a green dot where the satellite rises (AOS), a red dot where it sets (LOS), and a red marker at the highest point. North is up; the horizon is the outer circle, the zenith is the center, and a soft yellow fill marks the high-elevation (above 45°) zone.
- **Pass details** per satellite: AOS time and rise azimuth (green), LOS time and set azimuth (red), the maximum elevation, and the pass date. A cell whose pass is happening right now shows a red **NOW** badge in place of the date.
- **Full six-color display** with a clean anti-aliased typeface. The text uses the GNU FreeSans font (FreeSansBold for satellite names) rather than the blocky built-in font. The SPECTRA 6 inks are used semantically: green = rise/AOS, red = set/LOS and peak, blue = ground track and headings, yellow = the good-elevation zone. The satellite name and the max-elevation figure are colored by pass quality (green = marginal, blue = good, red = excellent), so you can judge each pass at a glance.
- **Event-driven refresh**: the panel redraws only when a pass on the *currently displayed* page ends (rolling that cell forward to its next pass), when you switch pages, when you press the refresh button, when you save new configuration, and once a day for fresh orbital data. A pass ending on the page you're not viewing updates silently — no wasteful refresh — and while a pass is in progress its cell stays on screen.
- **Wi-Fi-only configuration**: a built-in web page with up to twenty dropdowns (grouped under five page headings) populated from the AMSAT bulletin satellite list, plus station location fields (lat/lon, altitude, or a Maidenhead grid) and toggles to enable or disable LED and sound alerts independently. Each dropdown has a **(blank)** choice, so you're never forced to assign a satellite to a slot.
- **Orbital data over Wi-Fi** from the AMSAT daily bulletin, rebuilt into SGP4 elements on-device, with an optional offline cache (internal flash) so passes keep computing without a connection when a filesystem partition is available.
- **RTC-backed UTC timekeeping** (RX8130CE) for accurate pass prediction immediately on power-up, even before Wi-Fi connects. The time is used internally only — it isn't shown on screen, since the panel refreshes far too infrequently for a clock to stay accurate.
- **Status footer** showing the age of the orbital data as "GP data: MMM DD HH:MM UTC" (the time of the last successful bulletin update), so you can see at a glance how current the predictions are.
- **Pass alerts (LED + sound) for every configured satellite**, independently toggleable from the web config. The two onboard RGB LEDs hold a steady color showing the current phase of the nearest pass across *all* tracked satellites (every page, not just the one on screen), and the speaker plays a short tone at each transition. The phases are: amber from 5 minutes before AOS, orange from 1 minute before AOS, green for the whole pass (AOS to LOS), and red for 30 seconds after LOS, then off. A distinct tone marks each boundary (two low beeps at T-5, three mid beeps at T-1, a rising two-tone at AOS, a falling two-tone at LOS). The LED shows the most urgent phase across all satellites (in progress beats imminent beats upcoming beats just-ended), so you're alerted to a pass even when it's on a page you aren't viewing. Either or both outputs can be turned off in the web config; the setting persists across reboots.

## Hardware

- **M5Paper Color ESP32S3 Dev Kit** — ESP32-S3R8, 16 MB flash, 8 MB PSRAM, 4" SPECTRA 6 color e-paper (600×400), RX8130CE RTC, 1250 mAh battery, Wi-Fi.

The three physical buttons control refresh and paging (see below); no external wiring is required.

## Buttons

All three buttons are mapped per the board's published USER_KEY specification:

- **USER_KEY1** — manual refresh: re-fetch the AMSAT bulletin, recompute all passes, and redraw. **Hold it for ~1 second** to instead save a screenshot of the current screen to the microSD card (see below).
- **USER_KEY2** — page up: show the previous occupied page.
- **USER_KEY3** — page down: show the next occupied page.

A short press and a long press of USER_KEY1 are distinguished by an 800 ms hold threshold, so a normal tap refreshes and only a deliberate hold triggers a screenshot.

### Screenshots

Holding USER_KEY1 captures the current framebuffer and writes it to the microSD card as a 24-bit uncompressed BMP at `/screenshots/shotNNNN.bmp`, where `NNNN` is the next free number (so successive captures don't overwrite each other). The footer briefly shows "Screenshot saved" on success, or "Screenshot failed (no SD?)" if the card couldn't be written. At 400×600 each file is about 720 KB.

The card is initialised the first time you take a screenshot and is otherwise never touched, so running without a card has no effect on normal operation. This is separate from the optional LittleFS bulletin cache — the two use different storage. If screenshots fail, check that a FAT-formatted microSD card is inserted and that the SD SPI pins defined near the top of the sketch (`SD_SPI_CS_PIN` etc., defaulting to the published M5PaperS3 pinout) match your unit.

The button-to-`M5.Btn` mapping is defined near the top of the sketch (`BTN_REFRESH`, `BTN_PAGE_UP`, `BTN_PAGE_DN`). If a future board revision changes the assignment, swap those three defines.

## Display orientation

The dashboard runs in **portrait** (400 wide × 600 tall) via `setRotation(0)`. If your unit reads upside-down, change it to `setRotation(2)` in `setup()`.

## What the screen shows

A thin header runs across the top: **PaperSatColor**, then **Config:** followed by the device's IP address (where to open the setup page over Wi-Fi), and the battery percentage at the right. Below that is the 2×2 grid showing the (up to) four satellites of the current page, with any unused slot left blank, and a status footer with a small legend, a page indicator ("Pg 1/3", counting only the pages that actually have satellites), and the age of the orbital data. Use the up/down buttons to move between the occupied pages. If no satellites are configured at all, the grid shows a short prompt pointing you to the web config instead.

### What each cell shows

```
            SAT NAME            <- colored by pass quality
            (polar plot)
             N
           W + E                <- horizon circle, yellow 45 deg zone,
             S                     blue track, green AOS dot, red LOS dot,
                                   red peak marker
  AOS hh:mm  Az nnn   (green)
  LOS hh:mm  Az nnn   (red)
  Max El nn          MM/DD or NOW
```

The polar plot maps each sampled point of the pass to a radius of `(90 − elevation) / 90`, so a point at the zenith sits at the center and a point on the horizon sits on the outer circle; the angle is the azimuth measured clockwise from north. The green dot is where the satellite rises, the red dot is where it sets, and the red marker is the highest point of the pass. The text gives the rise time and azimuth, the set time and azimuth, and the maximum elevation.

## First-time setup

1. Flash and power on. On first boot the device starts a Wi-Fi access point named **`PaperSatColor-Setup`** (this is WiFiManager's captive portal). Join it from a phone or laptop and enter your network credentials.
2. Once the device is on your network, the header at the top of the screen shows **Config:** followed by its **IP address**.
3. Browse to that IP address. The setup page has up to twenty dropdowns, grouped under headings Page 1 through Page 5 (four slots each), populated from the AMSAT bulletin. Pick a satellite for each slot you want to use and leave the rest on **(blank)**. Blank slots stay empty and any page left entirely blank won't be shown. An **Alerts** section at the bottom lets you enable or disable LED and sound alerts independently.
4. Set your station location: latitude/longitude and altitude, or just a Maidenhead grid square (which overrides lat/lon). If you leave the defaults, the device will try to estimate your location from your public IP on first run.
5. Press **Save & Refresh**. The device stores your choices, fetches fresh orbital elements for the chosen satellites, recomputes their next passes, and redraws once (about 20 seconds on this panel).

Your configuration persists across reboots.

## How it works

On boot the device connects to Wi-Fi, seeds its clock from the RTC, syncs time over NTP, and downloads the AMSAT daily bulletin (`daily-bulletin.json`). It reconstructs standard two-line elements for the configured satellites from the bulletin's discrete orbital-element fields, then runs the SGP4 propagator to find each satellite's next pass — capturing the AOS time and rise azimuth, the LOS time and set azimuth, the peak elevation, and a set of points along the pass for the polar plot. The bulletin is cached to on-board flash so the tracker keeps working offline using the last known elements, and the time of the last successful update is remembered across reboots.

The main loop is almost idle: it services the configuration web server, drives the alert LEDs and tones (when enabled), and watches the per-satellite scheduled roll times — each satellite has its own LOS+margin timestamp after which it recomputes its next pass. While a pass is in progress its cell stays on screen; only once the pass concludes (and its post-pass alert window has elapsed) does the device compute the next pass for that satellite and, if it's on the current page, redraw once — so the slow panel never refreshes mid-pass and never re-shows a pass that already finished. A daily refresh also runs to pull the latest bulletin. Nothing else triggers a redraw, which keeps the panel quiet and the power draw low.

On startup, before any new download completes, the device uses its saved location, the RTC clock, and the last cached bulletin so it can show passes immediately; the footer reports the date of that last update. The RTC is battery-backed, and once NTP provides a more precise time it is written back so the correct time survives a full power-off.

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

`WiFi`, `WebServer`, `HTTPClient`, `Preferences`, `LittleFS`, `SPI`, and `SD` ship with the ESP32 core (the latter two are used for the screenshot feature).

> **LED data pin:** the sketch defines `LED_DATA_PIN` near the top (default 21). RGB-LED wiring varies between board revisions, so check this against your unit's GPIO map. If it's wrong, only the LED alerts are affected — the tones and the rest of the app still work.

> **microSD pins:** the SD SPI pins (`SD_SPI_CS_PIN`, `SD_SPI_SCK_PIN`, `SD_SPI_MOSI_PIN`, `SD_SPI_MISO_PIN`) are defined near the top and default to the published M5PaperS3 pinout (47/39/38/40). They are used only for the screenshot feature; if screenshots report no card, verify these against your unit's pinout.

### Flashing

Connect the board over USB-C and flash from the Arduino IDE (or `pio run -t upload`). If the board does not enter download mode automatically, hold the side reset button while flashing.

## Configuration defaults

| Setting | Default |
| --- | --- |
| Tracked satellites | Page 1: ISS, RS-44, AO-07, SO-50 · Page 2: AO-91, AO-27, FO-29, PO-101 · Pages 3–5: blank |
| Location | 38.8626, -77.0562 (Washington, DC area), auto-located on first run |
| Orbital data source | AMSAT daily bulletin |
| Time base | UTC (NTP via `pool.ntp.org`, RTC-backed) |
| LED alerts | Enabled |
| Sound alerts | Enabled |

All of these are changeable from the web page; your choices are saved automatically.

## Notes and limitations

- **Color e-paper is slow and cannot do partial refresh.** Every redraw is a full-panel flash of roughly 15–19 seconds. This is inherent to SPECTRA 6 and is the entire reason the dashboard is event-driven rather than live.
- **No microSD card is required** for normal operation. The card slot is left unprobed at startup, and the orbital-data cache uses internal flash (LittleFS). If the flash filesystem can't be mounted the device still runs — it just re-downloads the bulletin each time instead of caching it. To enable caching (so the data survives a reboot without a fresh download), select an Arduino partition scheme that includes a SPIFFS/LittleFS partition. The time of the last successful update is stored separately and persists across reboots either way. A microSD card is needed only if you want to use the screenshot feature.
- **All times are UTC.**
- A satellite with no pass in the prediction window shows "no pass found"; a satellite whose elements could not be built shows "no orbital data" (usually resolved after the next bulletin download).
- The battery percentage assumes a single-cell LiPo (3.4 V empty, 4.2 V full); if your board's power reading differs, the percentage may need calibration.
- The configuration web page is served unencrypted on your local network and is not password-protected; anyone on the network can change the tracked satellites.

## Troubleshooting

- **"GP parse failed" or "Download failed".** The device fetches the AMSAT bulletin over HTTPS using an insecure TLS client (no certificate is bundled on-device). If the download still fails it is almost always a network issue — confirm the device has internet access and that `newark192.amsat.org` is reachable from your network. Once a successful download is cached, the dashboard keeps working from that copy.
- **The header shows "Config: offline".** Wi-Fi hasn't connected. Re-run the captive-portal setup by connecting to the `PaperSatColor-Setup` access point.
- **The footer shows "Waiting for first download" or "No WiFi yet, no cached data".** The device has never successfully fetched the bulletin and has no cache to fall back on. Confirm it has internet access; once one download succeeds, the footer switches to "GP data: <date>" and that timestamp persists across reboots.

## Credits

Orbital data courtesy of AMSAT. Satellite propagation uses the SGP4 model.
