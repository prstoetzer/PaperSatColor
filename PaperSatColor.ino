#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <Sgp4.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <LittleFS.h>
#include <ArduinoJson.h>  // Install via Library Manager: "ArduinoJson" by Benoit Blanchon

// ====================== DISPLAY: M5Paper Color (ESP32-S3, 4" SPECTRA 6) ======================
// 6-color (Spectra 6) panel: black, white, red, yellow, blue, green. Driven via
// M5.Display (M5GFX). Refreshes are FULL only and slow (~15-19s with whole-panel
// flashing), so we redraw rarely and use color, not animation, to convey state.
//
// PORTRAIT orientation: native panel is 400(w) x 600(h). setRotation(0) keeps it
// portrait. All layout below assumes 400 wide x 600 tall.
//
// INPUT: no touchscreen. Three hardware buttons via M5Unified Button_Class:
//   BtnA = UP/PREV, BtnB = SELECT (long-press = BACK), BtnC = DOWN/NEXT.
// Navigation uses a highlight cursor (selIndex) over the focusable items on each
// screen; A/C move it, B activates. Text entry scrolls a character with A/C and
// commits it with B.
#define SCR_W 400
#define SCR_H 600

// Spectra 6 palette. M5GFX maps the nearest of the 6 hardware inks for these.
#define COL_BG       TFT_WHITE
#define COL_FG       TFT_BLACK
#define COL_ACCENT   TFT_RED     // current satellite, "visible now"
#define COL_PATH     TFT_BLUE    // ground-track / pass arc
#define COL_OK       TFT_GREEN   // good status
#define COL_WARN     TFT_RED     // warnings/errors
#define COL_HILITE   TFT_BLUE    // selection cursor highlight

// ---- Button-navigation state ----
// selIndex is the currently highlighted focusable item on the active screen.
// itemCount is set by each draw function so handleButtons() knows the range.
int  selIndex = 0;
int  itemCount = 0;
// For text-entry screens: index into the active character set being scrolled.
int  charCursor = 0;
// Hold threshold (ms) for BtnB long-press = Back.
const uint32_t HOLD_MS = 600;

// ====================== CONFIG ======================
double qth_lat = 38.8626;
double qth_lon = -77.0562;
double qth_alt = 10.0;

String selectedName = "ISS";
String selectedNorad = "25544";

struct Satellite {
  char name[25];
  char norad[10];
};
Satellite satList[200];
int satCount = 0;
int currentSatPage = 0;
const int satsPerPage = 10;

Sgp4 sat;
Preferences prefs;

char currentTLE1[80], currentTLE2[80];
time_t lastTLETime = 0;
bool forceTLEUpdate = false;

struct Pass {
  time_t aos, los;
  double maxEl;
};
Pass passes[8];
int passCount = 0;

unsigned long lastUpdate = 0;

// One-shot: becomes true once we've persisted an NTP-synced clock to the RTC.
bool rtcSyncedFromNtp = false;

// Threshold for considering time(nullptr) valid (post 2021 to detect unset NTP time)
const time_t TLE_TIME_VALID_THRESHOLD = 1609459200LL; // 2021-01-01 UTC

enum Screen { MAIN, SAT_SELECT, SETUP_MENU, GRID_INPUT, LATLON_INPUT, TIME_INPUT };
Screen currentScreen = MAIN;

String inputBuffer = "";
String statusMsg = "Booting...";

// Character sets for the 3-button text-entry screens. The two trailing pseudo
// "characters" act as inline actions while scrolling: backspace and commit/done.
const char* GRID_CHARS   = "ABCDEFGHIJKLMNOPQRSTUVWX0123456789";
const char* LATLON_CHARS = "0123456789.-,";
const char* TIME_CHARS   = "0123456789-T: ";
const char* CHAR_BKSP = "<DEL>";
const char* CHAR_DONE = "<DONE>";

// ====================== FORWARD DECLARATIONS ======================
void drawMainScreen();
void drawSatSelectScreen();
void drawSetupMenu();
void drawGridInputScreen();
void drawLatLonInputScreen();
void drawTimeInputScreen();
void drawDegreeSymbol(int16_t x, int16_t y);
void redrawCurrent();
void commitGrid();
void commitLatLon();
void commitTime();
void openSetupPortal();
void saveConfig();
void loadConfig();
void updateData();
bool autoLocateViaWiFi();
void writeSystemClockToRtc();
bool syncSystemClockFromRtc();

// ====================== HELPER ======================
void drawDegreeSymbol(int16_t x, int16_t y) {
  M5.Display.drawCircle(x + 3, y + 4, 2, COL_FG);
}

time_t jdToUnix(double jd) {
  return (jd - 2440587.5) * 86400.0;
}

void setSystemTime(int year, int mon, int day, int hour, int min, int sec) {
  struct tm t;
  t.tm_year = year - 1900;
  t.tm_mon = mon - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = sec;
  t.tm_isdst = 0;
  time_t epoch = mktime(&t);
  if (epoch != (time_t)-1) {
    struct timeval tv;
    tv.tv_sec = epoch;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    writeSystemClockToRtc();   // persist across power loss
  }
}

// ====================== REAL-TIME CLOCK (RX8130CE) ======================
// The board has a battery-backed RTC. We use it so the tracker has a valid UTC
// clock the instant it powers on, even with no Wi-Fi, and so manually entered or
// NTP-synced time survives a full power-off. All RTC values are kept in UTC, to
// match NTP (configTime offset 0) and the UTC-labeled displays.

// Write the current ESP32 system clock into the RTC (call after NTP or manual set).
void writeSystemClockToRtc() {
  if (!M5.Rtc.isEnabled()) return;
  time_t now = time(nullptr);
  if (now < TLE_TIME_VALID_THRESHOLD) return;  // don't store an unset clock
  M5.Rtc.setDateTime(gmtime(&now));
}

// Seed the ESP32 system clock from the RTC at boot. Returns true if the RTC held
// a plausible time (year >= 2021) and the system clock was set from it.
bool syncSystemClockFromRtc() {
  if (!M5.Rtc.isEnabled()) return false;
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year < 2021) return false;       // RTC never set / lost power
  struct tm t = dt.get_tm();
  t.tm_isdst = 0;
  time_t epoch = mktime(&t);                    // tm is UTC (ESP32 default TZ)
  if (epoch == (time_t)-1) return false;
  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  return true;
}

// ====================== MAIDENHEAD ======================
void gridToLatLon(const char* mgrid, double &lat, double &lon) {
  String g = String(mgrid); g.toUpperCase();
  if (g.length() < 4) return;
  lon = (g[0]-'A')*20.0 - 180.0 + 1.0;
  lat = (g[1]-'A')*10.0 - 90.0 + 0.5;
  if (g.length() >= 4) {
    lon += (g[2]-'0')*2.0;
    lat += (g[3]-'0')*1.0;
  }
  if (g.length() >= 6) {
    lon += (tolower(g[4])-'a')*(2.0/24.0);
    lat += (tolower(g[5])-'a')*(1.0/24.0);
  }
}

void latLonToGrid(double lat, double lon, char* gridOut) {
  // Normalize longitude to -180..180
  lon = fmod(lon + 180.0, 360.0);
  if (lon < 0) lon += 360.0;
  lon -= 180.0;

  lat = fmax(-90.0, fmin(90.0, lat));

  int fieldLon = (int)((lon + 180.0) / 20.0);
  int fieldLat = (int)((lat + 90.0) / 10.0);

  double squareLon = fmod((lon + 180.0), 20.0) / 2.0;
  double squareLat = fmod((lat + 90.0), 10.0);

  int subsquareLon = (int)(squareLon * 12.0);
  int subsquareLat = (int)(squareLat * 24.0);

  gridOut[0] = 'A' + fieldLon;
  gridOut[1] = 'A' + fieldLat;
  gridOut[2] = '0' + (int)squareLon;
  gridOut[3] = '0' + (int)squareLat;
  gridOut[4] = 'a' + subsquareLon;
  gridOut[5] = 'a' + subsquareLat;
  gridOut[6] = '\0';
}

// ====================== WIFI GEOLOCATION ======================
bool autoLocateViaWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    statusMsg = "WiFi not connected for location";
    drawMainScreen();
    return false;
  }
  HTTPClient http;
  // ip-api.com is reliable, no key needed for moderate use, returns clean JSON
  http.begin("http://ip-api.com/json/?fields=status,lat,lon,city,country");
  http.setTimeout(12000);
  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    DynamicJsonDocument doc(1536);
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc["status"] == "success" && doc.containsKey("lat") && doc.containsKey("lon")) {
      qth_lat = doc["lat"].as<double>();
      qth_lon = doc["lon"].as<double>();
      saveConfig();

      char grid[7];
      latLonToGrid(qth_lat, qth_lon, grid);
      statusMsg = "WiFi Loc: " + String(grid);
      drawMainScreen();
      return true;
    }
  } else {
    http.end();
  }
  statusMsg = "WiFi geolocation failed";
  drawMainScreen();
  return false;
}

void loadConfig() {
  prefs.begin("sattracker", true);
  qth_lat = prefs.getDouble("lat", 40.7128);
  qth_lon = prefs.getDouble("lon", -74.0060);
  selectedNorad = prefs.getString("norad", "25544");
  selectedName = prefs.getString("name", "ISS");
  lastTLETime = prefs.getULong("lastTLE", 0);
  prefs.getString("tle1", currentTLE1, sizeof(currentTLE1));
  prefs.getString("tle2", currentTLE2, sizeof(currentTLE2));
  prefs.end();
}

void saveConfig() {
  prefs.begin("sattracker", false);
  prefs.putDouble("lat", qth_lat);
  prefs.putDouble("lon", qth_lon);
  prefs.putString("norad", selectedNorad);
  prefs.putString("name", selectedName);
  prefs.end();
}

// ====================== GP ORBITAL ELEMENTS -> TLE ======================
// AMSAT's daily-bulletin.json "tle" text field is being deprecated (the 5-digit
// NORAD catalog runs out ~2026, after which new objects get 6-digit catalog
// numbers that don't fit the TLE text format). The JSON still carries every
// discrete GP/OMM orbital element, so we rebuild the two standard SGP4 TLE lines
// from those fields. The reconstructed lines are byte-format-identical to the
// classic AMSAT TLE the SGP4 library already accepts.

// Standard TLE checksum over the first 68 columns: digits add their value,
// a '-' adds 1, everything else adds 0; result mod 10.
static char tleChecksum(const char* line) {
  int sum = 0;
  for (int i = 0; i < 68 && line[i]; i++) {
    char c = line[i];
    if (c >= '0' && c <= '9') sum += c - '0';
    else if (c == '-') sum += 1;
  }
  return '0' + (sum % 10);
}

// Format a value into the 8-char TLE assumed-decimal exponential field used by
// BSTAR and the 2nd derivative, e.g. 0.0001293 -> " 12930-3" (= .12930e-3).
static void tleExpField(double value, char* out) {
  if (value == 0.0 || !isfinite(value)) { strcpy(out, " 00000-0"); return; }
  char sign = (value < 0) ? '-' : ' ';
  double a = fabs(value);
  int exp = 0;
  while (a >= 1.0) { a /= 10.0; exp++; }
  while (a < 0.1)  { a *= 10.0; exp--; }
  long mant = lround(a * 100000.0);
  if (mant >= 100000) { mant /= 10; exp++; }   // guard rounding overflow
  sprintf(out, "%c%05ld%c%d", sign, mant, (exp < 0) ? '-' : '+', abs(exp));
}

// Convert "YYYY-MM-DD HH:MM:SS.ssssss" (also accepts the 'T' separator) into a
// 2-digit epoch year and fractional day-of-year, as needed for TLE columns 19-32.
static bool parseGPEpoch(const char* epoch, int &yy, double &doy) {
  int Y, Mo, D, h, m; double s = 0.0;
  int n = sscanf(epoch, "%d-%d-%d %d:%d:%lf", &Y, &Mo, &D, &h, &m, &s);
  if (n < 6) n = sscanf(epoch, "%d-%d-%dT%d:%d:%lf", &Y, &Mo, &D, &h, &m, &s);
  if (n < 5) return false;
  static const int cum[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  bool leap = (Y % 4 == 0 && (Y % 100 != 0 || Y % 400 == 0));
  int day = cum[Mo - 1] + D + ((leap && Mo > 2) ? 1 : 0);
  doy = (double)day + (h * 3600.0 + m * 60.0 + s) / 86400.0;
  yy = Y % 100;
  return true;
}

// Build standard TLE line1/line2 (NUL-terminated, 69 cols) from a GP JSON object.
// Returns false if the mandatory elements are missing/invalid.
static bool buildTLEFromGP(JsonObject s, char* tle1, char* tle2) {
  const char* epochC = s["EPOCH"] | "";
  if (strlen(epochC) < 10) return false;

  int yy; double doy;
  if (!parseGPEpoch(epochC, yy, doy)) return false;

  double mm = s["MEAN_MOTION"].as<double>();
  if (mm <= 0.0) return false;  // no usable elements

  long catnum = s["NORAD_CAT_ID"].as<long>();
  // SGP4 math never uses the catalog number; clamp to 5 cols so the fixed-width
  // TLE layout stays valid even once 6-digit (100000+) catalog numbers appear.
  long catCol = (catnum > 99999) ? (catnum % 100000) : catnum;

  char cls = ((const char*)(s["CLASSIFICATION_TYPE"] | "U"))[0];
  if (cls == 0) cls = 'U';
  char eph = ((const char*)(s["EPHEMERIS_TYPE"] | "0"))[0];
  if (eph == 0) eph = '0';

  // International designator from OBJECT_ID "YYYY-NNNAAA" -> "YYNNNAAA" (8 cols).
  char intl[9] = "        ";
  const char* oid = s["OBJECT_ID"] | "";
  const char* dash = strchr(oid, '-');
  if (dash && (dash - oid) >= 4) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%c%c%s", oid[2], oid[3], dash + 1);
    snprintf(intl, sizeof(intl), "%-8.8s", tmp);
  }

  double ndot    = s["MEAN_MOTION_DOT"].as<double>();
  double ndotdot = s["MEAN_MOTION_DDOT"].as<double>();
  double bstar   = s["BSTAR"].as<double>();
  double incl    = s["INCLINATION"].as<double>();
  double raan    = s["RA_OF_ASC_NODE"].as<double>();
  double ecc     = s["ECCENTRICITY"].as<double>();
  double argp    = s["ARG_OF_PERICENTER"].as<double>();
  double ma      = s["MEAN_ANOMALY"].as<double>();
  long   elset   = (long)(s["ELEMENT_SET_NO"] | 0.0);
  long   revnum  = s["REV_AT_EPOCH"].as<long>() % 100000;

  char ndotStr[16];
  sprintf(ndotStr, "%c.%08ld", (ndot < 0) ? '-' : ' ', lround(fabs(ndot) * 1e8));
  char ddotStr[10], bstarStr[10];
  tleExpField(ndotdot, ddotStr);
  tleExpField(bstar, bstarStr);

  // ---- Line 1 (68-char body + checksum) ----
  sprintf(tle1, "1 %5ld%c %-8s %02d%012.8f %s %s %s %c %4ld",
          catCol, cls, intl, yy, doy, ndotStr, ddotStr, bstarStr, eph, elset);
  tle1[68] = tleChecksum(tle1);
  tle1[69] = '\0';

  // ---- Line 2 (eccentricity is 7 digits with an assumed leading decimal) ----
  long eccCol = lround(ecc * 1e7);
  sprintf(tle2, "2 %5ld %8.4f %8.4f %07ld %8.4f %8.4f %11.8f%5ld",
          catCol, incl, raan, eccCol, argp, ma, mm, revnum);
  tle2[68] = tleChecksum(tle2);
  tle2[69] = '\0';

  return true;
}

// ====================== GP Data from AMSAT daily-bulletin.json (orbital elements -> TLE for SGP4) ======================
bool parseGPJson(const String& payload) {
  satCount = 0;
  DynamicJsonDocument doc(49152);  // Sufficient for ~89K daily-bulletin.json; uses PSRAM on M5Paper S3 if available
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    // JSON parse failed; could log but no Serial in normal run
    return false;
  }

  JsonArray sats;
  if (doc.is<JsonArray>()) {
    sats = doc.as<JsonArray>();
  } else if (doc["satellites"].is<JsonArray>()) {
    sats = doc["satellites"].as<JsonArray>();
  } else if (doc["data"].is<JsonArray>()) {
    sats = doc["data"].as<JsonArray>();
  } else if (doc["GP"].is<JsonArray>()) {
    sats = doc["GP"].as<JsonArray>();
  } else if (doc["elements"].is<JsonArray>()) {
    sats = doc["elements"].as<JsonArray>();
  } else {
    return false; // unknown structure
  }

  for (JsonObject s : sats) {
    if (satCount >= 200) break;
    const char* nameC = s["AMSAT_NAME"] | s["OBJECT_NAME"] | s["name"] | s["SATNAME"] | s["title"] | "";
    const char* noradC = s["NORAD_CAT_ID"] | s["norad"] | s["CATNR"] | s["NORAD"] | s["id"] | "";
    String nameStr(nameC);
    String noradStr(noradC);
    nameStr.trim();
    noradStr.trim();
    if (nameStr.length() > 0 && noradStr.length() > 0) {
      nameStr.toCharArray(satList[satCount].name, sizeof(satList[satCount].name));
      noradStr.toCharArray(satList[satCount].norad, sizeof(satList[satCount].norad));

      // If this is the currently selected satellite, build its TLE from the
      // discrete GP orbital-element fields (MEAN_MOTION, ECCENTRICITY, EPOCH,
      // INCLINATION, etc.) instead of the deprecated "tle" text field.
      if (noradStr == selectedNorad) {
        char t1[80], t2[80];
        bool built = buildTLEFromGP(s, t1, t2);

        // Transitional fallback: if the GP element fields are absent but the
        // legacy "tle" text field is still present, parse that instead.
        if (!built) {
          String tleStr = s["tle"] | "";
          if (tleStr.length() > 20) {
            int firstNewline = tleStr.indexOf('\n');
            if (firstNewline > 0) {
              int secondNewline = tleStr.indexOf('\n', firstNewline + 1);
              if (secondNewline > firstNewline) {
                String line1 = tleStr.substring(firstNewline + 1, secondNewline);
                String line2 = tleStr.substring(secondNewline + 1);
                line1.trim();
                line2.trim();
                line1.toCharArray(t1, sizeof(t1));
                line2.toCharArray(t2, sizeof(t2));
                built = (strlen(t1) > 60 && strlen(t2) > 60);
              }
            }
          }
        }

        if (built) {
          strncpy(currentTLE1, t1, sizeof(currentTLE1));
          strncpy(currentTLE2, t2, sizeof(currentTLE2));
          currentTLE1[sizeof(currentTLE1) - 1] = '\0';
          currentTLE2[sizeof(currentTLE2) - 1] = '\0';

          // Also save to Preferences for offline use
          prefs.begin("sattracker", false);
          prefs.putString("tle1", currentTLE1);
          prefs.putString("tle2", currentTLE2);
          prefs.end();
        }
      }

      satCount++;
    }
  }
  return satCount > 0;
}

bool fetchTLE() {
  bool haveLocal = LittleFS.exists("/daily-bulletin.json");
  time_t now = time(nullptr);
  bool timeValid = (now > TLE_TIME_VALID_THRESHOLD);

  bool needDownload = false;
  bool forceThisTime = forceTLEUpdate;
  forceTLEUpdate = false;  // reset after this call
  if (WiFi.status() == WL_CONNECTED) {
    if (forceThisTime || lastTLETime == 0 || (timeValid && (now - lastTLETime > 86400))) {
      needDownload = true;
    } else if (!haveLocal) {
      // No local cache file persisted (e.g. LittleFS write issue); retry periodically (every ~1h)
      // to acquire GP data without hammering AMSAT server every 60s on repeated refresh
      if (lastTLETime == 0 || (timeValid && (now - lastTLETime > 3600))) {
        needDownload = true;
      }
    }
  }

  if (needDownload && WiFi.status() == WL_CONNECTED) {
    statusMsg = "Downloading AMSAT GP data...";
    drawMainScreen();

    HTTPClient http;
    http.begin("https://newark192.amsat.org/gpdata/current/daily-bulletin.json");
    http.setTimeout(25000);
    int code = http.GET();

    if (code == HTTP_CODE_OK) {
      String payload = http.getString();
      http.end();

      // Save to local LittleFS for offline use (the curated amateur satellite GP bulletin)
      File f = LittleFS.open("/daily-bulletin.json", "w");
      if (f) {
        f.print(payload);
        f.close();
      }

      if (now > TLE_TIME_VALID_THRESHOLD) {
        lastTLETime = now;
        prefs.begin("sattracker", false);
        prefs.putULong("lastTLE", (unsigned long)lastTLETime);
        prefs.end();
      } else {
        lastTLETime = 1;  // non-zero sentinel (RAM only)
      }

      if (parseGPJson(payload)) {
        // Successfully parsed satellite list + TLE directly from AMSAT GP bulletin.
        // No Celestrak call is made (purely local parsing).
        if (strlen(currentTLE1) > 60 && strlen(currentTLE2) > 60) {
          struct tm t = *gmtime(&now);
          char msg[50];
          sprintf(msg, "GP Data %02d/%02d/%04d %02d:%02d UTC", 
                  t.tm_mon + 1, t.tm_mday, t.tm_year + 1900,
                  t.tm_hour, t.tm_min);
          statusMsg = msg;
          return true;
        } else {
          statusMsg = "GP list loaded but no TLE for selected satellite";
          return (strlen(currentTLE1) > 10); // use cached TLE if available
        }
      } else {
        statusMsg = "GP JSON parse failed after download";
        return false;
      }
    } else {
      http.end();
      statusMsg = "Download failed (code " + String(code) + "), using local...";
      if (now > TLE_TIME_VALID_THRESHOLD) {
        lastTLETime = now;
        prefs.begin("sattracker", false);
        prefs.putULong("lastTLE", (unsigned long)lastTLETime);
        prefs.end();
      } else {
        lastTLETime = 1;
      }
      // fall through to local load
    }
  }

  // Fallback / normal path: load from local JSON file (offline operation with last known GP list + cached TLE lines)
  if (haveLocal) {
    File f = LittleFS.open("/daily-bulletin.json", "r");
    if (f) {
      String payload = f.readString();
      f.close();
      if (parseGPJson(payload)) {
        if (needDownload) {
          statusMsg = "Using local GP data (update failed)";
        } else {
          statusMsg = "Using cached local GP data";
        }
        return (strlen(currentTLE1) > 10);
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED && !haveLocal) {
    statusMsg = "No Wifi & No local GP data";
  } else {
    statusMsg = "No GP data available";
  }
  return false;
}

void predictPasses() {
  passCount = 0;
  sat.site(qth_lat, qth_lon, qth_alt);

  passinfo p;
  // Use library's pass finder (reliable for detecting passes, maxEl, and LOS/jdstop)
  // but compute AOS manually from the peak because library jdstart is buggy for some sats like RS-44
  sat.initpredpoint((unsigned long)time(nullptr), 0.0);

  while (passCount < 8) {
    if (!sat.nextpass(&p, 40, false, 0.0)) {
      break; // no more passes found
    }
    // Only accept passes with reasonable max elevation and valid stop > max time
    if (p.maxelevation > 0.5 && p.jdstop > p.jdmax) {
      // Library provides good p.jdmax, p.jdstop (LOS), p.maxelevation
      // Ignore p.jdstart (often wrongly equals LOS or peak time)
      time_t peakTime = jdToUnix(p.jdmax);
      time_t losTimeLib = jdToUnix(p.jdstop);

      // Manually find AOS by searching backward from peak until elevation drops to <=0
      time_t aosTime = peakTime;
      const long COARSE_BACK_SEC = 30;
      bool crossedBelow = false;
      for (long back = 0; back < 2 * 3600; back += COARSE_BACK_SEC) { // search up to 2 hours back
        time_t tt = peakTime - back;
        if (tt < time(nullptr) - 3600) break;
        sat.findsat((unsigned long)tt);
        if (sat.satEl <= 0.0) {
          aosTime = tt;
          crossedBelow = true;
          break;
        }
      }
      if (!crossedBelow) {
        continue; // couldn't find AOS, skip this pass
      }

      // Refine AOS: step forward from the rough below point to find first time El > 0
      time_t refinedAOS = aosTime;
      for (int d = 0; d < 120; ++d) { // up to 2 minutes refine window
        time_t tt = aosTime + d;
        sat.findsat((unsigned long)tt);
        if (sat.satEl > 0.0) {
          refinedAOS = tt;
          break;
        }
      }

      // Use library's LOS (confirmed correct by user) and maxEl
      time_t refinedLOS = losTimeLib;

      if (refinedLOS > refinedAOS + 30) {
        passes[passCount].aos = refinedAOS;
        passes[passCount].los = refinedLOS;
        passes[passCount].maxEl = p.maxelevation;
        passCount++;
      }
    }
  }
}

void updateCurrentPosition() {
  sat.findsat((unsigned long)time(nullptr));
}

void updateData() {
  if (WiFi.status() != WL_CONNECTED) {
    statusMsg = "WiFi disconnected - retrying...";
    WiFi.begin();
    delay(1500);
  }
  if (fetchTLE()) {
    sat.init(selectedName.c_str(), currentTLE1, currentTLE2);
    sat.site(qth_lat, qth_lon, qth_alt);
    predictPasses();
    updateCurrentPosition();
    statusMsg = "Tracking " + selectedName;
  }
  lastUpdate = millis();
}

// ====================== BATTERY ======================
int getBatteryPercent() {
  float voltage = M5.Power.getBatteryVoltage() / 1000.0;
  int percent = (voltage - 3.4) / (4.2 - 3.4) * 100;
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  return percent;
}

// ====================== BUTTON-NAV HELPERS ======================
// Draw a full-width menu row; highlighted rows get an inverted (filled) style so
// the selection is obvious on a slow color EPD without needing a blinking cursor.
void drawMenuItem(int x, int y, int w, int h, const char* label, bool selected) {
  if (selected) {
    M5.Display.fillRoundRect(x, y, w, h, 6, COL_HILITE);
    M5.Display.setTextColor(COL_BG);
  } else {
    M5.Display.drawRoundRect(x, y, w, h, 6, COL_FG);
    M5.Display.setTextColor(COL_FG);
  }
  M5.Display.drawString(label, x + 12, y + (h - 16) / 2 + 2);
  M5.Display.setTextColor(COL_FG);
}

// Hint bar drawn at the very bottom of every screen so the button mapping for the
// current context is always visible (the buttons themselves are unlabeled).
void drawButtonHints(const char* a, const char* b, const char* c) {
  int y = SCR_H - 22;
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(COL_FG);
  M5.Display.drawFastHLine(0, y - 4, SCR_W, COL_FG);
  char line[64];
  snprintf(line, sizeof(line), "A:%s   B:%s   C:%s", a, b, c);
  M5.Display.drawString(line, 8, y + 2);
}

// ====================== SATELLITE ICON ======================
void drawSatelliteIcon(int x, int y, int size) {
  M5.Display.fillRect(x - size/2, y - size/2, size, size, COL_ACCENT);
}

// ====================== MAIN SCREEN (portrait 400x600) ======================
// Focusable items: 0=Refresh, 1=Select Sat, 2=Setup.
void drawMainScreen() {
  itemCount = 3;
  if (selIndex < 0) selIndex = 0;
  if (selIndex >= itemCount) selIndex = itemCount - 1;

  M5.Display.clearDisplay();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_FG);

  // --- Header ---
  M5.Display.setTextSize(2);
  M5.Display.drawString("PaperSatColor", 12, 8);
  M5.Display.setTextColor(sat.satEl > 0 ? COL_ACCENT : COL_FG);
  M5.Display.drawString(selectedName.c_str(), 12, 34);
  M5.Display.setTextColor(COL_FG);

  // Time + battery on the header's right side
  M5.Display.setTextSize(1);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  char ts[20];
  sprintf(ts, "%02d:%02d UTC", timeinfo.tm_hour, timeinfo.tm_min);
  M5.Display.drawString(ts, 250, 10);
  char bat[10];
  sprintf(bat, "%d%%", getBatteryPercent());
  M5.Display.drawString(bat, 250, 26);

  // --- Polar plot (centered, upper area) ---
  int cx = SCR_W / 2, cy = 230, r = 150;
  M5.Display.drawCircle(cx, cy, r, COL_FG);
  M5.Display.drawCircle(cx, cy, r/2, COL_FG);
  M5.Display.drawCircle(cx, cy, 12, COL_FG);

  for (int i = 0; i < 8; i++) {
    float angle = i * 45.0 * PI / 180.0;
    int x1 = cx + (int)(r * 0.2 * sin(angle));
    int y1 = cy - (int)(r * 0.2 * cos(angle));
    int x2 = cx + (int)(r * sin(angle));
    int y2 = cy - (int)(r * cos(angle));
    M5.Display.drawLine(x1, y1, x2, y2, COL_FG);
  }

  M5.Display.setTextSize(1);
  M5.Display.drawString("N", cx-3, cy-r-14);
  M5.Display.drawString("S", cx-3, cy+r+5);
  M5.Display.drawString("E", cx+r+8, cy-3);
  M5.Display.drawString("W", cx-r-14, cy-3);

  // Determine which pass to plot
  int passToPlot = -1;
  if (passCount > 0) {
    time_t now = time(nullptr);
    if (sat.satEl > 0) {
      for (int i = 0; i < passCount; i++) {
        if (passes[i].aos <= now && passes[i].los >= now) { passToPlot = i; break; }
      }
    } else {
      passToPlot = 0;
    }
  }

  // Pass arc (blue)
  if (passToPlot >= 0) {
    time_t startT = passes[passToPlot].aos;
    time_t endT = passes[passToPlot].los;
    long duration = endT - startT;
    if (duration > 0) {
      const int numPoints = 36;
      long step = duration / (numPoints - 1);
      if (step < 15) step = 15;
      int prevX = -1, prevY = -1;
      double savedAz = sat.satAz;
      double savedEl = sat.satEl;
      for (int i = 0; i < numPoints; i++) {
        time_t t = startT + (long)i * step;
        if (t > endT) t = endT;
        sat.findsat((unsigned long)t);
        if (sat.satEl > 0.0) {
          double az = sat.satAz * PI / 180.0;
          double eln = (90.0 - sat.satEl) / 90.0;
          int px = cx + (int)(r * eln * sin(az));
          int py = cy - (int)(r * eln * cos(az));
          if (prevX >= 0) M5.Display.drawLine(prevX, prevY, px, py, COL_PATH);
          prevX = px; prevY = py;
        } else {
          prevX = -1;
        }
      }
      sat.satAz = savedAz;
      sat.satEl = savedEl;
    }
  }

  // Current satellite position (red) + direction arrow
  if (sat.satEl > 0) {
    double az = sat.satAz * PI / 180.0;
    double eln = (90.0 - sat.satEl) / 90.0;
    int px = cx + (int)(r * eln * sin(az));
    int py = cy - (int)(r * eln * cos(az));
    drawSatelliteIcon(px, py, 14);

    time_t nowT = time(nullptr);
    time_t futureT = nowT + 45;
    if (passToPlot >= 0 && passes[passToPlot].los < futureT + 10) {
      futureT = passes[passToPlot].los - 5;
    }
    if (futureT > nowT + 5) {
      double savedAz2 = sat.satAz;
      double savedEl2 = sat.satEl;
      sat.findsat((unsigned long)futureT);
      if (sat.satEl > 0.0) {
        double azf = sat.satAz * PI / 180.0;
        double elnf = (90.0 - sat.satEl) / 90.0;
        int pxf = cx + (int)(r * elnf * sin(azf));
        int pyf = cy - (int)(r * elnf * cos(azf));
        int dx = pxf - px;
        int dy = pyf - py;
        double len = sqrt(dx * dx + dy * dy);
        if (len > 3.0) {
          double scale = 20.0 / len;
          int ax = px + (int)(dx * scale);
          int ay = py + (int)(dy * scale);
          M5.Display.drawLine(px, py, ax, ay, COL_ACCENT);
          double angle = atan2(dy, dx);
          double asz = 9.0;
          int hx1 = ax - (int)(asz * cos(angle - 0.4));
          int hy1 = ay - (int)(asz * sin(angle - 0.4));
          int hx2 = ax - (int)(asz * cos(angle + 0.4));
          int hy2 = ay - (int)(asz * sin(angle + 0.4));
          M5.Display.drawLine(ax, ay, hx1, hy1, COL_ACCENT);
          M5.Display.drawLine(ax, ay, hx2, hy2, COL_ACCENT);
        }
      }
      sat.satAz = savedAz2;
      sat.satEl = savedEl2;
    }
  }

  // --- Az / El line below the plot ---
  M5.Display.setTextSize(1);
  int infoY = cy + r + 16;
  char buf[32];
  sprintf(buf, "Az: %.1f", sat.satAz);
  M5.Display.drawString(buf, 12, infoY);
  int x = 12 + M5.Display.textWidth(buf);
  drawDegreeSymbol(x, infoY);
  sprintf(buf, "   El: %.1f", sat.satEl);
  x += 12;
  M5.Display.drawString(buf, x, infoY);
  x += M5.Display.textWidth(buf);
  drawDegreeSymbol(x, infoY);

  // --- Next passes ---
  int passY = infoY + 20;
  M5.Display.drawString("Next Passes (UTC):", 12, passY);
  for (int i = 0; i < passCount && i < 3; i++) {
    struct tm aos_tm = *gmtime(&passes[i].aos);
    struct tm los_tm = *gmtime(&passes[i].los);
    char line[80];
    sprintf(line, "%02d:%02d:%02d > %02d:%02d:%02d  %.0f",
            aos_tm.tm_hour, aos_tm.tm_min, aos_tm.tm_sec,
            los_tm.tm_hour, los_tm.tm_min, los_tm.tm_sec, passes[i].maxEl);
    M5.Display.drawString(line, 12, passY + 16 + i*15);
    int px = 12 + M5.Display.textWidth(line);
    drawDegreeSymbol(px, passY + 16 + i*15);
  }

  // --- Status + GP timestamp ---
  int statusY = passY + 16 + 3*15 + 6;
  if (lastTLETime > TLE_TIME_VALID_THRESHOLD) {
    struct tm t = *gmtime(&lastTLETime);
    char tleStr[50];
    sprintf(tleStr, "GP %02d/%02d %02d:%02dz",
            t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
    M5.Display.drawString(tleStr, 12, statusY);
  }
  M5.Display.drawString(statusMsg.c_str(), 12, statusY + 14);

  // --- Action row (3 items, highlighted by selIndex) ---
  int btnY = SCR_H - 70;
  int bw = (SCR_W - 4*8) / 3;   // three across with 8px gutters
  drawMenuItem(8 + 0*(bw+8), btnY, bw, 40, "Refresh",  selIndex == 0);
  drawMenuItem(8 + 1*(bw+8), btnY, bw, 40, "Sat List", selIndex == 1);
  drawMenuItem(8 + 2*(bw+8), btnY, bw, 40, "Setup",    selIndex == 2);

  drawButtonHints("Prev", "Select", "Next");
  M5.Display.display();
}

// ====================== OTHER SCREENS ======================
// SAT_SELECT focusable items: 0..numOnPage-1 = sat rows on this page;
// then the action buttons: [Prev][Next][Update GP][Back] (Prev/Next only
// present when there are multiple pages, but kept in the index space for
// simplicity and skipped on activation when not applicable).
// Helper exposes how many sat rows are on the current page.
int satSelectNumOnPage() {
  int startIdx = currentSatPage * satsPerPage;
  int remaining = satCount - startIdx;
  if (remaining < 0) remaining = 0;
  return (satsPerPage < remaining ? satsPerPage : remaining);
}

void drawSatSelectScreen() {
  int numOnPage = satSelectNumOnPage();
  int totalPages = (satCount + satsPerPage - 1) / satsPerPage;
  if (totalPages < 1) totalPages = 1;

  // Item layout: sat rows + 4 actions (Prev, Next, Update GP, Back).
  itemCount = numOnPage + 4;
  if (selIndex < 0) selIndex = 0;
  if (selIndex >= itemCount) selIndex = itemCount - 1;

  M5.Display.clearDisplay();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_FG);
  M5.Display.setTextSize(2);
  M5.Display.drawString("Select Satellite", 12, 8);
  M5.Display.setTextSize(1);
  char pageStr[20];
  sprintf(pageStr, "Page %d/%d", currentSatPage + 1, totalPages);
  M5.Display.drawString(pageStr, 280, 14);

  int startIdx = currentSatPage * satsPerPage;
  for (int j = 0; j < numOnPage; j++) {
    int i = startIdx + j;
    int y = 40 + j * 34;
    drawMenuItem(12, y, SCR_W - 24, 30, satList[i].name, selIndex == j);
  }

  // Action row: two columns x two rows below the list.
  int aY = 40 + 10 * 34 + 8;   // below max-height list (10 rows)
  int aw = (SCR_W - 3*8) / 2;
  int prevIdx   = numOnPage + 0;
  int nextIdx   = numOnPage + 1;
  int updateIdx = numOnPage + 2;
  int backIdx   = numOnPage + 3;
  drawMenuItem(8,            aY,      aw, 36, "< Prev Page", selIndex == prevIdx);
  drawMenuItem(8 + aw + 8,   aY,      aw, 36, "Next Page >", selIndex == nextIdx);
  drawMenuItem(8,            aY + 44, aw, 36, "Update GP",   selIndex == updateIdx);
  drawMenuItem(8 + aw + 8,   aY + 44, aw, 36, "Back",        selIndex == backIdx);

  drawButtonHints("Prev", "Select", "Next");
  M5.Display.display();
}

void drawSetupMenu() {
  itemCount = 6;  // 5 options + Back
  if (selIndex < 0) selIndex = 0;
  if (selIndex >= itemCount) selIndex = itemCount - 1;

  M5.Display.clearDisplay();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_FG);
  M5.Display.setTextSize(2);
  M5.Display.drawString("Setup", 12, 8);
  M5.Display.setTextSize(1);

  const char* labels[] = {
    "Enter Maidenhead Grid",
    "Enter Lat / Lon",
    "WiFi Configuration",
    "Auto Location via WiFi",
    "Set Time/Date (UTC)",
    "Back"
  };
  for (int i = 0; i < 6; i++) {
    int y = 48 + i * 50;
    drawMenuItem(12, y, SCR_W - 24, 40, labels[i], selIndex == i);
  }

  drawButtonHints("Prev", "Select", "Next");
  M5.Display.display();
}

// ---- Shared character-scroll text-entry renderer ----
// Shows the prompt, the buffer being built, and a big "wheel" of characters with
// the one at charCursor highlighted. The set includes <DEL> and <DONE> as the
// final two scrollable entries so all editing is reachable with A/C scroll + B.
void drawCharEntry(const char* prompt, const char* prompt2, const char* charset) {
  M5.Display.clearDisplay();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_FG);

  M5.Display.setTextSize(1);
  M5.Display.drawString(prompt, 12, 12);
  if (prompt2 && prompt2[0]) M5.Display.drawString(prompt2, 12, 28);

  // Current buffer
  M5.Display.setTextSize(3);
  M5.Display.drawString(inputBuffer.length() ? inputBuffer.c_str() : "_", 16, 60);
  M5.Display.setTextSize(1);

  // Build the full selectable list = charset chars + DEL + DONE.
  int n = strlen(charset);
  int total = n + 2;            // + DEL + DONE
  itemCount = total;           // handleButtons scrolls over this range
  if (charCursor < 0) charCursor = total - 1;
  if (charCursor >= total) charCursor = 0;

  // Render a horizontal wheel of ~7 entries centered on charCursor.
  int midX = SCR_W / 2;
  int cellW = 54;
  int wheelY = 200;
  M5.Display.drawString("Character:", 12, wheelY - 26);
  for (int off = -3; off <= 3; off++) {
    int idx = ((charCursor + off) % total + total) % total;
    char label[8];
    if (idx < n)            { label[0] = charset[idx]; label[1] = '\0'; }
    else if (idx == n)      strcpy(label, "DEL");
    else                    strcpy(label, "OK");
    int cxCell = midX + off * cellW;
    bool sel = (off == 0);
    int cw = sel ? 48 : 40;
    int ch = sel ? 54 : 44;
    int cy = wheelY + (sel ? 0 : 6);
    if (sel) {
      M5.Display.fillRoundRect(cxCell - cw/2, cy, cw, ch, 6, COL_HILITE);
      M5.Display.setTextColor(COL_BG);
      M5.Display.setTextSize(idx < n ? 3 : 2);
    } else {
      M5.Display.drawRoundRect(cxCell - cw/2, cy, cw, ch, 6, COL_FG);
      M5.Display.setTextColor(COL_FG);
      M5.Display.setTextSize(idx < n ? 2 : 1);
    }
    int tw = M5.Display.textWidth(label);
    M5.Display.drawString(label, cxCell - tw/2, cy + ch/2 - 10);
    M5.Display.setTextColor(COL_FG);
    M5.Display.setTextSize(1);
  }

  drawButtonHints("Prev char", "Pick", "Next char");
  M5.Display.display();
}

void drawGridInputScreen() {
  drawCharEntry("Enter Maidenhead Grid (4 or 6 chars).",
                "Scroll to OK to confirm, DEL to erase.", GRID_CHARS);
}

void drawLatLonInputScreen() {
  drawCharEntry("Enter Lat,Lon  e.g. 40.7128,-74.0060",
                "Scroll to OK to confirm, DEL to erase.", LATLON_CHARS);
}

void drawTimeInputScreen() {
  drawCharEntry("Set UTC: YYYY-MM-DD HH:MM:SS (or T sep)",
                "Scroll to OK to confirm, DEL to erase.", TIME_CHARS);
}

// ---- Commit handlers (shared by the entry screens' OK action) ----
void commitGrid() {
  if (inputBuffer.length() >= 4) gridToLatLon(inputBuffer.c_str(), qth_lat, qth_lon);
  saveConfig();
}

void commitLatLon() {
  if (inputBuffer.length() > 0) {
    int comma = inputBuffer.indexOf(',');
    if (comma > 0) {
      qth_lat = inputBuffer.substring(0, comma).toDouble();
      qth_lon = inputBuffer.substring(comma + 1).toDouble();
      saveConfig();
    }
  }
}

void commitTime() {
  int y, m, d, h, mi, s = 0;
  int nn = sscanf(inputBuffer.c_str(), "%d-%d-%d %d:%d:%d", &y, &m, &d, &h, &mi, &s);
  if (nn < 6) nn = sscanf(inputBuffer.c_str(), "%d-%d-%dT%d:%d:%d", &y, &m, &d, &h, &mi, &s);
  if (nn < 6) { nn = sscanf(inputBuffer.c_str(), "%d-%d-%d %d:%d", &y, &m, &d, &h, &mi); if (nn == 5) s = 0; }
  if (nn >= 5 && y >= 2020 && y <= 2100 && m >= 1 && m <= 12 && d >= 1 && d <= 31 &&
      h >= 0 && h <= 23 && mi >= 0 && mi <= 59 && s >= 0 && s <= 59) {
    setSystemTime(y, m, d, h, mi, s);
    statusMsg = "Time set successfully (UTC)";
  } else {
    statusMsg = "Invalid format. Use YYYY-MM-DD HH:MM:SS";
  }
}


// ====================== SCREEN DISPATCH ======================
void redrawCurrent() {
  switch (currentScreen) {
    case MAIN:         drawMainScreen();        break;
    case SAT_SELECT:   drawSatSelectScreen();   break;
    case SETUP_MENU:   drawSetupMenu();         break;
    case GRID_INPUT:   drawGridInputScreen();   break;
    case LATLON_INPUT: drawLatLonInputScreen(); break;
    case TIME_INPUT:   drawTimeInputScreen();   break;
  }
}

// Reset navigation state when entering a screen.
void enterScreen(Screen s) {
  currentScreen = s;
  selIndex = 0;
  charCursor = 0;
  redrawCurrent();
}

// ====================== BUTTON HANDLER ======================
// BtnA = Prev/Up, BtnC = Next/Down, BtnB = Select (short) / Back (long-hold).
// Each branch performs the action and redraws exactly once (every redraw is a
// slow full EPD refresh, so we never redraw more than necessary per press).
void handleButtons() {
  bool up    = M5.BtnA.wasClicked();
  bool down  = M5.BtnC.wasClicked();
  bool sel   = M5.BtnB.wasClicked();
  bool back  = M5.BtnB.wasHold();   // long-press B = Back

  if (!up && !down && !sel && !back) return;

  // ---------- Text-entry screens use the char wheel ----------
  if (currentScreen == GRID_INPUT || currentScreen == LATLON_INPUT || currentScreen == TIME_INPUT) {
    const char* charset = (currentScreen == GRID_INPUT) ? GRID_CHARS
                        : (currentScreen == LATLON_INPUT) ? LATLON_CHARS : TIME_CHARS;
    int n = strlen(charset);
    int total = n + 2;  // + DEL + OK

    if (back) { enterScreen(SETUP_MENU); return; }
    if (up)   { charCursor = (charCursor - 1 + total) % total; redrawCurrent(); return; }
    if (down) { charCursor = (charCursor + 1) % total; redrawCurrent(); return; }
    if (sel) {
      if (charCursor < n) {                       // append a real character
        size_t cap = (currentScreen == GRID_INPUT) ? 6 : 19;
        if (inputBuffer.length() < cap) inputBuffer += charset[charCursor];
        redrawCurrent();
      } else if (charCursor == n) {               // DEL
        if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
        redrawCurrent();
      } else {                                    // OK -> commit + return to MAIN
        if (currentScreen == GRID_INPUT)   commitGrid();
        else if (currentScreen == LATLON_INPUT) commitLatLon();
        else                                commitTime();
        currentScreen = MAIN; selIndex = 0;
        updateData();
        drawMainScreen();
      }
      return;
    }
    return;
  }

  // ---------- Menu/list screens use selIndex ----------
  if (up)   { selIndex = (selIndex - 1 + itemCount) % itemCount; redrawCurrent(); return; }
  if (down) { selIndex = (selIndex + 1) % itemCount; redrawCurrent(); return; }

  if (currentScreen == MAIN) {
    if (back) return;  // already at top level
    if (sel) {
      if (selIndex == 0)      { updateData(); drawMainScreen(); }
      else if (selIndex == 1) { currentSatPage = 0; enterScreen(SAT_SELECT); }
      else                    { enterScreen(SETUP_MENU); }
    }
    return;
  }

  if (currentScreen == SAT_SELECT) {
    if (back) { enterScreen(MAIN); return; }
    if (sel) {
      int numOnPage = satSelectNumOnPage();
      int totalPages = (satCount + satsPerPage - 1) / satsPerPage;
      if (totalPages < 1) totalPages = 1;
      if (selIndex < numOnPage) {                 // pick a satellite
        int i = currentSatPage * satsPerPage + selIndex;
        selectedName = satList[i].name;
        selectedNorad = satList[i].norad;
        saveConfig();
        currentTLE1[0] = '\0';
        currentTLE2[0] = '\0';
        currentScreen = MAIN; selIndex = 0;
        updateData();
        drawMainScreen();
      } else {
        int action = selIndex - numOnPage;        // 0=Prev,1=Next,2=Update,3=Back
        if (action == 0) {                        // Prev page
          if (currentSatPage > 0) { currentSatPage--; selIndex = 0; }
          drawSatSelectScreen();
        } else if (action == 1) {                 // Next page
          if (currentSatPage < totalPages - 1) { currentSatPage++; selIndex = 0; }
          drawSatSelectScreen();
        } else if (action == 2) {                 // Update GP
          forceTLEUpdate = true;
          updateData();
          drawSatSelectScreen();
        } else {                                  // Back
          enterScreen(MAIN);
        }
      }
    }
    return;
  }

  if (currentScreen == SETUP_MENU) {
    if (back) { enterScreen(MAIN); return; }
    if (sel) {
      switch (selIndex) {
        case 0: inputBuffer = ""; enterScreen(GRID_INPUT);   break;
        case 1: inputBuffer = ""; enterScreen(LATLON_INPUT); break;
        case 2: openSetupPortal();                            break;
        case 3: autoLocateViaWiFi(); enterScreen(MAIN);       break;
        case 4: inputBuffer = ""; enterScreen(TIME_INPUT);    break;
        case 5: enterScreen(MAIN);                            break;
      }
    }
    return;
  }
}

void openSetupPortal() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  M5.Display.clearDisplay();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_FG);
  M5.Display.setTextSize(2);
  M5.Display.drawString("WiFi Setup Portal", 20, 30);
  M5.Display.setTextSize(1);
  M5.Display.drawString("Connect to: M5PaperColor-Setup", 20, 80);
  M5.Display.drawString("Then open 192.168.4.1 in browser", 20, 110);
  M5.Display.drawString("Portal times out after 5 minutes", 20, 150);
  M5.Display.display();

  wm.startConfigPortal("M5PaperColor-Setup");

  // Portal closed (credentials saved or timeout)
  WiFi.begin();           // reconnect with new creds
  delay(800);             // give WiFi a moment to connect

  statusMsg = "WiFi credentials saved";
  // Bonus: automatically set location from public IP now that we have internet
  if (WiFi.status() == WL_CONNECTED) {
    autoLocateViaWiFi();
  }
  currentScreen = MAIN;
  drawMainScreen();
}

void setup() {
  M5.begin();
  // Native panel is 400(w) x 600(h). Rotation 0 keeps it PORTRAIT.
  // If your unit reads upside-down, use setRotation(2).
  M5.Display.setRotation(0);

  // On the Spectra 6 panel M5GFX picks the 6-color refresh waveform itself, so
  // we don't force a mono fast-update mode here (those exist only on grayscale
  // EPDs). If your M5GFX build exposes epd_mode and you want to pin quality:
  //   M5.Display.setEpdMode(m5gfx::epd_quality);
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(COL_FG);

  // Long-press threshold so BtnB.wasHold() fires for "Back".
  M5.BtnB.setHoldThresh(HOLD_MS);

  if (!LittleFS.begin()) {
    LittleFS.format();  // Attempt to recover corrupted or unformatted LittleFS
    if (!LittleFS.begin()) {
      // LittleFS mount failed; downloads will still work but no offline cache
      statusMsg = "LittleFS mount failed";
    }
  }

  loadConfig();

  // Seed the system clock from the battery-backed RTC so time is valid before we
  // run any orbital math, even with no network yet. NTP (below) will refine it.
  if (syncSystemClockFromRtc()) {
    statusMsg = "Time from RTC";
  }

  WiFi.begin();
  configTime(0, 0, "pool.ntp.org");

  updateData();
  selIndex = 0;
  drawMainScreen();
}

void loop() {
  M5.update();
  handleButtons();

  // Once NTP has set a valid clock, persist it to the RTC a single time so the
  // good time survives power-off. (NTP completes asynchronously after boot.)
  if (!rtcSyncedFromNtp && WiFi.status() == WL_CONNECTED &&
      time(nullptr) > TLE_TIME_VALID_THRESHOLD) {
    writeSystemClockToRtc();
    rtcSyncedFromNtp = true;
  }

  if (currentScreen == MAIN) {
    // Spectra 6 full refreshes are slow (~15-19s) and flash the whole panel, so
    // we auto-refresh rarely. Button presses are handled immediately above; these
    // are just the periodic position/pass updates on the main screen.
    unsigned long currentInterval = (sat.satEl > 0) ? 60000UL : 300000UL;

    if (millis() - lastUpdate > currentInterval) {
      updateData();
      drawMainScreen();
    }
  }
  delay(20);
}
