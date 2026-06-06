// ============================================================================
// PaperSatColor  -  up-to-20-satellite next-pass dashboard for M5Paper Color
// ----------------------------------------------------------------------------
// Hardware: M5Paper Color ESP32-S3 Dev Kit (4" SPECTRA 6 color e-paper, 400x600
// portrait, RX8130CE RTC, Wi-Fi, three USER_KEY buttons, two RGB LEDs, speaker).
//
// Concept: a STATIC dashboard. Up to 20 satellites are tracked across up to 5
// pages of 4, each page a 2x2 grid; each cell shows ONE satellite's next pass as
// a polar (azimuth/elevation) plot of the pass ground track, plus AOS/LOS times
// and the pass maximum azimuth and elevation. Slots may be left blank (the spot
// is left empty) and a page with no satellites is hidden entirely, so the number
// of pages shown matches how many satellites are actually configured. Because
// the Spectra 6 panel takes ~15-19s to refresh and cannot do partial updates,
// the display is drawn only when something actually changes: a tracked pass on
// the current page begins or ends (a "pass event"), the page is switched, a
// manual refresh is pressed, or the configuration is changed.
//
// Buttons (mapped per the board's published USER_KEY spec): USER_KEY1 = manual
// refresh; USER_KEY2/USER_KEY3 = page up/down through the occupied pages.
//
// Configuration is done SOLELY over Wi-Fi: the device serves a web page (its IP
// is shown on screen) with up to 20 dropdowns - populated from the AMSAT
// bulletin satellite list, each with a (blank) option so no slot is forced - to
// choose the tracked satellites, plus optional station location fields. There
// are no on-device configuration menus.
// ============================================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <Sgp4.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <LittleFS.h>
#include <ArduinoJson.h>  // "ArduinoJson" by Benoit Blanchon
#include <FastLED.h>      // RGB LEDs (M5Unified does not drive them)

// ====================== DISPLAY ======================
#define SCR_W 400
#define SCR_H 600

// Spectra 6 gives us six inks: black, white, red, yellow, blue, green. We use
// each one with a consistent meaning so the dashboard reads at a glance:
#define COL_BG       TFT_WHITE
#define COL_FG       TFT_BLACK
#define COL_AOS      TFT_GREEN   // acquisition of signal (rise) - "go"
#define COL_LOS      TFT_RED     // loss of signal (set) - "stop"
#define COL_PEAK     TFT_RED     // peak-elevation marker
#define COL_PATH     TFT_BLUE    // pass ground-track arc
#define COL_HIGH     TFT_RED     // high-elevation (excellent) pass accent
#define COL_MED      TFT_BLUE    // medium-elevation pass accent
#define COL_LOW      TFT_GREEN   // low-elevation (marginal) pass accent
#define COL_NOW      TFT_RED     // "pass in progress" highlight
#define COL_HDR      TFT_BLUE    // header text accents
#define COL_GRID     0x8410      // mid-grey graticule (RGB565)
#define COL_RINGFILL TFT_YELLOW  // soft fill inside the 45-deg ring

// Fonts: M5GFX bundles the GNU FreeFont family (anti-aliased, proportional),
// far nicer than the blocky built-in Font0. We use a bold sans for headings and
// a regular sans for body text. With top_left datum, drawString's y is the top
// of the text (matching the rest of the layout math).
#define FONT_NAME   &fonts::FreeSansBold9pt7b   // satellite name / headings
#define FONT_BODY   &fonts::FreeSans9pt7b       // body text
#define FONT_SMALL  &fonts::FreeSans9pt7b       // small labels (same face)
#define LINE_H      16                          // ~height of 9pt FreeSans line

// ====================== STATION / CONFIG ======================
double qth_lat = 38.8626;
double qth_lon = -77.0562;
double qth_alt = 10.0;

#define PER_PAGE    4     // 2x2 grid per page
#define NUM_PAGES   5     // up to five pages
#define NUM_TRACKED (PER_PAGE * NUM_PAGES)   // up to 20 satellites total
#define ARC_POINTS  24    // polar arc samples per pass

// ---------------- Alerts (LED + sound) ----------------
// The device flashes its two RGB LEDs and plays a tone at four moments around
// every tracked pass: 5 minutes before AOS, 1 minute before AOS, at AOS (rise),
// and at LOS (set). Each alert fires exactly once per pass.
//
// LEDs use FastLED (M5Unified doesn't drive them). VERIFY the data pin against
// the M5Paper Color GPIO map - this is the documented default but boards vary.
#define LED_DATA_PIN  21      // <-- confirm on your unit's pinout
#define NUM_LEDS      2
#define LED_BRIGHTNESS 40     // 0-255; keep modest (heat) and battery-friendly
CRGB leds[NUM_LEDS];
bool ledsReady = false;

// Sound uses the built-in speaker (ES8311 codec + AW8737A amp) via M5.Speaker.
#define ALERT_VOLUME  150     // 0-255

// ---------------- Buttons ----------------
// Three physical buttons via M5Unified, mapped per the board's published
// specification (USER_KEY labels). M5Unified's BtnA/B/C correspond to the
// silkscreen keys as noted below; swap if a future board revision differs.
//   USER_KEY1 = M5.BtnC -> manual refresh (re-fetch + recompute + redraw)
//   USER_KEY2 = M5.BtnB -> page up   (previous page)
//   USER_KEY3 = M5.BtnA -> page down (next page)
#define BTN_REFRESH  M5.BtnC   // USER_KEY1
#define BTN_PAGE_UP  M5.BtnB   // USER_KEY2
#define BTN_PAGE_DN  M5.BtnA   // USER_KEY3

// The four alert offsets relative to AOS (LOS handled separately).
#define ALERT_T5_SEC  (5*60)
#define ALERT_T1_SEC  (1*60)

// Per-pass alert kinds.
enum AlertKind { AL_T5 = 0, AL_T1 = 1, AL_AOS = 2, AL_LOS = 3, AL_COUNT = 4 };

Sgp4 sat;                 // single reused propagator (re-init per satellite)
Preferences prefs;
WebServer server(80);     // config web UI

// Satellite catalog parsed from the AMSAT bulletin (for the web dropdowns).
struct Satellite { char name[25]; char norad[10]; };
Satellite satList[250];
int satCount = 0;

// One tracked dashboard satellite.
struct TrackedSat {
  char  norad[10];
  char  name[25];
  char  tle1[80];
  char  tle2[80];
  bool  haveTLE;
  bool   hasPass;
  time_t aos;
  time_t los;
  double maxEl;        // peak elevation (deg)
  double aosAz;        // azimuth at AOS - where it rises (deg)
  double losAz;        // azimuth at LOS - where it sets (deg)
  int    arcN;
  float  arcAz[ARC_POINTS];
  float  arcEl[ARC_POINTS];
  bool   alerted[AL_COUNT];   // which alerts have fired for the current pass
  time_t nextRoll;            // when to roll this sat forward to its next pass (los+margin)
};
TrackedSat tracked[NUM_TRACKED];

// Default tracked set. Only the first eight slots are pre-filled (pages 1-2);
// the remaining twelve slots (pages 3-5) start blank. Empty string = blank slot.
// Defaults apply only to a fresh/cleared device; the web config can fill or
// blank any slot afterward.
const char* DEFAULT_NORAD[NUM_TRACKED] = {
  "25544", "44909", "7530",  "27607",
  "43017", "22825", "24278", "43678",
  "", "", "", "",
  "", "", "", "",
  "", "", "", "" };
const char* DEFAULT_NAME[NUM_TRACKED]  = {
  "ISS",   "RS-44", "AO-07", "SO-50",
  "AO-91", "AO-27", "FO-29", "PO-101",
  "", "", "", "",
  "", "", "", "",
  "", "", "", "" };

time_t lastTLETime = 0;
bool   forceTLEUpdate = false;
String statusMsg = "Booting...";

int    currentPage = 0;     // which page (of NUM_PAGES) is on screen
bool   needsRedraw = true;  // request a single redraw on next loop

bool rtcSyncedFromNtp = false;
bool fsAvailable = false;   // true only if LittleFS mounted; the app works without it
const time_t TLE_TIME_VALID_THRESHOLD = 1609459200LL; // 2021-01-01 UTC

// ====================== FORWARD DECLARATIONS ======================
void   drawDashboard();
void   drawDegreeSymbol(int16_t x, int16_t y);
void   saveConfig();
void   loadConfig();
bool   fetchBulletin();
bool   parseGPJson(const String& payload, bool buildTrackedTLEs);
void   recomputeAllPasses();
void   checkAlerts();
void   updateLedPhase();
void   fireAlert(AlertKind k, const char* satName);
void   computeNextPass(int idx);
bool   autoLocateViaWiFi();
void   writeSystemClockToRtc();
bool   syncSystemClockFromRtc();
void   startConfigServer();
void   handleRoot();
void   handleSave();
void   gridToLatLon(const char* mgrid, double &lat, double &lon);
void   latLonToGrid(double lat, double lon, char* gridOut);

// ====================== SLOT / PAGE HELPERS ======================
// A slot is "blank" when it has no NORAD id assigned. Blank slots are never
// drawn and are skipped by pass computation. A page is "occupied" if any of its
// four slots is non-blank; blank pages are never rendered and are skipped when
// paging.
inline bool slotEmpty(int idx) {
  return idx < 0 || idx >= NUM_TRACKED || tracked[idx].norad[0] == '\0';
}
inline bool pageOccupied(int page) {
  if (page < 0 || page >= NUM_PAGES) return false;
  for (int s = 0; s < PER_PAGE; s++) if (!slotEmpty(page * PER_PAGE + s)) return true;
  return false;
}
int occupiedPageCount() {
  int n = 0;
  for (int p = 0; p < NUM_PAGES; p++) if (pageOccupied(p)) n++;
  return n;
}
// Step from the current page to the next/prev occupied page, wrapping around.
// dir = +1 (next) or -1 (prev). Returns the same page if none other qualifies.
int adjacentOccupiedPage(int from, int dir) {
  for (int step = 1; step <= NUM_PAGES; step++) {
    int cand = ((from + dir * step) % NUM_PAGES + NUM_PAGES) % NUM_PAGES;
    if (pageOccupied(cand)) return cand;
  }
  return from;
}
// Ordinal (1-based) of a page among the occupied pages, for the "Pg x/y" label.
int occupiedPageOrdinal(int page) {
  int n = 0;
  for (int p = 0; p <= page && p < NUM_PAGES; p++) if (pageOccupied(p)) n++;
  return n;
}


// ====================== TIME HELPERS ======================
time_t jdToUnix(double jd) { return (time_t)((jd - 2440587.5) * 86400.0); }

void setSystemTime(int year, int mon, int day, int hour, int min, int sec) {
  struct tm t = {};
  t.tm_year = year - 1900; t.tm_mon = mon - 1; t.tm_mday = day;
  t.tm_hour = hour; t.tm_min = min; t.tm_sec = sec; t.tm_isdst = 0;
  time_t epoch = mktime(&t);
  if (epoch != (time_t)-1) {
    struct timeval tv; tv.tv_sec = epoch; tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    writeSystemClockToRtc();
  }
}

// ====================== RTC (RX8130CE) ======================
void writeSystemClockToRtc() {
  if (!M5.Rtc.isEnabled()) return;
  time_t now = time(nullptr);
  if (now < TLE_TIME_VALID_THRESHOLD) return;
  M5.Rtc.setDateTime(gmtime(&now));
}
bool syncSystemClockFromRtc() {
  if (!M5.Rtc.isEnabled()) return false;
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year < 2021) return false;
  struct tm t = dt.get_tm(); t.tm_isdst = 0;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  struct timeval tv; tv.tv_sec = epoch; tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  return true;
}

// ====================== MAIDENHEAD ======================
void gridToLatLon(const char* mgrid, double &lat, double &lon) {
  String g = String(mgrid); g.toUpperCase();
  if (g.length() < 4) return;
  lon = (g[0]-'A')*20.0 - 180.0 + 1.0;
  lat = (g[1]-'A')*10.0 - 90.0 + 0.5;
  lon += (g[2]-'0')*2.0;
  lat += (g[3]-'0')*1.0;
  if (g.length() >= 6) {
    lon += (tolower(g[4])-'a')*(2.0/24.0);
    lat += (tolower(g[5])-'a')*(1.0/24.0);
  }
}
void latLonToGrid(double lat, double lon, char* gridOut) {
  lon = fmod(lon + 180.0, 360.0); if (lon < 0) lon += 360.0; lon -= 180.0;
  lat = fmax(-90.0, fmin(90.0, lat));
  int fieldLon = (int)((lon + 180.0) / 20.0);
  int fieldLat = (int)((lat + 90.0) / 10.0);
  double squareLon = fmod((lon + 180.0), 20.0) / 2.0;
  double squareLat = fmod((lat + 90.0), 10.0);
  gridOut[0]='A'+fieldLon; gridOut[1]='A'+fieldLat;
  gridOut[2]='0'+(int)squareLon; gridOut[3]='0'+(int)squareLat;
  gridOut[4]='a'+(int)(squareLon*12.0); gridOut[5]='a'+(int)(squareLat*24.0); gridOut[6]='\0';
}

// ====================== CONFIG STORAGE ======================
void loadConfig() {
  prefs.begin("satdash", true);
  qth_lat = prefs.getDouble("lat", qth_lat);
  qth_lon = prefs.getDouble("lon", qth_lon);
  qth_alt = prefs.getDouble("alt", qth_alt);
  lastTLETime = prefs.getULong("lastTLE", 0);
  for (int i = 0; i < NUM_TRACKED; i++) {
    char kn[8], km[8], k1[8], k2[8];
    snprintf(kn,sizeof(kn),"n%d",i); snprintf(km,sizeof(km),"m%d",i);
    snprintf(k1,sizeof(k1),"t1_%d",i); snprintf(k2,sizeof(k2),"t2_%d",i);
    String n = prefs.getString(kn, DEFAULT_NORAD[i]);
    String m = prefs.getString(km, DEFAULT_NAME[i]);
    n.toCharArray(tracked[i].norad, sizeof(tracked[i].norad));
    m.toCharArray(tracked[i].name,  sizeof(tracked[i].name));
    tracked[i].haveTLE = false; tracked[i].hasPass = false;
    prefs.getString(k1, tracked[i].tle1, sizeof(tracked[i].tle1));
    prefs.getString(k2, tracked[i].tle2, sizeof(tracked[i].tle2));
    if (strlen(tracked[i].tle1) > 60 && strlen(tracked[i].tle2) > 60) tracked[i].haveTLE = true;
  }
  prefs.end();
}
void saveConfig() {
  prefs.begin("satdash", false);
  prefs.putDouble("lat", qth_lat);
  prefs.putDouble("lon", qth_lon);
  prefs.putDouble("alt", qth_alt);
  for (int i = 0; i < NUM_TRACKED; i++) {
    char kn[8], km[8];
    snprintf(kn,sizeof(kn),"n%d",i); snprintf(km,sizeof(km),"m%d",i);
    prefs.putString(kn, tracked[i].norad);
    prefs.putString(km, tracked[i].name);
  }
  prefs.end();
}

// ====================== WIFI GEOLOCATION ======================
bool autoLocateViaWiFi() {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.begin("http://ip-api.com/json/?fields=status,lat,lon,city,country");
  http.setTimeout(12000);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString(); http.end();
    DynamicJsonDocument doc(1536);
    if (!deserializeJson(doc, payload) && doc["status"] == "success" &&
        doc.containsKey("lat") && doc.containsKey("lon")) {
      qth_lat = doc["lat"].as<double>();
      qth_lon = doc["lon"].as<double>();
      saveConfig();
      return true;
    }
  } else http.end();
  return false;
}

// ====================== GP ELEMENTS -> TLE ======================
static char tleChecksum(const char* line) {
  int sum = 0;
  for (int i = 0; i < 68 && line[i]; i++) {
    char c = line[i];
    if (c >= '0' && c <= '9') sum += c - '0'; else if (c == '-') sum += 1;
  }
  return '0' + (sum % 10);
}
static void tleExpField(double value, char* out) {
  if (value == 0.0 || !isfinite(value)) { strcpy(out, " 00000-0"); return; }
  char sign = (value < 0) ? '-' : ' ';
  double a = fabs(value); int exp = 0;
  while (a >= 1.0) { a /= 10.0; exp++; }
  while (a < 0.1)  { a *= 10.0; exp--; }
  long mant = lround(a * 100000.0);
  if (mant >= 100000) { mant /= 10; exp++; }
  sprintf(out, "%c%05ld%c%d", sign, mant, (exp < 0) ? '-' : '+', abs(exp));
}
static bool parseGPEpoch(const char* epoch, int &yy, double &doy) {
  int Y, Mo, D, h, m; double s = 0.0;
  int n = sscanf(epoch, "%d-%d-%d %d:%d:%lf", &Y, &Mo, &D, &h, &m, &s);
  if (n < 6) n = sscanf(epoch, "%d-%d-%dT%d:%d:%lf", &Y, &Mo, &D, &h, &m, &s);
  if (n < 5) return false;
  static const int cum[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  bool leap = (Y%4==0 && (Y%100!=0 || Y%400==0));
  int day = cum[Mo-1] + D + ((leap && Mo>2)?1:0);
  doy = (double)day + (h*3600.0 + m*60.0 + s)/86400.0;
  yy = Y % 100;
  return true;
}
static bool buildTLEFromGP(JsonObject s, char* tle1, char* tle2) {
  const char* epochC = s["EPOCH"] | "";
  if (strlen(epochC) < 10) return false;
  int yy; double doy;
  if (!parseGPEpoch(epochC, yy, doy)) return false;
  double mm = s["MEAN_MOTION"].as<double>();
  if (mm <= 0.0) return false;
  long catnum = s["NORAD_CAT_ID"].as<long>();
  long catCol = (catnum > 99999) ? (catnum % 100000) : catnum;
  char cls = ((const char*)(s["CLASSIFICATION_TYPE"] | "U"))[0]; if (cls==0) cls='U';
  char eph = '0' + (char)(s["EPHEMERIS_TYPE"].as<int>() % 10);   // numeric in the GP JSON
  char intl[9] = "        ";
  const char* oid = s["OBJECT_ID"] | "";
  const char* dash = strchr(oid, '-');
  if (dash && (dash - oid) >= 4) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%c%c%s", oid[2], oid[3], dash + 1);
    snprintf(intl, sizeof(intl), "%-8.8s", tmp);
  }
  double ndot=s["MEAN_MOTION_DOT"].as<double>(), ndotdot=s["MEAN_MOTION_DDOT"].as<double>();
  double bstar=s["BSTAR"].as<double>(), incl=s["INCLINATION"].as<double>();
  double raan=s["RA_OF_ASC_NODE"].as<double>(), ecc=s["ECCENTRICITY"].as<double>();
  double argp=s["ARG_OF_PERICENTER"].as<double>(), ma=s["MEAN_ANOMALY"].as<double>();
  long elset=(long)(s["ELEMENT_SET_NO"] | 0.0), revnum=s["REV_AT_EPOCH"].as<long>()%100000;
  char ndotStr[16];
  sprintf(ndotStr, "%c.%08ld", (ndot<0)?'-':' ', lround(fabs(ndot)*1e8));
  char ddotStr[10], bstarStr[10];
  tleExpField(ndotdot, ddotStr); tleExpField(bstar, bstarStr);
  sprintf(tle1, "1 %5ld%c %-8s %02d%012.8f %s %s %s %c %4ld",
          catCol, cls, intl, yy, doy, ndotStr, ddotStr, bstarStr, eph, elset);
  tle1[68]=tleChecksum(tle1); tle1[69]='\0';
  long eccCol = lround(ecc*1e7);
  sprintf(tle2, "2 %5ld %8.4f %8.4f %07ld %8.4f %8.4f %11.8f%5ld",
          catCol, incl, raan, eccCol, argp, ma, mm, revnum);
  tle2[68]=tleChecksum(tle2); tle2[69]='\0';
  return true;
}

bool parseGPJson(const String& payload, bool buildTrackedTLEs) {
  satCount = 0;
  // The AMSAT bulletin can be ~90KB+ of JSON with full GP element fields for
  // every satellite. ArduinoJson needs several times the text size as working
  // memory, so allocate generously - the S3 has 8MB PSRAM and DynamicJsonDocument
  // will draw from it. (49152 was far too small and caused "GP parse failed".)
  DynamicJsonDocument doc(512 * 1024);
  DeserializationError derr = deserializeJson(doc, payload);
  if (derr) { statusMsg = String("JSON err: ") + derr.c_str(); return false; }
  JsonArray sats;
  if (doc.is<JsonArray>())                    sats = doc.as<JsonArray>();
  else if (doc["satellites"].is<JsonArray>()) sats = doc["satellites"].as<JsonArray>();
  else if (doc["data"].is<JsonArray>())       sats = doc["data"].as<JsonArray>();
  else if (doc["GP"].is<JsonArray>())         sats = doc["GP"].as<JsonArray>();
  else if (doc["elements"].is<JsonArray>())   sats = doc["elements"].as<JsonArray>();
  else return false;

  for (JsonObject s : sats) {
    if (satCount >= 250) break;

    // Name: a string in every variant.
    const char* nameC = s["AMSAT_NAME"] | s["OBJECT_NAME"] | s["name"] | s["SATNAME"] | s["title"] | "";
    String nameStr(nameC);
    nameStr.trim();

    // NORAD catalog id: in the CelesTrak GP / OMM JSON it is a NUMBER
    // (e.g. "NORAD_CAT_ID": 25544), so reading it with a string fallback yields
    // an empty string and every satellite gets skipped (the cause of
    // "GP parse failed"). Read it as a long when numeric, else as a string.
    String noradStr;
    JsonVariant nv = s["NORAD_CAT_ID"];
    if (nv.isNull()) nv = s["norad"];
    if (nv.isNull()) nv = s["CATNR"];
    if (nv.isNull()) nv = s["NORAD"];
    if (nv.isNull()) nv = s["id"];
    if (nv.is<long>()) {
      noradStr = String((long)nv.as<long>());
    } else {
      noradStr = String((const char*)(nv | ""));
    }
    noradStr.trim();

    if (nameStr.length()==0 || noradStr.length()==0) continue;
    nameStr.toCharArray(satList[satCount].name, sizeof(satList[satCount].name));
    noradStr.toCharArray(satList[satCount].norad, sizeof(satList[satCount].norad));

    if (buildTrackedTLEs) {
      long thisNorad = atol(noradStr.c_str());
      for (int i = 0; i < NUM_TRACKED; i++) {
        if (slotEmpty(i)) continue;     // don't try to match blank slots
        // Compare numerically so "07530" (config) matches 7530 (JSON number).
        if (atol(tracked[i].norad) == thisNorad && thisNorad != 0) {
          char t1[80], t2[80];
          bool built = buildTLEFromGP(s, t1, t2);
          if (!built) {
            String tleStr = s["tle"] | "";
            if (tleStr.length() > 20) {
              int n1 = tleStr.indexOf('\n');
              if (n1 > 0) {
                int n2 = tleStr.indexOf('\n', n1 + 1);
                if (n2 > n1) {
                  String l1 = tleStr.substring(n1+1, n2), l2 = tleStr.substring(n2+1);
                  l1.trim(); l2.trim();
                  l1.toCharArray(t1, sizeof(t1)); l2.toCharArray(t2, sizeof(t2));
                  built = (strlen(t1) > 60 && strlen(t2) > 60);
                }
              }
            }
          }
          if (built) {
            strncpy(tracked[i].tle1, t1, sizeof(tracked[i].tle1));
            strncpy(tracked[i].tle2, t2, sizeof(tracked[i].tle2));
            tracked[i].tle1[sizeof(tracked[i].tle1)-1]='\0';
            tracked[i].tle2[sizeof(tracked[i].tle2)-1]='\0';
            tracked[i].haveTLE = true;
            nameStr.toCharArray(tracked[i].name, sizeof(tracked[i].name));
            char k1[8], k2[8];
            snprintf(k1,sizeof(k1),"t1_%d",i); snprintf(k2,sizeof(k2),"t2_%d",i);
            prefs.begin("satdash", false);
            prefs.putString(k1, tracked[i].tle1);
            prefs.putString(k2, tracked[i].tle2);
            prefs.end();
          }
        }
      }
    }
    satCount++;
  }
  return satCount > 0;
}

// Format the persistent data-status line: "GP data: MMM DD HH:MM" from the last
// successful update, or a short state if we've never gotten data.
static void setGpStatusFromCache() {
  if (lastTLETime > TLE_TIME_VALID_THRESHOLD) {
    struct tm t = *gmtime(&lastTLETime);
    static const char* mon[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    char buf[40];
    snprintf(buf, sizeof(buf), "GP data: %s %02d %02d:%02d UTC",
             mon[t.tm_mon], t.tm_mday, t.tm_hour, t.tm_min);
    statusMsg = String(buf);
  } else if (lastTLETime != 0) {
    statusMsg = "GP data loaded";          // have data but no valid timestamp yet
  }
  // else: leave whatever boot/error message is set
}

bool fetchBulletin() {
  bool haveLocal = fsAvailable && LittleFS.exists("/daily-bulletin.json");
  time_t now = time(nullptr);
  bool timeValid = (now > TLE_TIME_VALID_THRESHOLD);

  bool needDownload = false;
  bool forceThisTime = forceTLEUpdate; forceTLEUpdate = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (forceThisTime || lastTLETime == 0 || (timeValid && (now - lastTLETime > 86400))) needDownload = true;
    else if (!haveLocal && (lastTLETime == 0 || (timeValid && (now - lastTLETime > 3600)))) needDownload = true;
  }

  if (needDownload && WiFi.status() == WL_CONNECTED) {
    // Use an explicit TLS client set to insecure (no cert pinning). The AMSAT
    // host's certificate chain isn't bundled on-device, so without this the
    // HTTPS GET fails with -1 and we'd fall through with no data.
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(25);

    HTTPClient http;
    http.begin(client, "https://newark192.amsat.org/gpdata/current/daily-bulletin.json");
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(25000);
    int code = http.GET();
    if (code == 200) {
      String payload = http.getString(); http.end();
      if (payload.length() > 100 && parseGPJson(payload, true)) {
        if (fsAvailable) {
          File f = LittleFS.open("/daily-bulletin.json", "w");
          if (f) { f.print(payload); f.close(); }
        }
        lastTLETime = timeValid ? now : 1;
        if (timeValid) { prefs.begin("satdash", false); prefs.putULong("lastTLE",(unsigned long)lastTLETime); prefs.end(); }
        setGpStatusFromCache();
        if (lastTLETime <= TLE_TIME_VALID_THRESHOLD) statusMsg = "GP data updated";
        return true;
      }
      statusMsg = "GP parse failed (" + String((int)payload.length()) + " bytes)";
      // fall through to try a cached copy
    } else {
      http.end();
      statusMsg = "Download failed (HTTP " + String(code) + ")";
      // fall through to try a cached copy
    }
  }

  if (haveLocal) {
    File f = LittleFS.open("/daily-bulletin.json", "r");
    if (f) {
      String payload = f.readString(); f.close();
      if (payload.length() > 100 && parseGPJson(payload, true)) {
        setGpStatusFromCache();
        if (lastTLETime <= TLE_TIME_VALID_THRESHOLD) statusMsg = "GP data loaded";
        return true;
      }
    }
  }

  // Nothing parsed this call. Be honest only when we truly have no data; if we
  // previously had a successful update (lastTLETime set) keep showing that, so a
  // transient download miss doesn't replace a valid "GP data: <date>" line.
  if (lastTLETime > TLE_TIME_VALID_THRESHOLD) {
    setGpStatusFromCache();
  } else if (WiFi.status() != WL_CONNECTED && !haveLocal) {
    statusMsg = "No WiFi yet, no cached data";
  } else if (!haveLocal) {
    statusMsg = "Waiting for first download";
  }
  return false;
}

// ====================== PASS PREDICTION ======================
// --- Pass-search diagnostics: set DIAG_PASS to 1 to print pass-search
//     internals to the serial console (115200 baud) for ISS (25544) and RS-44
//     (44909). Left in place but disabled - flip to 1 if a satellite ever shows
//     "no pass found" again and you want to see exactly why. ---
#define DIAG_PASS 0

void computeNextPass(int idx) {
  TrackedSat &T = tracked[idx];
  T.hasPass = false; T.arcN = 0; T.nextRoll = 0;
  for (int a = 0; a < AL_COUNT; a++) T.alerted[a] = false;  // new pass -> re-arm alerts

  if (slotEmpty(idx)) return;   // blank slot: nothing to compute

#if DIAG_PASS
  bool diag = (strcmp(T.norad, "25544") == 0 || strcmp(T.norad, "44909") == 0);
  if (diag) {
    Serial.println();
    Serial.printf("[PASS] === %s (NORAD %s) idx=%d ===\n", T.name, T.norad, idx);
    Serial.printf("[PASS] haveTLE=%d\n", (int)T.haveTLE);
    Serial.printf("[PASS] site lat=%.5f lon=%.5f alt=%.1f\n", qth_lat, qth_lon, qth_alt);
    Serial.printf("[PASS] tle1[%d]=%s\n", (int)strlen(T.tle1), T.tle1);
    Serial.printf("[PASS] tle2[%d]=%s\n", (int)strlen(T.tle2), T.tle2);
    time_t nd = time(nullptr);
    Serial.printf("[PASS] now(unix)=%ld  %s", (long)nd, asctime(gmtime(&nd)));
  }
#endif

  if (!T.haveTLE) {
#if DIAG_PASS
    if (diag) Serial.println("[PASS] ABORT: no TLE loaded for this satellite");
#endif
    return;
  }

  sat.init(T.name, T.tle1, T.tle2);
  sat.site(qth_lat, qth_lon, qth_alt);

  // Find the next pass using the library's own forward search. The Hopperpop
  // nextpass() advances an internal predict-point cursor as it iterates, so the
  // robust pattern is: set the predict point ONCE to slightly before now, then
  // call nextpass repeatedly - each call returns the next pass after the
  // previous one, with the cursor carried over. We take the first pass whose LOS
  // is in the future. (Re-initialising the predict point between calls is what
  // made earlier versions unreliable.)
  passinfo p;
  time_t nowT = time(nullptr);
  sat.initpredpoint((unsigned long)(nowT - 1800), 0.0);   // start 30 min back, once

  bool found = false;
  for (int attempt = 0; attempt < 30 && !found; attempt++) {
    bool ok = sat.nextpass(&p, 100, false, 0.0);
#if DIAG_PASS
    if (diag) {
      Serial.printf("[PASS] attempt %d: nextpass()=%d", attempt, (int)ok);
      if (ok) {
        time_t a = jdToUnix(p.jdstart), m = jdToUnix(p.jdmax), s = jdToUnix(p.jdstop);
        Serial.printf("  maxEl=%.2f jdstart=%.6f(%ld) jdmax=%.6f(%ld) jdstop=%.6f(%ld) dStop-now=%lds",
                      p.maxelevation, p.jdstart, (long)a, p.jdmax, (long)m,
                      p.jdstop, (long)s, (long)(s - nowT));
      }
      Serial.println();
    }
#endif
    if (!ok) break;   // no further pass found
    if (p.maxelevation <= 0.5 || p.jdstop <= p.jdmax) {
#if DIAG_PASS
      if (diag) Serial.println("[PASS]   -> skip: degenerate (low maxEl or jdstop<=jdmax)");
#endif
      continue;
    }
    if (jdToUnix(p.jdstop) <= nowT + 5) {
#if DIAG_PASS
      if (diag) Serial.println("[PASS]   -> skip: pass already ended");
#endif
      continue;
    }
    found = true;
  }
  if (!found) {
#if DIAG_PASS
    if (diag) Serial.println("[PASS] RESULT: no usable pass found -> 'no pass found'");
#endif
    return;
  }

  time_t losTime  = jdToUnix(p.jdstop);

  {
    // AOS: walk back from the peak until elevation drops to <= 0. We do NOT use
    // the library's p.jdstart - it is unreliable for some satellites (e.g.
    // RS-44, where it can equal the peak or LOS time). Deriving AOS from the
    // peak is what worked across ISS, AO-7, and RS-44. The walk-back window is
    // generous (3h) so it covers long high-elevation passes, and it is bounded
    // by the pass's own peak so it never runs away.
    time_t peakTime = jdToUnix(p.jdmax);
    time_t aosTime = peakTime;
    bool crossed = false;
    for (long back = 30; back <= 3*3600; back += 30) {
      time_t tt = peakTime - back;
      sat.findsat((unsigned long)tt);
      if (sat.satEl <= 0.0) { aosTime = tt; crossed = true; break; }
    }
    if (!crossed) {
      // Fallback: couldn't find a horizon crossing within the window. Rather
      // than drop the pass (showing a false "no pass found"), approximate AOS
      // from the library's pass start so the pass is still displayed.
      aosTime = jdToUnix(p.jdstart);
      if (aosTime >= losTime || aosTime < peakTime - 3*3600) aosTime = peakTime - 600;
    } else {
      // Refine forward to the first sample with El > 0 (a clean AOS instant).
      for (int d = 0; d < 120; d++) {
        sat.findsat((unsigned long)(aosTime + d));
        if (sat.satEl > 0.0) { aosTime += d; break; }
      }
    }
#if DIAG_PASS
    if (diag) {
      Serial.printf("[PASS] walk-back crossed=%d  AOS=%ld  LOS=%ld  dur=%lds\n",
                    (int)crossed, (long)aosTime, (long)losTime, (long)(losTime - aosTime));
    }
#endif
    if (losTime <= aosTime + 20) {
#if DIAG_PASS
      if (diag) Serial.println("[PASS] REJECT: pass too short (los <= aos+20) -> no pass shown");
#endif
      return;   // implausibly short; skip
    }

    // Azimuth where the satellite rises (AOS) and sets (LOS).
    sat.findsat((unsigned long)aosTime);
    double azAtAos = sat.satAz;
    sat.findsat((unsigned long)losTime);
    double azAtLos = sat.satAz;

    long dur = losTime - aosTime;
    int n = 0;
    for (int k = 0; k < ARC_POINTS; k++) {
      time_t tt = aosTime + (time_t)((double)dur * k / (ARC_POINTS - 1));
      sat.findsat((unsigned long)tt);
      if (sat.satEl >= 0.0) { T.arcAz[n] = (float)sat.satAz; T.arcEl[n] = (float)sat.satEl; n++; }
    }
    T.arcN = n;
    T.aos = aosTime; T.los = losTime;
    T.nextRoll = losTime + 35;   // roll this sat forward ~35s after its pass ends
    T.maxEl = p.maxelevation; T.aosAz = azAtAos; T.losAz = azAtLos;
    T.hasPass = true;
#if DIAG_PASS
    if (diag) {
      Serial.printf("[PASS] SUCCESS: hasPass=1 maxEl=%.1f aosAz=%.0f losAz=%.0f arcN=%d\n",
                    T.maxEl, T.aosAz, T.losAz, T.arcN);
    }
#endif
  }
}

void recomputeAllPasses() {
  // Recompute the next pass for every tracked satellite (all NUM_TRACKED slots,
  // across every page, so alerts and LED phases work for off-page satellites
  // too; blank slots return immediately). Each satellite gets its own nextRoll
  // time (los+margin); the loop watches those individually and only redraws when
  // a satellite on the CURRENT page rolls.
  for (int i = 0; i < NUM_TRACKED; i++) computeNextPass(i);
}

// ====================== ALERTS (LED + SOUND) ======================
// Two independent behaviors:
//  1) The RGB LEDs hold a STEADY color reflecting the current phase of the
//     nearest pass, so a glance tells you where things stand:
//        T-5 .. T-1 min : amber   (pass coming up)
//        T-1 min .. AOS : orange  (imminent)
//        AOS .. LOS     : green   (pass in progress - go!)
//        LOS .. LOS+30s : red     (just ended)
//        otherwise      : off
//     Across all configured satellites, the LED shows the most urgent phase
//     among them (in progress > imminent > upcoming > just-ended).
//  2) A short distinct TONE plays once at each boundary (T-5, T-1, AOS, LOS) as
//     an audible cue at the moment the phase changes.

static CRGB curLedColor = CRGB::Black;

static void ledFill(const CRGB& c) {
  if (!ledsReady) return;
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = c;
  FastLED.show();
}

// Set the steady LED color only when it changes (avoids redundant SPI writes).
static void ledSet(const CRGB& c) {
  if (c.r == curLedColor.r && c.g == curLedColor.g && c.b == curLedColor.b) return;
  curLedColor = c;
  ledFill(c);
}

// One-shot tone signature per boundary (distinct so you can tell them apart).
void fireAlert(AlertKind k, const char* satName) {
  M5.Speaker.setVolume(ALERT_VOLUME);
  switch (k) {
    case AL_T5:                                   // 5 min out: two low beeps
      M5.Speaker.tone(660, 180); delay(230);
      M5.Speaker.tone(660, 180); delay(200);
      break;
    case AL_T1:                                   // 1 min out: three mid beeps
      for (int i = 0; i < 3; i++) { M5.Speaker.tone(880, 150); delay(200); }
      break;
    case AL_AOS:                                  // rise: rising two-tone
      M5.Speaker.tone(880, 200);  delay(230);
      M5.Speaker.tone(1320, 350); delay(380);
      break;
    case AL_LOS:                                  // set: falling two-tone
      M5.Speaker.tone(1320, 200); delay(230);
      M5.Speaker.tone(660, 350);  delay(380);
      break;
    default: break;
  }
}

// Drive the steady LED color from the most urgent pass phase across all sats.
// Priority: in-progress (green) > imminent (orange) > upcoming (amber) >
// just-ended (red) > off. Returns nothing; updates the LEDs directly.
void updateLedPhase() {
  time_t now = time(nullptr);
  if (now < TLE_TIME_VALID_THRESHOLD) { ledSet(CRGB::Black); return; }

  // Rank the phases; pick the highest-priority active one.
  enum { PH_NONE=0, PH_POST=1, PH_T5=2, PH_T1=3, PH_PASS=4 };
  int best = PH_NONE;
  for (int i = 0; i < NUM_TRACKED; i++) {
    TrackedSat &T = tracked[i];
    if (!T.hasPass) continue;
    int ph = PH_NONE;
    if (now >= T.aos && now <= T.los)                       ph = PH_PASS;
    else if (now >= T.aos - ALERT_T1_SEC && now < T.aos)    ph = PH_T1;
    else if (now >= T.aos - ALERT_T5_SEC && now < T.aos)    ph = PH_T5;
    else if (now > T.los && now <= T.los + 30)              ph = PH_POST;
    if (ph > best) best = ph;
  }
  switch (best) {
    case PH_PASS: ledSet(CRGB::Green);          break;  // in progress
    case PH_T1:   ledSet(CRGB(255, 70, 0));     break;  // orange, imminent
    case PH_T5:   ledSet(CRGB(255, 120, 0));    break;  // amber, upcoming
    case PH_POST: ledSet(CRGB::Red);            break;  // just ended (<=30s)
    default:      ledSet(CRGB::Black);          break;  // off
  }
}

// Fire the one-shot boundary tones. Each boundary chimes once per pass. A 30s
// window absorbs scheduling delay (e.g. a slow EPD refresh); >30s late = missed.
void checkAlerts() {
  time_t now = time(nullptr);
  if (now < TLE_TIME_VALID_THRESHOLD) return;   // clock not set yet

  for (int i = 0; i < NUM_TRACKED; i++) {
    TrackedSat &T = tracked[i];
    if (!T.hasPass) continue;

    struct { AlertKind k; time_t when; } pts[AL_COUNT] = {
      { AL_T5,  T.aos - ALERT_T5_SEC },
      { AL_T1,  T.aos - ALERT_T1_SEC },
      { AL_AOS, T.aos },
      { AL_LOS, T.los },
    };
    for (int a = 0; a < AL_COUNT; a++) {
      if (T.alerted[pts[a].k]) continue;
      long late = (long)(now - pts[a].when);
      if (late >= 0 && late <= 30) {        // within the firing window
        T.alerted[pts[a].k] = true;
        fireAlert(pts[a].k, T.name);
      } else if (late > 30) {
        T.alerted[pts[a].k] = true;          // missed (e.g. powered on late) - skip quietly
      }
    }
  }
}

// ====================== BATTERY ======================
int getBatteryPercent() {
  float v = M5.Power.getBatteryVoltage() / 1000.0;
  int pct = (int)((v - 3.4) / (4.2 - 3.4) * 100);
  if (pct > 100) pct = 100; if (pct < 0) pct = 0;
  return pct;
}

// ====================== DRAWING ======================
void drawDegreeSymbol(int16_t x, int16_t y) {
  M5.Display.drawCircle(x + 3, y + 4, 2, COL_FG);  // sits at the cap height of 9pt sans
}

// Draw one satellite cell: a polar az/el plot of the next pass plus text.
// Layout is built top-down from a cursor with explicit padding so nothing
// overlaps and there is breathing room on every side.
void drawCell(int idx, int originX, int originY, int cellW, int cellH) {
  if (slotEmpty(idx)) return;        // blank slot: leave the spot empty
  TrackedSat &T = tracked[idx];

  const int PAD = 9;                 // interior padding from the cell edges
  int innerX = originX + PAD;
  int innerW = cellW - 2 * PAD;

  // Pass-quality accent color from peak elevation:
  //   < 20 deg marginal (green), 20-50 good (blue), > 50 excellent (red).
  int quality = COL_MED;
  if (T.hasPass) {
    if      (T.maxEl >= 50.0) quality = COL_HIGH;
    else if (T.maxEl >= 20.0) quality = COL_MED;
    else                      quality = COL_LOW;
  }

  // --- Name (bold), top of cell ---
  int nameH = 18;
  M5.Display.setFont(FONT_NAME);
  M5.Display.setTextColor(T.hasPass ? quality : COL_FG);
  int tw = M5.Display.textWidth(T.name);
  if (tw > innerW) tw = innerW;      // (centering still fine if it's wide)
  M5.Display.drawString(T.name, originX + cellW/2 - M5.Display.textWidth(T.name)/2, originY + PAD);
  M5.Display.setTextColor(COL_FG);
  M5.Display.setFont(FONT_BODY);

  // --- No-data cases: skip the (meaningless) empty plot and show a centered
  //     message well clear of the name. This avoids drawing text over the plot. ---
  if (!T.haveTLE || !T.hasPass) {
    const char* msg = !T.haveTLE ? "no orbital data" : "no pass found";
    M5.Display.setFont(FONT_BODY);
    M5.Display.setTextColor(!T.haveTLE ? COL_LOS : COL_GRID);
    int mw = M5.Display.textWidth(msg);
    int my = originY + cellH / 2 - 8;          // vertically centered in the cell
    M5.Display.drawString(msg, originX + cellW/2 - mw/2, my);
    M5.Display.setTextColor(COL_FG);
    return;
  }

  // --- Polar plot, centered, sized to leave room for the text block ---
  const int TEXT_LH = 20;                    // roomier line height for the text block
  const int SLABEL  = 16;                    // height taken by the "S" label below the plot
  int topUsed = PAD + nameH + 18;            // down to where the plot circle's top sits (incl. N label)
  int r = 58;                                // fixed, comfortable plot radius
  // If a very short cell ever can't fit it, shrink gracefully.
  int maxR = (cellH - topUsed - SLABEL - (3 * TEXT_LH) - PAD - 12) / 2;
  if (r > maxR) r = maxR;
  if (r < 34) r = 34;
  int cx = originX + cellW / 2;
  int cy = originY + topUsed + r;

  // Graticule with soft yellow "good elevation" fill inside the 45-deg ring.
  M5.Display.fillCircle(cx, cy, r/2, COL_RINGFILL);
  M5.Display.drawCircle(cx, cy, r, COL_FG);
  M5.Display.drawCircle(cx, cy, r/2, COL_GRID);
  M5.Display.fillCircle(cx, cy, 2, COL_FG);
  for (int a = 0; a < 360; a += 90) {
    float rad = a * PI / 180.0;
    M5.Display.drawLine(cx, cy, cx + (int)(r*sin(rad)), cy - (int)(r*cos(rad)), COL_GRID);
  }
  M5.Display.setFont(FONT_BODY);
  M5.Display.setTextColor(COL_FG);
  M5.Display.drawString("N", cx - 4,     cy - r - 15);
  M5.Display.drawString("S", cx - 4,     cy + r + 1);
  M5.Display.drawString("E", cx + r + 4, cy - 8);
  M5.Display.drawString("W", cx - r - 14, cy - 8);

  // Vertically center the 3-line text block in the leftover space below the plot
  // (there is extra room at the bottom of each cell, so balance it rather than
  // crowding the text right under the plot).
  int blockTop  = cy + r + SLABEL;                  // first free y below the S label
  int blockH    = 3 * TEXT_LH;
  int remaining = (originY + cellH - PAD) - blockTop - blockH;
  int gap       = remaining > 12 ? remaining / 2 : 6;
  int textY     = blockTop + gap;

  // Helper to map an az/el sample to a screen point on the polar plot.
  auto polar = [&](float azDeg, float elDeg, int &ox, int &oy) {
    float az = azDeg * PI / 180.0;
    float eln = (90.0 - elDeg) / 90.0;
    ox = cx + (int)(r * eln * sin(az));
    oy = cy - (int)(r * eln * cos(az));
  };

  // Pass arc (blue), 2px thick.
  int px = -1, py = -1;
  for (int k = 0; k < T.arcN; k++) {
    int x, y; polar(T.arcAz[k], T.arcEl[k], x, y);
    if (px >= 0) {
      M5.Display.drawLine(px, py, x, y, COL_PATH);
      M5.Display.drawLine(px, py + 1, x, y + 1, COL_PATH);
    }
    px = x; py = y;
  }
  // AOS (green) and LOS (red) endpoints.
  if (T.arcN > 0) {
    int ax, ay, lx, ly;
    polar(T.arcAz[0], T.arcEl[0], ax, ay);
    polar(T.arcAz[T.arcN - 1], T.arcEl[T.arcN - 1], lx, ly);
    M5.Display.fillCircle(ax, ay, 4, COL_AOS); M5.Display.drawCircle(ax, ay, 4, COL_FG);
    M5.Display.fillCircle(lx, ly, 4, COL_LOS); M5.Display.drawCircle(lx, ly, 4, COL_FG);
  }
  // Peak marker (red) at the highest sampled point.
  {
    int bestK = 0; float bestEl = -1;
    for (int k = 0; k < T.arcN; k++) if (T.arcEl[k] > bestEl) { bestEl = T.arcEl[k]; bestK = k; }
    if (T.arcN > 0) { int hx, hy; polar(T.arcAz[bestK], T.arcEl[bestK], hx, hy); M5.Display.fillCircle(hx, hy, 3, COL_PEAK); }
  }

  // --- Text block ---
  struct tm a = *gmtime(&T.aos);
  struct tm l = *gmtime(&T.los);
  char line[40];
  time_t now = time(nullptr);
  bool nowUp = (T.aos <= now && now <= T.los);

  // AOS line (green): rise time + rise azimuth.
  M5.Display.setTextColor(COL_AOS);
  sprintf(line, "AOS %02d:%02d  Az %.0f", a.tm_hour, a.tm_min, T.aosAz);
  M5.Display.drawString(line, innerX, textY);
  drawDegreeSymbol(innerX + M5.Display.textWidth(line), textY);

  // LOS line (red): set time + set azimuth.
  M5.Display.setTextColor(COL_LOS);
  sprintf(line, "LOS %02d:%02d  Az %.0f", l.tm_hour, l.tm_min, T.losAz);
  M5.Display.drawString(line, innerX, textY + TEXT_LH);
  drawDegreeSymbol(innerX + M5.Display.textWidth(line), textY + TEXT_LH);

  // Max elevation (colored by quality).
  M5.Display.setTextColor(quality);
  sprintf(line, "Max El %.0f", T.maxEl);
  M5.Display.drawString(line, innerX, textY + 2 * TEXT_LH);
  drawDegreeSymbol(innerX + M5.Display.textWidth(line), textY + 2 * TEXT_LH);

  // Right side of the Max-El line: date, or "NOW" badge if a pass is happening.
  if (nowUp) {
    M5.Display.setTextColor(COL_NOW);
    const char* nb = "NOW";
    M5.Display.drawString(nb, originX + cellW - PAD - M5.Display.textWidth(nb), textY + 2 * TEXT_LH);
  } else {
    M5.Display.setTextColor(COL_GRID);
    sprintf(line, "%02d/%02d", a.tm_mon + 1, a.tm_mday);
    M5.Display.drawString(line, originX + cellW - PAD - M5.Display.textWidth(line), textY + 2 * TEXT_LH);
  }
  M5.Display.setTextColor(COL_FG);
}

void drawDashboard() {
  // Never render a blank page: if the current page has no satellites (e.g. after
  // a config change emptied it), snap to the nearest occupied page. If every
  // page is blank, fall through and draw an empty grid with a clear hint.
  if (!pageOccupied(currentPage) && occupiedPageCount() > 0) {
    currentPage = adjacentOccupiedPage(currentPage, +1);
  }

  M5.Display.waitDisplay();
  M5.Display.startWrite();
  M5.Display.fillScreen(COL_BG);

  // ----- Header bar -----  (TOP_MARGIN keeps text off the bezel)
  // Left to right: app name, "Config:" + device IP (where to configure over
  // WiFi), and battery level at the right. No clock - the panel refreshes far
  // too infrequently for a time display to stay accurate.
  const int TOP_MARGIN = 6;
  int hx = 8;

  // App name (bold, accent color).
  M5.Display.setFont(FONT_NAME);
  M5.Display.setTextColor(COL_HDR);
  M5.Display.drawString("PaperSatColor", hx, TOP_MARGIN);
  hx += M5.Display.textWidth("PaperSatColor") + 12;

  // "Config:" label + IP address (or offline state).
  M5.Display.setFont(FONT_BODY);
  M5.Display.setTextColor(COL_FG);
  M5.Display.drawString("Config:", hx, TOP_MARGIN);
  hx += M5.Display.textWidth("Config:") + 6;
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("offline");
  M5.Display.setTextColor(WiFi.status() == WL_CONNECTED ? COL_HDR : COL_LOS);
  M5.Display.drawString(ip.c_str(), hx, TOP_MARGIN);

  // Battery (right-aligned), colored by level.
  int batPct = getBatteryPercent();
  char bat[8]; sprintf(bat, "%d%%", batPct);
  int bw = M5.Display.textWidth(bat);
  int batCol = (batPct >= 50) ? COL_AOS : (batPct >= 20) ? COL_HDR : COL_LOS;
  M5.Display.setTextColor(batCol);
  M5.Display.drawString(bat, SCR_W - bw - 8, TOP_MARGIN);
  M5.Display.setTextColor(COL_FG);

  // ----- 2x2 grid of cells -----
  const int headerH = 30;            // header band height (text + rule + gap)
  const int footerH = 26;            // footer band height (legend + bottom margin)
  M5.Display.drawFastHLine(8, headerH - 4, SCR_W - 16, COL_HDR);  // inset rule

  int gridTop = headerH;
  int gridH = SCR_H - headerH - footerH;
  int cellW = SCR_W / 2;
  int cellH = gridH / 2;

  // Draw the four cells of the current page. Satellite index = page*PER_PAGE + slot.
  // Blank slots render nothing (drawCell no-ops), leaving the spot empty.
  for (int slot = 0; slot < PER_PAGE; slot++) {
    int i = currentPage * PER_PAGE + slot;
    int col = slot % 2;
    int row = slot / 2;
    int ox = col * cellW;
    int oy = gridTop + row * cellH;
    drawCell(i, ox, oy, cellW, cellH);
  }

  if (occupiedPageCount() == 0) {
    // No satellites configured at all - guide the user to the web config rather
    // than showing an empty grid with no explanation.
    M5.Display.setFont(FONT_BODY);
    M5.Display.setTextColor(COL_LOS);
    const char* m1 = "No satellites configured";
    const char* m2 = "Open Config: in a browser to add some";
    M5.Display.drawString(m1, SCR_W/2 - M5.Display.textWidth(m1)/2, gridTop + gridH/2 - 16);
    M5.Display.drawString(m2, SCR_W/2 - M5.Display.textWidth(m2)/2, gridTop + gridH/2 + 6);
    M5.Display.setTextColor(COL_FG);
  } else {
    // grid dividers (inset from the edges so they don't touch the bezel)
    M5.Display.drawFastVLine(cellW, gridTop + 4, gridH - 8, COL_GRID);
    M5.Display.drawFastHLine(8, gridTop + cellH, SCR_W - 16, COL_GRID);
  }

  // ----- Footer: compact legend on the left, status on the right (clipped) -----
  int footTop = SCR_H - footerH;
  M5.Display.drawFastHLine(8, footTop + 2, SCR_W - 16, COL_HDR);
  int fy = footTop + 7;
  M5.Display.setFont(FONT_BODY);

  // Minimal legend: just the two endpoint markers (the El colors are obvious
  // from the cells themselves, and crowding the footer caused overruns).
  M5.Display.fillCircle(10, fy + 7, 3, COL_AOS);
  M5.Display.setTextColor(COL_FG);
  M5.Display.drawString("rise", 18, fy);
  M5.Display.fillCircle(58, fy + 7, 3, COL_LOS);
  M5.Display.drawString("set", 66, fy);

  // Page indicator (e.g. "Pg 1/3"), counting only occupied (rendered) pages.
  char pg[12];
  snprintf(pg, sizeof(pg), "Pg %d/%d", occupiedPageOrdinal(currentPage), occupiedPageCount());
  M5.Display.setTextColor(COL_HDR);
  M5.Display.drawString(pg, 100, fy);
  int pageEnd = 100 + M5.Display.textWidth(pg);

  // Status message: right-aligned region, truncated with an ellipsis so it can
  // never run past the screen edge or collide with the legend/page indicator.
  int statusX = pageEnd + 10;
  int statusMaxW = SCR_W - 8 - statusX;     // available width to the right margin
  String st = statusMsg;
  // Trim characters until it fits the available width.
  while (st.length() > 1 && M5.Display.textWidth(st.c_str()) > statusMaxW) {
    st = st.substring(0, st.length() - 1);
  }
  if (st.length() < statusMsg.length() && st.length() > 1) {
    st = st.substring(0, st.length() - 1) + ".";   // mark truncation
  }
  M5.Display.setTextColor(COL_FG);
  int stw = M5.Display.textWidth(st.c_str());
  M5.Display.drawString(st.c_str(), SCR_W - 8 - stw, fy);   // right-aligned

  M5.Display.endWrite();
  M5.Display.display();   // single full refresh
}

// ====================== CONFIG WEB SERVER ======================
// Serves a page with up to NUM_TRACKED <select> dropdowns (populated from the
// AMSAT catalog, grouped by page, each with a (blank) option) and station
// location fields. Saving persists config, refetches/recomputes, and requests
// one redraw. All configuration happens here - there is no on-device UI.
void handleRoot() {
  String html;
  html.reserve(20000);
  html += "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>";
  html += "<title>PaperSatColor Setup</title><style>";
  html += "body{font-family:sans-serif;margin:16px;max-width:640px}";
  html += "h1{font-size:1.3em}h2{font-size:1.05em;margin:18px 0 2px;border-bottom:1px solid #ccc}";
  html += "label{display:block;margin:8px 0 3px;font-weight:bold}";
  html += "select,input{width:100%;padding:6px;font-size:1em}";
  html += "button{margin-top:16px;padding:10px 18px;font-size:1em}";
  html += ".row{display:flex;gap:10px}.row>div{flex:1}</style></head><body>";
  html += "<h1>PaperSatColor</h1><p>Assign up to " + String(NUM_TRACKED) +
          " satellites across " + String(NUM_PAGES) + " pages of " + String(PER_PAGE) +
          ". Leave any slot on <b>(blank)</b> to skip it - a page with no "
          "satellites is hidden entirely. Catalog: " + String(satCount) +
          " satellites from the AMSAT bulletin.</p>";
  html += "<form method=POST action=/save>";

  for (int i = 0; i < NUM_TRACKED; i++) {
    int slot = i % PER_PAGE + 1;
    if (i % PER_PAGE == 0) {
      html += "<h2>Page " + String(i / PER_PAGE + 1) + "</h2>";
    }
    html += "<label>Slot " + String(slot) + "</label><select name=n" + String(i) + ">";
    // Blank option - selected when this slot is currently empty. This is what
    // lets the user leave a slot (and therefore a page) unused.
    html += "<option value=''";
    if (slotEmpty(i)) html += " selected";
    html += ">&mdash; (blank) &mdash;</option>";
    // Catalog options.
    bool found = false;
    for (int j = 0; j < satCount; j++) {
      bool sel = (!slotEmpty(i) && strcmp(satList[j].norad, tracked[i].norad) == 0);
      if (sel) found = true;
      html += "<option value='" + String(satList[j].norad) + "'";
      if (sel) html += " selected";
      html += ">" + String(satList[j].name) + " (" + String(satList[j].norad) + ")</option>";
    }
    // Current selection not in the catalog (e.g. catalog not loaded yet): keep it.
    if (!found && !slotEmpty(i)) {
      html += "<option value='" + String(tracked[i].norad) + "' selected>" +
              String(tracked[i].name) + " (" + String(tracked[i].norad) + ")</option>";
    }
    html += "</select>";
  }

  char lats[16], lons[16], alts[16];
  dtostrf(qth_lat, 0, 4, lats);
  dtostrf(qth_lon, 0, 4, lons);
  dtostrf(qth_alt, 0, 1, alts);
  html += "<label>Station location</label><div class=row>";
  html += "<div><input name=lat value='" + String(lats) + "' placeholder='lat'></div>";
  html += "<div><input name=lon value='" + String(lons) + "' placeholder='lon'></div>";
  html += "<div><input name=alt value='" + String(alts) + "' placeholder='alt m'></div></div>";
  html += "<p><small>Or enter a Maidenhead grid (overrides lat/lon if 4+ chars):</small>";
  html += "<input name=grid placeholder='e.g. FM18lv'></p>";

  html += "<button type=submit>Save &amp; Refresh</button>";
  html += "</form></body></html>";
  server.send(200, "text/html", html);
}

void handleSave() {
  for (int i = 0; i < NUM_TRACKED; i++) {
    String arg = "n" + String(i);
    if (server.hasArg(arg)) {
      String norad = server.arg(arg);
      norad.trim();
      if (norad.length() == 0) {
        // Blank slot: clear everything so it is treated as empty everywhere.
        tracked[i].norad[0] = '\0';
        tracked[i].name[0]  = '\0';
        tracked[i].tle1[0]  = '\0';
        tracked[i].tle2[0]  = '\0';
        tracked[i].haveTLE  = false;
        tracked[i].hasPass  = false;
        tracked[i].nextRoll = 0;
        continue;
      }
      norad.toCharArray(tracked[i].norad, sizeof(tracked[i].norad));
      // resolve display name from catalog
      for (int j = 0; j < satCount; j++) {
        if (norad == satList[j].norad) {
          strncpy(tracked[i].name, satList[j].name, sizeof(tracked[i].name));
          tracked[i].name[sizeof(tracked[i].name)-1] = '\0';
          break;
        }
      }
      tracked[i].haveTLE = false;  // force rebuild from bulletin
    }
  }
  if (server.hasArg("grid") && server.arg("grid").length() >= 4) {
    char g[12];
    server.arg("grid").toCharArray(g, sizeof(g));
    gridToLatLon(g, qth_lat, qth_lon);
  } else {
    if (server.hasArg("lat")) qth_lat = server.arg("lat").toDouble();
    if (server.hasArg("lon")) qth_lon = server.arg("lon").toDouble();
  }
  if (server.hasArg("alt")) qth_alt = server.arg("alt").toDouble();

  saveConfig();

  // Respond immediately, then apply (the apply path does a slow EPD refresh).
  server.send(200, "text/html",
    "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;margin:24px'>"
    "<h2>Saved.</h2><p>The display will refresh in ~20s.</p>"
    "<p><a href='/'>Back to setup</a></p></body>");

  forceTLEUpdate = true;     // pull fresh elements for newly chosen sats
  fetchBulletin();
  recomputeAllPasses();
  needsRedraw = true;
}

void startConfigServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

// ====================== SETUP / LOOP ======================
void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = true;             // enable the built-in speaker for alert tones
  M5.begin(cfg);                       // M5Unified does NOT probe SD by default
  Serial.begin(115200);                // diagnostics console (115200 baud)
  M5.Display.setRotation(0);          // portrait 400x600 (use 2 if upside down)
  M5.Display.setAutoDisplay(false);   // EPD: accumulate drawing, push once via display()
  M5.Display.setTextDatum(top_left);  // drawString y = top of glyph (matches layout math)
  M5.Display.setFont(FONT_BODY);

  // RGB LEDs (FastLED). If the data pin is wrong for your unit the rest of the
  // app is unaffected - only the LED alerts won't show.
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
  FastLED.show();
  ledsReady = true;

  // Speaker volume for alert tones.
  M5.Speaker.setVolume(ALERT_VOLUME);

  // Boot splash (single refresh).
  M5.Display.startWrite();
  M5.Display.fillScreen(COL_BG);
  M5.Display.setFont(&fonts::FreeSansBold18pt7b);
  M5.Display.setTextColor(COL_HDR);
  M5.Display.drawString("PaperSatColor", 12, 36);
  M5.Display.setFont(FONT_BODY);
  M5.Display.setTextColor(COL_FG);
  M5.Display.drawString("Starting up...", 12, 80);
  M5.Display.drawString("Connecting WiFi, loading orbital data.", 12, 104);
  M5.Display.drawString("First boot may take ~30s (color e-ink is slow).", 12, 124);
  M5.Display.endWrite();
  M5.Display.display();

  // LittleFS is OPTIONAL - it only provides an offline cache of the bulletin, so
  // the app runs fine without it (it just re-downloads each time). A single
  // format-on-fail mount handles a blank partition. If there is no LittleFS
  // partition in the selected flash layout at all, this returns false and the
  // "no media"/mount warnings from the FS layer are harmless - caching is simply
  // disabled. Pick a partition scheme that includes SPIFFS/LittleFS to enable it.
  fsAvailable = LittleFS.begin(true);   // formatOnFail = true

  loadConfig();
  if (syncSystemClockFromRtc()) statusMsg = "Time from RTC";

  // Connect WiFi via stored credentials / captive portal ("PaperSatColor-Setup").
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.autoConnect("PaperSatColor-Setup");

  configTime(0, 0, "pool.ntp.org");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(250);

  if (WiFi.status() == WL_CONNECTED) {
    // First run with default location? Try to auto-locate.
    autoLocateViaWiFi();
  }

  startConfigServer();
  fetchBulletin();
  recomputeAllPasses();
  needsRedraw = true;
}

void loop() {
  M5.update();
  server.handleClient();

  // Persist NTP-synced time to the RTC once.
  if (!rtcSyncedFromNtp && WiFi.status() == WL_CONNECTED &&
      time(nullptr) > TLE_TIME_VALID_THRESHOLD) {
    writeSystemClockToRtc();
    rtcSyncedFromNtp = true;
  }

  time_t now = time(nullptr);

  // ----- Buttons -----
  // USER_KEY1: manual refresh (re-fetch bulletin, recompute all, redraw).
  if (BTN_REFRESH.wasPressed()) {
    statusMsg = "Refreshing...";
    forceTLEUpdate = true;
    fetchBulletin();
    recomputeAllPasses();
    needsRedraw = true;
  }
  // Up / Down: move to the previous / next OCCUPIED page, wrapping around and
  // skipping any blank pages entirely. With only one occupied page these are
  // no-ops; with none, there is nothing to page to.
  if (BTN_PAGE_UP.wasPressed()) {
    int np = adjacentOccupiedPage(currentPage, -1);
    if (np != currentPage) { currentPage = np; needsRedraw = true; }
  }
  if (BTN_PAGE_DN.wasPressed()) {
    int np = adjacentOccupiedPage(currentPage, +1);
    if (np != currentPage) { currentPage = np; needsRedraw = true; }
  }

  // Fire LED/sound alerts for ALL tracked satellites (every page) FIRST, before
  // any roll-forward below clears a just-started pass's alert flags.
  checkAlerts();
  updateLedPhase();   // steady phase color across all configured satellites

  // Per-satellite roll-forward: when a satellite's pass has ended (now past its
  // nextRoll = los+margin), recompute just that satellite's next pass. Only
  // request a screen redraw if that satellite is on the CURRENTLY DISPLAYED page
  // - a pass ending on the other page updates silently (no wasteful refresh).
  for (int i = 0; i < NUM_TRACKED; i++) {
    if (tracked[i].hasPass && tracked[i].nextRoll != 0 && now >= tracked[i].nextRoll) {
      computeNextPass(i);
      if (i / PER_PAGE == currentPage) needsRedraw = true;
    }
  }

  // Safety net: also recompute/redraw once a day even if no event fired (e.g. a
  // satellite with no upcoming pass, or to pick up a fresh daily bulletin).
  static time_t lastDaily = 0;
  if (now > TLE_TIME_VALID_THRESHOLD && (lastDaily == 0 || now - lastDaily > 86400)) {
    lastDaily = now;
    fetchBulletin();
    recomputeAllPasses();
    needsRedraw = true;
  }

  if (needsRedraw) {
    needsRedraw = false;
    drawDashboard();
  }

  delay(50);
}
