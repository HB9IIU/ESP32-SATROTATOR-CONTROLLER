// wip.cpp
#include "myconfig.h"
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <NTPClient.h>
#include <Sgp4.h>
#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <math.h>
#define TLE_UPDATE_INTERVAL_HOURS 10

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);
Preferences preferences;
Sgp4 sat;
WebSocketsServer webSocket = WebSocketsServer(81);
AsyncWebServer server(80);

unsigned long lastNtpSync = 0;
unsigned long lastTrackTime = 0;
int tzOffset;
const int numSatellitesToTrack = sizeof(satellitesToTrack) / sizeof(satellitesToTrack[0]);
int trackedSatIndex = 4;
float previousDist[numSatellitesToTrack] = {0};
unsigned long previousTime[numSatellitesToTrack] = {0};
float azAtAOS; // 📡 Azimuth at AOS
float azAtLOS; // 📡 Azimuth at LOS
String trackingTarget = "ISS"; // default target to track, could be "sun", "moon", or satellite name
bool trackingEnabled = true;

// ===================================================
// 📌 FUNCTION PROTOTYPES
// ===================================================

#pragma region ✅ Core Functions
void setup();
void loop();
#pragma endregion

#pragma region 📡 Wi-Fi & Time
void connectToWiFi();
int getTimeZoneOffsetFromAPI();
bool synchronizeTimeWithNTP();
void printUtcAndLocalTime();
#pragma endregion

#pragma region 🛰️ TLE Management
void checkAndRefreshTLEsIfNeeded();
void downloadTLEsAndStore();
void loadTLEsIntoRAM();
#pragma endregion

#pragma region 🛰️ Satellite Tracking
void loadAndTrackSatelliteByCatalogNumber(int catalogNumber, bool shouldBroadcast);
void trackConfiguredSatellites();
bool predictNextPass(int catalogNumber);
#pragma endregion

#pragma region ☀️🌙 Sun & Moon Calculations
double getJulianDate();
double julianFromYMD(int year, int month, int day);
void getSunAzElAndRiseSet(double &azimuth, double &elevation, double &sunriseUTC, double &sunsetUTC);
void getMoonAzElAndRiseSet(double &azimuth, double &elevation, double &moonriseUTC, double &moonsetUTC);
String getSunMoonJson();
#pragma endregion

#pragma region 🧮 Math & Formatting Helpers
double fixAngle(double deg);
double jdToHour(double jd);
String formatTimeUTC(double hourUTC);
String formatTimeLocal(double hourUTC);
String formatDurationMMSS(double seconds);
String formatDurationHHMM(double seconds);
String formatTimeStamp(int d, int m, int y, int h, int min);
String StringFormat(const char *fmt, ...);
#pragma endregion

// ===============================================
// 🛰️ TLE RAM Cache
// This avoids reading TLEs from Preferences every second.
// It's filled once at boot using loadTLEsIntoRAM().
// ===============================================
struct TLEData
{
    String name;
    String tle1;
    String tle2;
};

TLEData TLECache[numSatellitesToTrack];

/**
 * 🗂️ loadTLEsIntoRAM()
 *
 * Loads TLE data for all configured satellites from Preferences into RAM.
 * - Fills the TLECache[] array based on `satellitesToTrack[]`.
 * - Ensures that all data is ready for runtime use.
 * - Should be called once at boot after TLEs are downloaded.
 */
void loadTLEsIntoRAM()
{
    Serial.println("🧠 Loading TLEs into RAM cache...");
    preferences.begin("tle-storage", true);

    for (int i = 0; i < numSatellitesToTrack; ++i)
    {
        int catnr = satellitesToTrack[i];
        String base = String(catnr);

        TLECache[i].name = preferences.getString((base + "_name").c_str(), "");
        TLECache[i].tle1 = preferences.getString((base + "_tle1").c_str(), "");
        TLECache[i].tle2 = preferences.getString((base + "_tle2").c_str(), "");

        Serial.printf("🛰️  Cached %s (CATNR %d)", TLECache[i].name.c_str(), catnr);
        Serial.println("");
    }

    preferences.end();
}

// 🌞🌕 Sun and Moon Tracking Data
// This struct holds all computed data for the Sun and Moon,
// including azimuth, elevation, and rise/set times in UTC.
// It's updated every second in the loop() and can be accessed
// by the web server or WebSocket handler to provide live data.
// =======================================================
// Global struct and instance
struct SunMoonStruct
{
    double sunAzimuth, sunElevation, sunRiseUTC, sunSetUTC;
    double moonAzimuth, moonElevation, moonRiseUTC, moonSetUTC;
};
SunMoonStruct SunMoonInfo;

struct SatelliteInfo {
    char name[32];
    float azimuth, elevation, distance;
    float frequencyMHz;
    float correctedFreqMHz;
    float dopplerHz;
    double aosUTC, tcaUTC, losUTC;
    float maxElevation;
    float azAtAOS;   // 📡 Azimuth when satellite appears (AOS)
    float azAtLOS;   // 📡 Azimuth when satellite disappears (LOS)
    String passDuration;   // ⏱️ Duration in MM:SS
    String timeToEvent;    // ⏳ Time until AOS or LOS in HH:MM
};

SatelliteInfo TrackedSatInfo[numSatellitesToTrack];  // Updated every second


String StringFormat(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return String(buf);
}




String formatTimeStampFromJD(double jd, int tzHoursOffset = 0) {
    int y, m, d, h, min;
    double sec;
    invjday(jd, tzHoursOffset, false, y, m, d, h, min, sec);
    return formatTimeStamp(d, m, y, h, min);
}



double jdToHour(double jd)
{
    double dayFraction = jd - (int)jd;
    return dayFraction * 24.0;
}

// FOR SUN AND MOON

double getJulianDate();

const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / PI;

double fixAngle(double deg)
{
    return fmod(fmod(deg, 360.0) + 360.0, 360.0);
}

double getJulianDate()
{
    time_t now = time(nullptr);
    struct tm *utc = gmtime(&now);

    int year = utc->tm_year + 1900;
    int month = utc->tm_mon + 1;
    int day = utc->tm_mday;
    int hour = utc->tm_hour;
    int minute = utc->tm_min;
    int second = utc->tm_sec;

    if (month <= 2)
    {
        year -= 1;
        month += 12;
    }

    int A = year / 100;
    int B = 2 - A + A / 4;

    double JD = int(365.25 * (year + 4716)) +
                int(30.6001 * (month + 1)) +
                day + B - 1524.5 +
                (hour + minute / 60.0 + second / 3600.0) / 24.0;
    return JD;
}

String formatTimeUTC(double hourUTC)
{
    if (hourUTC < 0 || hourUTC >= 24)
        return "--:--";
    int h = int(hourUTC);
    int m = int((hourUTC - h) * 60);
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%02d:%02d", h, m);
    return String(buffer);
}

String formatTimeLocal(double hourUTC)
{
    if (hourUTC < 0 || hourUTC >= 24)
        return "--:--";

    double offsetHours = tzOffset / 3600.0;
    hourUTC += offsetHours;
    if (hourUTC >= 24)
        hourUTC -= 24;
    if (hourUTC < 0)
        hourUTC += 24;

    int h = int(hourUTC);
    int m = int((hourUTC - h) * 60);
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%02d:%02d", h, m);
    return String(buffer);
}

/**
 * 🌞 getSunAzElAndRiseSet()
 *
 * Computes and prints:
 * - The Sun's **azimuth** and **elevation**
 * - **Sunrise** and **sunset times** in UTC (fractional hours)
 * - **Transit time** (solar noon) and **day duration**
 * - Formatted local/UTC time strings for debugging/logging
 *
 * Parameters:
 *   azimuth     → (output) sun azimuth in degrees
 *   elevation   → (output) sun elevation in degrees
 *   sunriseUTC  → (output) sunrise time in fractional UTC hours
 *   sunsetUTC   → (output) sunset time in fractional UTC hours
 */
void getSunAzElAndRiseSet(double &azimuth, double &elevation, double &sunriseUTC, double &sunsetUTC)
{
    double JD = getJulianDate();
    double d = JD - 2451545.0;

    // --- Solar position calculations ---

    double g = radians(fmod(357.529 + 0.98560028 * d, 360));
    double q = fmod(280.459 + 0.98564736 * d, 360);
    double L = fmod(q + 1.915 * sin(g) + 0.020 * sin(2 * g), 360);
    double e = radians(23.439 - 0.00000036 * d);
    double L_rad = radians(L);

    double RA = atan2(cos(e) * sin(L_rad), cos(L_rad));
    double dec = asin(sin(e) * sin(L_rad));

    double GMST = fmod(280.16 + 360.9856235 * d, 360);
    double LST = radians(fmod(GMST + OBSERVER_LONGITUDE, 360));
    double HA = LST - RA;

    double latRad = radians(OBSERVER_LATITUDE);
    elevation = asin(sin(dec) * sin(latRad) + cos(dec) * cos(latRad) * cos(HA));
    azimuth = atan2(-sin(HA), tan(dec) * cos(latRad) - sin(latRad) * cos(HA));
    azimuth = fmod(degrees(azimuth) + 360, 360);
    elevation = degrees(elevation);

    // --- Sunrise/Sunset calculation ---

    double cosH = (cos(radians(90.833)) - sin(latRad) * sin(dec)) / (cos(latRad) * cos(dec));

    if (cosH < -1)
    {
        sunriseUTC = 0;
        sunsetUTC = 24;
    }
    else if (cosH > 1)
    {
        sunriseUTC = -1;
        sunsetUTC = -1;
    }
    else
    {
        double H = degrees(acos(cosH)) / 15.0;
        double transitUTC = fmod((720 - 4 * OBSERVER_LONGITUDE) / 60.0, 24);

        sunriseUTC = transitUTC - H;
        sunsetUTC = transitUTC + H;

        double dayDuration = sunsetUTC - sunriseUTC;
        Serial.printf("🌞 Day duration: %.2f hours\n", dayDuration);
        Serial.printf("🕛 Solar noon (transit): %.2f UTC\n", transitUTC);
    }

    // --- Print current sun position and formatted times ---
    Serial.printf("☀️  Sun Azimuth: %.2f°, Elevation: %.2f°\n", azimuth, elevation);
    Serial.printf("🌅 Sunrise: %s local / %s UTC\n",
                  formatTimeLocal(sunriseUTC).c_str(),
                  formatTimeUTC(sunriseUTC).c_str());
    Serial.printf("🌇 Sunset : %s local / %s UTC\n",
                  formatTimeLocal(sunsetUTC).c_str(),
                  formatTimeUTC(sunsetUTC).c_str());
}

double julianFromYMD(int year, int month, int day)
{
    if (month <= 2)
    {
        year--;
        month += 12;
    }
    int A = year / 100;
    int B = 2 - A + A / 4;
    return floor(365.25 * (year + 4716)) +
           floor(30.6001 * (month + 1)) +
           day + B - 1524.5;
}

/**
 * 🌕 getMoonAzElAndRiseSet()
 *
 * Computes Moon's current azimuth and elevation, along with rise and set times.
 * - Uses simplified orbital elements and trigonometry.
 * - Computes moonrise and moonset by hourly elevation sweep.
 * - Times are returned in UTC fractional hours.
 * - Also prints results in local and UTC format.
 */
void getMoonAzElAndRiseSet(double &azimuth, double &elevation, double &moonriseUTC, double &moonsetUTC)
{
    double JD = getJulianDate();
    double D = JD - 2451545.0;

    // --- Position calculation ---
    double L = fixAngle(218.316 + 13.176396 * D);     // Mean longitude
    double M = fixAngle(134.963 + 13.064993 * D);     // Mean anomaly
    double F = fixAngle(93.272 + 13.229350 * D);      // Argument of latitude
    double D_sun = fixAngle(297.850 + 12.190749 * D); // Elongation
    double N = fixAngle(125.044 - 0.0529538 * D);     // Ascending node

    // Convert to radians
    L *= DEG2RAD;
    M *= DEG2RAD;
    F *= DEG2RAD;
    D_sun *= DEG2RAD;
    N *= DEG2RAD;

    // Ecliptic longitude and latitude
    double lon = L + DEG2RAD * (6.289 * sin(M) +
                                1.274 * sin(2 * D_sun - M) +
                                0.658 * sin(2 * D_sun) +
                                0.214 * sin(2 * M) +
                                0.11 * sin(D_sun));
    double lat = DEG2RAD * (5.128 * sin(F) +
                            0.280 * sin(M + F) +
                            0.277 * sin(M - F) +
                            0.173 * sin(2 * D_sun - F));

    double e = DEG2RAD * (23.439 - 0.00000036 * D);

    double x = cos(lat) * cos(lon);
    double y = cos(lat) * sin(lon) * cos(e) - sin(lat) * sin(e);
    double z = cos(lat) * sin(lon) * sin(e) + sin(lat) * cos(e);

    double RA = atan2(y, x);
    double dec = asin(z);

    double T = D / 36525.0;
    double GMST = fixAngle(280.46061837 + 360.98564736629 * D +
                           T * T * (0.000387933 - T / 38710000.0));
    double LST = DEG2RAD * fixAngle(GMST + OBSERVER_LONGITUDE);

    double HA = LST - RA;

    double latRad = DEG2RAD * OBSERVER_LATITUDE;
    elevation = asin(sin(dec) * sin(latRad) + cos(dec) * cos(latRad) * cos(HA));
    azimuth = atan2(-sin(HA),
                    tan(dec) * cos(latRad) - sin(latRad) * cos(HA));
    azimuth = fixAngle(RAD2DEG * azimuth);
    elevation = RAD2DEG * elevation;

    // --- Moonrise and Moonset (hourly sweep) ---
    time_t now = time(nullptr);
    struct tm *utc = gmtime(&now);
    int year = utc->tm_year + 1900;
    int month = utc->tm_mon + 1;
    int day = utc->tm_mday;

    double jdMidnight = julianFromYMD(year, month, day);

    double prevEl = 0, currEl = 0;
    bool foundRise = false, foundSet = false;
    moonriseUTC = -1;
    moonsetUTC = -1;

    for (int h = 0; h <= 24; ++h)
    {
        double testJD = jdMidnight + h / 24.0;
        double dtest = testJD - 2451545.0;

        double Lm = fixAngle(218.316 + 13.176396 * dtest);
        double Mm = fixAngle(134.963 + 13.064993 * dtest);
        double Fm = fixAngle(93.272 + 13.229350 * dtest);
        double Dm = fixAngle(297.850 + 12.190749 * dtest);

        Lm *= DEG2RAD;
        Mm *= DEG2RAD;
        Fm *= DEG2RAD;
        Dm *= DEG2RAD;

        double lonm = Lm + DEG2RAD * (6.289 * sin(Mm) +
                                      1.274 * sin(2 * Dm - Mm) +
                                      0.658 * sin(2 * Dm) +
                                      0.214 * sin(2 * Mm) +
                                      0.11 * sin(Dm));
        double latm = DEG2RAD * (5.128 * sin(Fm));

        double em = DEG2RAD * (23.439 - 0.00000036 * dtest);
        double xm = cos(latm) * cos(lonm);
        double ym = cos(latm) * sin(lonm) * cos(em) - sin(latm) * sin(em);
        double zm = cos(latm) * sin(lonm) * sin(em) + sin(latm) * cos(em);

        double RAm = atan2(ym, xm);
        double decm = asin(zm);

        double GMSTm = fixAngle(280.46061837 + 360.98564736629 * dtest);
        double LSTm = DEG2RAD * fixAngle(GMSTm + OBSERVER_LONGITUDE);
        double HAm = LSTm - RAm;

        double elm = asin(sin(decm) * sin(latRad) + cos(decm) * cos(latRad) * cos(HAm));
        elm = RAD2DEG * elm;

        currEl = elm;

        if (h > 0)
        {
            if (!foundRise && prevEl < 0 && currEl >= 0)
            {
                double t = h - 1 + (-prevEl / (currEl - prevEl));
                moonriseUTC = t;
                foundRise = true;
            }
            if (!foundSet && prevEl >= 0 && currEl < 0)
            {
                double t = h - 1 + (prevEl / (prevEl - currEl));
                moonsetUTC = t;
                foundSet = true;
            }
        }

        prevEl = currEl;
    }

    // 🌕 Print to Serial
    Serial.printf("🌕 Moon Azimuth: %.2f°, Elevation: %.2f°\n", azimuth, elevation);
    Serial.printf("🌙 Moonrise: %s local / %s UTC\n",
                  formatTimeLocal(moonriseUTC).c_str(),
                  formatTimeUTC(moonriseUTC).c_str());
    Serial.printf("🌙 Moonset : %s local / %s UTC\n",
                  formatTimeLocal(moonsetUTC).c_str(),
                  formatTimeUTC(moonsetUTC).c_str());
}

// Returns the current Sun and Moon information in JSON format
// This is useful for AJAX requests from a web interface
String getSunMoonJson()
{
    String json = "{";
    json += "\"sunAzimuth\":" + String(SunMoonInfo.sunAzimuth, 2) + ",";
    json += "\"sunElevation\":" + String(SunMoonInfo.sunElevation, 2) + ",";
    json += "\"sunRiseUTC\":\"" + formatTimeUTC(SunMoonInfo.sunRiseUTC) + "\",";
    json += "\"sunSetUTC\":\"" + formatTimeUTC(SunMoonInfo.sunSetUTC) + "\",";
    json += "\"moonAzimuth\":" + String(SunMoonInfo.moonAzimuth, 2) + ",";
    json += "\"moonElevation\":" + String(SunMoonInfo.moonElevation, 2) + ",";
    json += "\"moonRiseUTC\":\"" + formatTimeUTC(SunMoonInfo.moonRiseUTC) + "\",";
    json += "\"moonSetUTC\":\"" + formatTimeUTC(SunMoonInfo.moonSetUTC) + "\"";
    json += "}";
    return json;
}

/**
 * 📶 connectToWiFi()
 *
 * Connects to Wi-Fi using primary or fallback credentials.
 * - Sets a custom hostname for both DHCP (`.fritz.box`) and mDNS (`.local`) resolution.
 * - Starts mDNS responder if possible.
 * - Shows all connection info via Serial.
 */
void connectToWiFi()
{
    WiFi.setHostname(DEVICE_HOSTNAME);
    int attempt = 0;
    const int maxAttempts = 5;

    Serial.println("🔌 Starting Wi-Fi connection...");
    Serial.printf("🏷️  Setting hostname: %s\n", DEVICE_HOSTNAME);

    while (WiFi.status() != WL_CONNECTED)
    {
        const bool usePrimary = (attempt < maxAttempts);
        const char *ssid = usePrimary ? WIFI_SSID : WIFI_SSID_ALT;
        const char *password = usePrimary ? WIFI_PASSWORD : WIFI_PASSWORD_ALT;

        Serial.printf("📡 Attempt %d (%s)...\n", attempt + 1, usePrimary ? "Primary SSID" : "Fallback SSID");
        Serial.printf("→ Connecting to SSID: %s\n", ssid);

        WiFi.disconnect(true);
        WiFi.begin(ssid, password);

        for (int i = 0; i < 10; ++i)
        {
            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.println("✅ Connected to Wi-Fi!");
                Serial.printf("📶 SSID    : %s\n", ssid);
                Serial.printf("📡 RSSI    : %ld dBm\n", WiFi.RSSI());
                Serial.printf("🌐 IP Addr : %s\n", WiFi.localIP().toString().c_str());
                Serial.printf("🏷️  Hostname: %s\n", DEVICE_HOSTNAME);

                // 🌐 Start mDNS so device is accessible via hostname.local
                if (MDNS.begin(DEVICE_HOSTNAME))
                {
                    Serial.printf("🟢 mDNS started! Access via http://%s.local\n", DEVICE_HOSTNAME);
                }
                else
                {
                    Serial.println("❌ Failed to start mDNS responder.");
                }

                return;
            }
            delay(500);
        }

        Serial.printf("⚠️  Connection to %s failed.\n", ssid);

        attempt++;
        if (attempt == maxAttempts * 2)
        {
            Serial.println("🔁 Retrying Wi-Fi connections from scratch...");
            attempt = 0;
        }
    }
}

/**
 * 🕓 getTimeZoneOffsetFromAPI()
 *
 * Retrieves the current timezone offset (in seconds) from the TimeZoneDB API
 * using the device's geographic position (hardcoded for now).
 *
 * - Parses the JSON response to extract the "gmtOffset" field.
 * - Returns the offset in seconds (e.g., 7200 for UTC+2).
 * - If the API request fails or parsing fails, falls back to UTC+2.
 */
int getTimeZoneOffsetFromAPI()
{
    String url = "http://api.timezonedb.com/v2.1/get-time-zone?key=" +
                 String(TIMEZONE_API_KEY) +
                 "&format=json&by=position&lat=46.2044&lng=6.1432";

    Serial.println("🌍 Requesting timezone offset from TimeZoneDB...");
    Serial.println("🌐 URL: " + url);

    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == 200)
    {
        String payload = http.getString();
        int idx = payload.indexOf("\"gmtOffset\":");

        if (idx >= 0)
        {
            int end = payload.indexOf(",", idx);
            String offsetStr = payload.substring(idx + 12, end);
            int offset = offsetStr.toInt();

            Serial.printf("✅ Timezone offset received: %d seconds (%+.1f hours)\n",
                          offset, offset / 3600.0);
            http.end();
            return offset;
        }
        else
        {
            Serial.println("⚠️  Could not find \"gmtOffset\" in response.");
        }
    }
    else
    {
        Serial.printf("❌ HTTP request failed. Code: %d\n", httpCode);
    }

    http.end();

    // Fallback to UTC+2
    Serial.println("⏱️  Using fallback offset: 7200 seconds (UTC+2)");
    return 7200;
}

/**
 * 🕒 synchronizeTimeWithNTP()
 *
 * Attempts to synchronize system time using a list of NTP servers.
 * - Tries up to 3 times per server until time is successfully set.
 * - Updates the system clock using `settimeofday`.
 * - Logs each server attempt, success, or failure to Serial.
 *
 * Returns:
 *   true if synchronization succeeded, false otherwise.
 */
bool synchronizeTimeWithNTP()
{
    const char *ntpServers[] = {
        "time.google.com",
        "time.nist.gov",
        "time.cloudflare.com",
        "pool.ntp.org",
        "europe.pool.ntp.org"};
    const int maxRetries = 3;

    Serial.println("🕒 Starting NTP time synchronization...");

    for (auto server : ntpServers)
    {
        timeClient.end();
        timeClient.setPoolServerName(server);
        timeClient.begin();

        Serial.printf("🌐 Trying NTP server: %s\n", server);

        for (int attempt = 1; attempt <= maxRetries; ++attempt)
        {
            Serial.printf("  🔁 Attempt %d/%d...\n", attempt, maxRetries);
            if (timeClient.update() && timeClient.getEpochTime() > 1000000000)
            {
                time_t currentTime = timeClient.getEpochTime();
                struct timeval nowTime;
                nowTime.tv_sec = currentTime;
                nowTime.tv_usec = 0;
                settimeofday(&nowTime, nullptr);

                struct tm *tm_utc = gmtime(&currentTime);
                Serial.printf("✅ Time synced successfully from %s\n", server);
                Serial.printf("🕒 UTC Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                              tm_utc->tm_year + 1900, tm_utc->tm_mon + 1, tm_utc->tm_mday,
                              tm_utc->tm_hour, tm_utc->tm_min, tm_utc->tm_sec);

                return true;
            }
            delay(2000);
        }

        Serial.printf("⚠️  Failed to sync with %s\n", server);
    }

    Serial.println("❌ All NTP servers failed. Time not synced.");
    return false;
}

/**
 * 🕒 printUtcAndLocalTime()
 *
 * Displays both the current UTC time and the local time using the manually applied
 * timezone offset (tzOffset, in seconds). This method avoids relying on system timezone
 * settings (`setenv`, `tzset`, `localtime`) to maintain cross-platform consistency.
 *
 * - Uses `gmtime()` to extract the broken-down UTC time.
 * - Calculates the local hour manually by applying `tzOffset / 3600.0`.
 * - Handles day wraparound when local time goes over 24 or below 0.
 *
 * Example Output:
 * 📅 UTC Time   : 22/04/2025 12:45
 * 📅 Local Time : 22/04/2025 14:45
 */
void printUtcAndLocalTime()
{
    time_t now = time(nullptr);
    struct tm *utc = gmtime(&now);

    double h = utc->tm_hour + utc->tm_min / 60.0;
    double localH = h + tzOffset / 3600.0;

    if (localH >= 24)
        localH -= 24;
    if (localH < 0)
        localH += 24;

    Serial.printf("📅 UTC Time   : %02d/%02d/%04d %02d:%02d\n",
                  utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
                  utc->tm_hour, utc->tm_min);

    Serial.printf("📅 Local Time : %02d/%02d/%04d %02d:%02d\n",
                  utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
                  int(localH), int((localH - int(localH)) * 60));
}

/**
 * 🔄 checkAndRefreshTLEsIfNeeded()
 *
 * This function checks whether TLE data needs to be refreshed.
 *
 * Logic:
 * - Retrieves the timestamp of the last TLE update from Preferences.
 * - If the update has never been performed (`last_update == 0`), it triggers a full download.
 * - If any configured satellite is missing TLE data in Preferences, it also forces a download.
 * - If the existing TLEs are older than the threshold (`TLE_UPDATE_INTERVAL_HOURS`), it refreshes.
 * - Otherwise, it logs that no update is needed.
 *
 * This ensures that:
 * - Newly added satellites get their TLEs immediately after first boot.
 * - Users don't have to wait several hours for new entries to populate.
 */
void checkAndRefreshTLEsIfNeeded()
{
    preferences.begin("tle-storage", true);
    unsigned long lastUpdate = preferences.getULong("last_update", 0);

    // 🔍 Check for missing TLEs
    bool anyMissingTLE = false;
    for (int i = 0; i < numSatellitesToTrack; ++i)
    {
        String base = String(satellitesToTrack[i]);
        String tle1 = preferences.getString((base + "_tle1").c_str(), "");
        String tle2 = preferences.getString((base + "_tle2").c_str(), "");

        if (tle1 == "" || tle2 == "")
        {
            Serial.printf("❓ Missing TLEs for CATNR %s\n", base.c_str());
            anyMissingTLE = true;
            break;
        }
    }
    preferences.end();

    if (lastUpdate == 0)
    {
        Serial.println("🆕 TLEs have never been updated. Starting initial download...");
        downloadTLEsAndStore();
        return;
    }

    if (anyMissingTLE)
    {
        Serial.println("📥 One or more TLEs missing. Forcing immediate TLE download...");
        downloadTLEsAndStore();
        return;
    }

    // ⏳ Check age-based refresh
    unsigned long now = time(nullptr);
    unsigned long interval = TLE_UPDATE_INTERVAL_HOURS * 3600;
    unsigned long age = now - lastUpdate;
    float ageHours = age / 3600.0;

    Serial.printf("⏱️  TLE age: %.2f hours (threshold: %d hours)\n", ageHours, TLE_UPDATE_INTERVAL_HOURS);

    if (age > interval)
    {
        Serial.println("🔄 TLE update required. Downloading new TLEs...");
        downloadTLEsAndStore();
    }
    else
    {
        Serial.println("✅ No TLE update needed. Using current data.");
    }
}




/**
 * 📥 downloadTLEsAndStore()
 *
 * Downloads TLE data for all configured satellites and stores it in Preferences.
 * - Fetches TLE from Celestrak using each satellite's catalog number.
 * - Stores name, line1, and line2 in Preferences under keys like "25544_name".
 * - Also stores the timestamp of the last successful update.
 *
 * Notes:
 * - TLEs are saved under namespace `"tle-storage"`.
 * - A small delay is added between downloads to avoid flooding.
 */
void downloadTLEsAndStore()
{
    Serial.println("🌐 Starting TLE download and storage...");

    preferences.begin("tle-storage", false);

    for (const auto &sat : satellites)
    {
        int catnr = sat.catalogNumber;

        // 🌍 Build Celestrak URL
        String url = "http://www.celestrak.org/NORAD/elements/gp.php?CATNR=" + String(catnr) + "&FORMAT=TLE";
        Serial.println("🔗 Fetching: " + url);

        HTTPClient http;
        http.begin(url);
        int code = http.GET();

        if (code == 200)
        {
            // 🛰️ Parse TLE lines
            String payload = http.getString();
            int n1 = payload.indexOf('\n');
            int n2 = payload.indexOf('\n', n1 + 1);

            String name = payload.substring(0, n1);
            name.trim();
            String l1 = payload.substring(n1 + 1, n2);
            l1.trim();
            String l2 = payload.substring(n2 + 1);
            l2.trim();

            // 🧠 Store into Preferences
            String base = String(catnr);
            preferences.putString((base + "_name").c_str(), name);
            preferences.putString((base + "_tle1").c_str(), l1);
            preferences.putString((base + "_tle2").c_str(), l2);

            Serial.printf("✅ Stored: %s (CATNR %d)\n", name.c_str(), catnr);
        }
        else
        {
            Serial.printf("❌ HTTP error %d for CATNR %d\n", code, catnr);
        }

        http.end();
        delay(500); // ⏳ Delay to avoid hitting rate limits
    }

    // 🕓 Store last update time
    preferences.putULong("last_update", time(nullptr));
    preferences.end();

    Serial.println("📦 TLE download and storage complete.");
}

/**
 * 🕒 formatDurationMMSS
 *
 * Converts a duration in seconds to a MM:SS format.
 * Suitable for short durations like satellite pass time.
 */
String formatDurationMMSS(double seconds)
{
    int total = (int)(seconds + 0.5); // round
    int mm = total / 60;
    int ss = total % 60;
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", mm, ss);
    return String(buf);
}

/**
 * ⏳ formatDurationHHMM
 *
 * Converts a duration in seconds to a HH:MM format.
 * Suitable for displaying "time until next pass".
 */
String formatDurationHHMM(double seconds)
{
    int total = (int)(seconds + 30); // round to nearest minute
    int hh = total / 3600;
    int mm = (total % 3600) / 60;
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", hh, mm);
    return String(buf);
}

/**
 * 🕒 formatTimeStamp()
 *
 * Formats a full date + time (from int values) into "DD/MM/YYYY HH:MM".
 * Useful for nicely aligned logging of UTC/local AOS/TCA/LOS times.
 */
String formatTimeStamp(int d, int m, int y, int h, int min)
{
    char buffer[20];
    //snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d", d, m, y, h, min);
    snprintf(buffer, sizeof(buffer), "%02d.%02d %02d:%02d", d, m, h, min);

    return String(buffer);
}

/**
 * 📡 predictNextPass()
 *
 * Computes and prints the next or current satellite pass for the given catalog number.
 * - Checks if satellite is already visible and adjusts prediction start time accordingly.
 * - Uses TLE data from the RAM cache (TLECache[]).
 * - Calculates AOS, TCA, LOS in both UTC and local time using the global `tzOffset`.
 * - Computes pass duration (MM:SS) and time until AOS (or LOS if visible).
 *
 * Returns:
 *   true if a pass is found and printed, false otherwise.
 */
bool predictNextPass(int catalogNumber)
{
    // Step 1: Find satellite index in RAM cache
    int satIndex = -1;
    for (int i = 0; i < numSatellitesToTrack; ++i)
    {
        if (satellitesToTrack[i] == catalogNumber)
        {
            satIndex = i;
            break;
        }
    }

    if (satIndex == -1)
    {
        Serial.printf("❌ Satellite with catalog number %d not found in TLECache.\n", catalogNumber);
        return false;
    }

    String name = TLECache[satIndex].name;
    String l1 = TLECache[satIndex].tle1;
    String l2 = TLECache[satIndex].tle2;

    if (name == "" || l1 == "" || l2 == "")
    {
        Serial.printf("❌ Missing TLE data for CATNR %d\n", catalogNumber);
        return false;
    }

    // Step 2: Prepare Sgp4 predictor
    char nameBuf[64], l1Buf[130], l2Buf[130];
    strncpy(nameBuf, name.c_str(), sizeof(nameBuf));
    strncpy(l1Buf, l1.c_str(), sizeof(l1Buf));
    strncpy(l2Buf, l2.c_str(), sizeof(l2Buf));

    Sgp4 predictor;
    predictor.site(OBSERVER_LATITUDE, OBSERVER_LONGITUDE, OBSERVER_ALTITUDE);
    predictor.init(nameBuf, l1Buf, l2Buf);

    // Step 3: Determine start time for prediction
    time_t now = time(nullptr);
    time_t lookbackStart = now;
    bool satCurrentlyVisible = false;

    const int maxLookbackMinutes = 20;
    for (int i = maxLookbackMinutes; i >= 0; --i)
    {
        time_t t = now - i * 60;
        predictor.findsat(static_cast<unsigned long>(t));
        if (predictor.satEl > 0)
        {
            satCurrentlyVisible = true;
            lookbackStart = t;
            break;
        }
    }

    predictor.initpredpoint(static_cast<unsigned long>(lookbackStart), 0.0);

    // Step 4: Compute next pass (could include current if visible)
    passinfo pass;
    if (!predictor.nextpass(&pass, 20))
    {
        Serial.printf("❌ Prediction failed for %s (CATNR %d)\n", nameBuf, catalogNumber);
        return false;
    }

    // Step 5: Convert AOS, TCA, LOS to human-readable times
    int y, m, d, h, min;
    double sec;
    int tz = tzOffset / 3600;

    invjday(pass.jdstart, 0, false, y, m, d, h, min, sec);
    String aosUTC = formatTimeStamp(d, m, y, h, min);
    invjday(pass.jdstart, tz, false, y, m, d, h, min, sec);
    String aosLocal = formatTimeStamp(d, m, y, h, min);

    invjday(pass.jdmax, 0, false, y, m, d, h, min, sec);
    String tcaUTC = formatTimeStamp(d, m, y, h, min);
    invjday(pass.jdmax, tz, false, y, m, d, h, min, sec);
    String tcaLocal = formatTimeStamp(d, m, y, h, min);

    invjday(pass.jdstop, 0, false, y, m, d, h, min, sec);
    String losUTC = formatTimeStamp(d, m, y, h, min);
    invjday(pass.jdstop, tz, false, y, m, d, h, min, sec);
    String losLocal = formatTimeStamp(d, m, y, h, min);

    // Step 6: Compute durations
    double durationSec = (pass.jdstop - pass.jdstart) * 86400.0;
    double secondsUntilEvent = satCurrentlyVisible
                                   ? (pass.jdstop - (now / 86400.0 + 2440587.5)) * 86400.0   // Until LOS
                                   : (pass.jdstart - (now / 86400.0 + 2440587.5)) * 86400.0; // Until AOS

    // Prevent negative duration formatting
    if (secondsUntilEvent < 0)
        secondsUntilEvent = 0;

    String duration = formatDurationMMSS(durationSec);
    String timeToEvent = formatDurationHHMM(secondsUntilEvent);
    TrackedSatInfo[satIndex].passDuration = duration;
    TrackedSatInfo[satIndex].timeToEvent = timeToEvent;
    TrackedSatInfo[satIndex].aosUTC = pass.jdstart;
TrackedSatInfo[satIndex].tcaUTC = pass.jdmax;
TrackedSatInfo[satIndex].losUTC = pass.jdstop;
TrackedSatInfo[satIndex].maxElevation = pass.maxelevation;

    // Step 7: Print results
    Serial.println();
    Serial.println("📡 Next Pass Times (local / UTC):");
    Serial.printf("AOS : %s local / %s UTC  az=%.1f°\n", aosLocal.c_str(), aosUTC.c_str(), pass.azstart);
    Serial.printf("TCA : %s local / %s UTC  el=%.1f°\n", tcaLocal.c_str(), tcaUTC.c_str(), pass.maxelevation);
    Serial.printf("LOS : %s local / %s UTC  az=%.1f°\n", losLocal.c_str(), losUTC.c_str(), pass.azstop);
    Serial.printf("⏱️  Duration     : %s\n", duration.c_str());

    if (satCurrentlyVisible)
    {
        Serial.printf("⏳ Until LOS    : %s\n", timeToEvent.c_str());
    }
    else
    {
        Serial.printf("⏳ Until AOS    : %s\n", timeToEvent.c_str());
    }
    TrackedSatInfo[satIndex].azAtAOS = pass.azstart;
    TrackedSatInfo[satIndex].azAtLOS = pass.azstop;
    return true;
}

/**
 * 🛰️ loadAndTrackSatelliteByCatalogNumber()
 *
 * Loads TLE data for a specific satellite, calculates its current position,
 * applies Doppler correction, and updates the `TrackedSatInfo` array.
 * Optionally broadcasts the satellite data over WebSocket.
 *
 * Parameters:
 *   catalogNumber   - NORAD catalog number of the satellite to track
 *   shouldBroadcast - whether to broadcast the satellite data via WebSocket
 *
 * Notes:
 * - Uses TLE data from Preferences (or RAM cache if you've optimized it).
 * - Applies manual tzOffset (seconds) to display local time.
 */
void loadAndTrackSatelliteByCatalogNumber(int catalogNumber, bool shouldBroadcast)
{
    // 🔁 Load TLE from cache or Preferences
    String name = "", l1 = "", l2 = "";
    for (int i = 0; i < numSatellitesToTrack; ++i)
    {
        if (satellitesToTrack[i] == catalogNumber)
        {
            name = TLECache[i].name;
            l1 = TLECache[i].tle1;
            l2 = TLECache[i].tle2;
            break;
        }
    }

    if (name == "" || l1 == "" || l2 == "")
    {
        Serial.printf("❌ TLE missing for CATNR %d\n", catalogNumber);
        return;
    }

    char nameBuf[64], l1Buf[130], l2Buf[130];
    strncpy(nameBuf, name.c_str(), sizeof(nameBuf));
    nameBuf[63] = '\0';
    strncpy(l1Buf, l1.c_str(), sizeof(l1Buf));
    l1Buf[129] = '\0';
    strncpy(l2Buf, l2.c_str(), sizeof(l2Buf));
    l2Buf[129] = '\0';

    // 🛰️ Initialize satellite with site location and TLE
    sat.site(OBSERVER_LATITUDE, OBSERVER_LONGITUDE, OBSERVER_ALTITUDE);
    sat.init(nameBuf, l1Buf, l2Buf);
    sat.findsat((unsigned long)time(nullptr));

    // 🆔 Find index of satellite in tracking array
    int satIndex = -1;
    for (int i = 0; i < numSatellitesToTrack; i++)
    {
        if (satellitesToTrack[i] == catalogNumber)
        {
            satIndex = i;
            break;
        }
    }

    // 🛸 Estimate radial velocity (for Doppler)
    float radialVelocity = 0.0;
    unsigned long nowMillis = millis();
    if (satIndex >= 0 && previousTime[satIndex] > 0)
    {
        float dt = (nowMillis - previousTime[satIndex]) / 1000.0;
        if (dt > 0.1)
        {
            radialVelocity = (sat.satDist - previousDist[satIndex]) / dt;
        }
    }
    if (satIndex >= 0)
    {
        previousDist[satIndex] = sat.satDist;
        previousTime[satIndex] = nowMillis;
    }

    // 📡 Get nominal downlink frequency (MHz)
    float nominalFreqMHz = 0.0;
    for (int i = 0; i < sizeof(satellites) / sizeof(satellites[0]); ++i)
    {
        if (satellites[i].catalogNumber == catalogNumber)
        {
            nominalFreqMHz = satellites[i].downlinkFreqMHz;
            break;
        }
    }

    float correctedHz = nominalFreqMHz * 1e6;
    if (nominalFreqMHz > 0.0)
    {
        correctedHz *= (1 - radialVelocity / 299792.458); // Doppler shift
    }

    // 🖨️ Print satellite position info
    Serial.printf("\n🛰️  %s\n", nameBuf);

    int y, m, d, h, min;
    double sec;

    invjday(sat.satJd, 0, false, y, m, d, h, min, sec); // UTC
    Serial.printf("📅 UTC Time   : %02d/%02d/%04d %02d:%02d:%02.0f\n", d, m, y, h, min, sec);

    invjday(sat.satJd, tzOffset / 3600, false, y, m, d, h, min, sec); // Local
    Serial.printf("📅 Local Time : %02d/%02d/%04d %02d:%02d:%02.0f\n", d, m, y, h, min, sec);

    Serial.printf("📡 Az: %.1f°, El: %.1f°, Dist: %.1f km\n", sat.satAz, sat.satEl, sat.satDist);
    Serial.printf("🌍 Lat: %.4f°, Lon: %.4f°, Alt: %.1f km\n", sat.satLat, sat.satLon, sat.satAlt);
    Serial.printf("🚀 Radial velocity ≈ %.3f km/s\n", radialVelocity);
    Serial.printf("📻 Nominal frequency: %.3f MHz\n", nominalFreqMHz);
    Serial.printf("📻 Corrected frequency: %.6f MHz\n", correctedHz / 1e6);
    Serial.printf("📻 Doppler shift: %+6.0f Hz\n", correctedHz - nominalFreqMHz * 1e6);

    // 🌐 Send WebSocket message if requested
    if (shouldBroadcast)
    {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "{\"name\":\"%s\",\"az\":%.1f,\"el\":%.1f,\"freq\":%.6f,"
                 "\"azAOS\":%.1f,\"azLOS\":%.1f}",
                 nameBuf,
                 sat.satAz,
                 sat.satEl,
                 correctedHz / 1e6,
                 TrackedSatInfo[satIndex].azAtAOS,
                 TrackedSatInfo[satIndex].azAtLOS);
        webSocket.broadcastTXT(msg);
        Serial.println("📤 WebSocket sent: " + String(msg));
    }

    // 🧠 Update global info array
    if (satIndex >= 0)
    {
        SatelliteInfo &info = TrackedSatInfo[satIndex];
        strncpy(info.name, nameBuf, sizeof(info.name));
        info.name[sizeof(info.name) - 1] = '\0';

        info.azimuth = sat.satAz;
        info.elevation = sat.satEl;
        info.distance = sat.satDist;
        info.frequencyMHz = nominalFreqMHz;
        info.correctedFreqMHz = correctedHz / 1e6;
        info.dopplerHz = correctedHz - nominalFreqMHz * 1e6;
    }

    // 🛰️ Predict and store next pass info (calls predictNextPass)
    predictNextPass(catalogNumber);
}

/**
 * 🛰️ trackConfiguredSatellites()
 *
 * Loops through all configured satellites and updates their real-time tracking data.
 * - Calls `loadAndTrackSatelliteByCatalogNumber()` for each satellite.
 * - Enables WebSocket broadcast only for the currently tracked satellite (based on `trackedSatIndex`).
 * - Updates azimuth, elevation, distance, Doppler shift, and pass predictions.
 */

void trackConfiguredSatellites()
{

    Serial.println("🔄 Tracking all configured satellites...");

    
    for (int i = 0; i < numSatellitesToTrack; ++i)
    {
        bool shouldBroadcast = false;
        if (trackingTarget == String(TrackedSatInfo[i].name)) {
            if (trackingEnabled==true) {
            shouldBroadcast = true; }
        }
        loadAndTrackSatelliteByCatalogNumber(satellitesToTrack[i], shouldBroadcast);
    }

    if (trackingEnabled && trackingTarget == "sun") {
        String msg = StringFormat(
            "{\"name\":\"Sun\",\"az\":%.1f,\"el\":%.1f}",
            SunMoonInfo.sunAzimuth,
            SunMoonInfo.sunElevation);
        webSocket.broadcastTXT(msg);
        Serial.println("📤 WebSocket sent (Sun): " + msg);
    }
    else if (trackingEnabled && trackingTarget == "moon") {
        String msg = StringFormat(
            "{\"name\":\"Moon\",\"az\":%.1f,\"el\":%.1f}",
            SunMoonInfo.moonAzimuth,
            SunMoonInfo.moonElevation);
        webSocket.broadcastTXT(msg);
        Serial.println("📤 WebSocket sent (Moon): " + msg);
    }
    

    Serial.println("✅ Satellite tracking complete.");
}


void setup()
{
    Serial.begin(115200);
    delay(2000);

    connectToWiFi();

    tzOffset = getTimeZoneOffsetFromAPI();

    synchronizeTimeWithNTP();

    printUtcAndLocalTime();

    lastNtpSync = millis();

    checkAndRefreshTLEsIfNeeded();

    loadTLEsIntoRAM();

    webSocket.begin();
   
    webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
        if (type == WStype_CONNECTED) {
            Serial.printf("✅ WebSocket client [%u] connected\n", num);
        } 
        else if (type == WStype_DISCONNECTED) {
            Serial.printf("❌ WebSocket client [%u] disconnected\n", num);
        } 
        else if (type == WStype_TEXT) {
            String msg = String((char *)payload);
            Serial.printf("📨 WebSocket [%u] message: %s\n", num, msg.c_str());
    
            // 🔴 DEBUG: Pause to read easily (optional, remove later)
            //delay(2000);
    
            // 🛑 Handle stop tracking
            if (msg == "stop" || msg == "\"stop\"" || msg == "{\"track\":\"\"}") {
                trackingEnabled = false;
                trackingTarget = "";
                Serial.println("🛑 Tracking has been stopped by client.");
                return;
            }
    
            // ✅ Handle new tracking target (expects JSON object with "track":"xyz")
            if (msg.startsWith("{") && msg.endsWith("}")) {
                int idx = msg.indexOf("\"track\":\"");
                if (idx >= 0) {
                    int start = idx + 9;
                    int end = msg.indexOf("\"", start);
                    if (end > start) {
                        trackingTarget = msg.substring(start, end);
                        trackingEnabled = true;
                        Serial.printf("📡 Tracking target set to: %s\n", trackingTarget.c_str());
                    }
                }
            }
        }
    });
    


    if (!SPIFFS.begin(true))
    {
        Serial.println("❌ SPIFFS Mount Failed");
        return;
    }



    // 🌞🌕 Route to serve Sun/Moon JSON data
    server.on("/sunmoon", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
    
        // ☀️ SUN
        json += "\"sunAzimuth\":" + String(SunMoonInfo.sunAzimuth, 2) + ",";
        json += "\"sunElevation\":" + String(SunMoonInfo.sunElevation, 2) + ",";
        json += "\"sunRiseUTC\":\"" + formatTimeUTC(SunMoonInfo.sunRiseUTC) + "\",";
        json += "\"sunSetUTC\":\"" + formatTimeUTC(SunMoonInfo.sunSetUTC) + "\",";
        json += "\"sunRiseLocal\":\"" + formatTimeLocal(SunMoonInfo.sunRiseUTC) + "\",";
        json += "\"sunSetLocal\":\"" + formatTimeLocal(SunMoonInfo.sunSetUTC) + "\",";
    
        // 🌕 MOON
        json += "\"moonAzimuth\":" + String(SunMoonInfo.moonAzimuth, 2) + ",";
        json += "\"moonElevation\":" + String(SunMoonInfo.moonElevation, 2) + ",";
        json += "\"moonRiseUTC\":\"" + formatTimeUTC(SunMoonInfo.moonRiseUTC) + "\",";
        json += "\"moonSetUTC\":\"" + formatTimeUTC(SunMoonInfo.moonSetUTC) + "\",";
        json += "\"moonRiseLocal\":\"" + formatTimeLocal(SunMoonInfo.moonRiseUTC) + "\",";
        json += "\"moonSetLocal\":\"" + formatTimeLocal(SunMoonInfo.moonSetUTC) + "\"";
    
        json += "}";
        request->send(200, "application/json", json);
    });
    

    server.on("/observer", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";
        json += "\"latitude\":" + String(OBSERVER_LATITUDE, 4) + ",";
        json += "\"longitude\":" + String(OBSERVER_LONGITUDE, 4);
        json += "}";
        request->send(200, "application/json", json);
    });
    server.serveStatic("/favicon.ico", SPIFFS, "/favicon.ico");

    server.on("/satellites", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        String json = "[";
        for (int i = 0; i < numSatellitesToTrack; i++) {
            if (i > 0) json += ",";
    
            SatelliteInfo &sat = TrackedSatInfo[i];
            json += "{";
            json += "\"name\":\"" + String(sat.name) + "\",";
            json += "\"azimuth\":" + String(sat.azimuth, 1) + ",";
            json += "\"elevation\":" + String(sat.elevation, 1) + ",";
            json += "\"distance\":" + String(sat.distance, 1) + ",";
            json += "\"frequencyMHz\":" + String(sat.frequencyMHz, 3) + ",";
            json += "\"correctedFreqMHz\":" + String(sat.correctedFreqMHz, 6) + ",";
            json += "\"dopplerHz\":" + String(sat.dopplerHz, 0) + ",";
    
            json += "\"aosUTC\":\"" + formatTimeStampFromJD(sat.aosUTC, 0) + "\",";
            json += "\"aosLocal\":\"" + formatTimeStampFromJD(sat.aosUTC, tzOffset / 3600) + "\",";
            json += "\"tcaUTC\":\"" + formatTimeStampFromJD(sat.tcaUTC, 0) + "\",";
            json += "\"tcaLocal\":\"" + formatTimeStampFromJD(sat.tcaUTC, tzOffset / 3600) + "\",";
            json += "\"losUTC\":\"" + formatTimeStampFromJD(sat.losUTC, 0) + "\",";
            json += "\"losLocal\":\"" + formatTimeStampFromJD(sat.losUTC, tzOffset / 3600) + "\",";
    
            json += "\"maxElevation\":" + String(sat.maxElevation, 1) + ",";
            json += "\"passDuration\":\"" + sat.passDuration + "\",";
            json += "\"timeToEvent\":\"" + sat.timeToEvent + "\"";
            json += "}";
        }
        json += "]";
        request->send(200, "application/json", json);
    });
    


    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.begin(); // ✅ Start the Async Web Server

    Serial.println("🌐 Async Web server started!");
    Serial.print("📡 Connect your browser to: http://");
    Serial.println(WiFi.localIP());
    delay(5000);
}




void loop()
{
    webSocket.loop();
    unsigned long currentMillis = millis();

    if (currentMillis - lastTrackTime > 900)
    {
        double az, el, rise, set;
    
        // 🌞 SUN
        getSunAzElAndRiseSet(az, el, rise, set);
        SunMoonInfo.sunAzimuth = az;
        SunMoonInfo.sunElevation = el;
        SunMoonInfo.sunRiseUTC = rise;
        SunMoonInfo.sunSetUTC = set;
    
        // 🌕 MOON
        getMoonAzElAndRiseSet(az, el, rise, set);
        SunMoonInfo.moonAzimuth = az;
        SunMoonInfo.moonElevation = el;
        SunMoonInfo.moonRiseUTC = rise;
        SunMoonInfo.moonSetUTC = set;
    
        // 🛰️ Only now: do tracking & broadcast
        trackConfiguredSatellites();
    
        lastTrackTime = currentMillis;
    }
    



    if (millis() - lastNtpSync > 3600000)
    {
        synchronizeTimeWithNTP();
        lastNtpSync = millis();
    }

    static unsigned long lastTLECheck = 0;
    if (currentMillis - lastTLECheck > 60000)
    {
        checkAndRefreshTLEsIfNeeded();
        lastTLECheck = currentMillis;
    }
}
