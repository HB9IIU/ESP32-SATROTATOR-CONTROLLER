// myconfig.h
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h> // For uint16_t, etc.

// Wi-Fi configuration
const char* WIFI_SSID = "MESH";
const char* WIFI_PASSWORD = "Nestle2010Nestle";

const char* WIFI_SSID_ALT = "NO WIFI FOR YOU!!!";
const char* WIFI_PASSWORD_ALT = "Nestle2010Nestle";

#define DEVICE_HOSTNAME "esp32azel"  // 🔤 Use lowercase + no dots or spaces for mDNS


// Observer location
const double OBSERVER_LATITUDE = 46.466623;
const double OBSERVER_LONGITUDE = 6.861569;
const double OBSERVER_ALTITUDE = 500.0;
const int DEFAULT_TZ_OFFSET=7200;
// Get your own Google Maps JavaScript API key here:
// https://console.cloud.google.com/google/maps-apis
// Make sure to enable the "Maps JavaScript API" and restrict the key by domain or IP.
const char* GOOGLE_MAPS_API_KEY = "AIzaSyAY-Cm9T7YQSUsU9XyrXUzHXUw4rQNoT0E";

// API configuration
// TimeZoneDB is a free service that provides a comprehensive time zone database for cities worldwide. 
// get yours here https://timezonedb.com/
const char* TIMEZONE_API_KEY = "EH7POYI19YHB";




// Update interval for TLEs in hours
#define TLE_UPDATE_INTERVAL_HOURS 10

// Satellite definition with downlink frequency (MHz)
struct Satellite {
    const char* name;
    int catalogNumber;
    float downlinkFreqMHz;
};

// Define the satellite array (name, NORAD ID, downlink frequency)
Satellite satellites[] = {
  {"SO-50 (SaudiSat-1C)", 27607, 436.795},
  {"ISS", 25544, 437.800},
  {"NOAA 18", 28654, 137.100},
  {"NOAA 19", 33591, 137.9125},
  {"LilacSat-2 (CAS-3H)", 40908, 437.200},
  {"PO-101 (Diwata-2)", 43678, 437.500},
  {"CAS-4A", 42761, 145.870},
  {"CAS-4B", 42759, 145.925},
};

// Select subset to track (by catalog number)
/*
const int satellitesToTrack[] = {
  27607, 25544, 28654, 33591, 40908, 43678, 42761, 42759
};
*/
const int satellitesToTrack[] = {
  27607, 25544, 28654, 33591, 40908, 43678
};



// 📡 GQRX TCP control interface
#define GQRX_HOST "192.168.0.52"  // 👈 Replace with your PC's LAN IP
#define GQRX_PORT 7356             // GQRX default control port


#define MAX_PASSES_PER_SAT 10



#endif