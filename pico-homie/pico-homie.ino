#include <Homie.h>

#define FW_NAME "pOwnTracks"
#define FW_VERSION "0.0.10"

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */

bool online = false;
HomieNode gpsNode("gps", "location");

#include <FS.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>

#define STORE		"/data/owntracks.log"

#define LED_ALIVE	D4
#define RXpin	D2
#define TXpin	D1

TinyGPSPlus gps;
SoftwareSerial nss(RXpin, TXpin);	// RXpin, TXpin

static void serialize(TinyGPSPlus &gps, char t);

unsigned long counter = 0L;

unsigned long pingInterval = 0L;
double last_lat, last_lon;
unsigned long trip = 0L;
bool have_first_fix = false;
bool csv = false;
unsigned long minDistance = 100;		// minimum distance in meters before 't'
unsigned long maxInterval = 60 * 60; // maximum interval in seconds before 'p'
unsigned long meters_since_pub;


void fs_prepare()
{
  if (SPIFFS.begin() == false) {
    Serial.println("formatting filesystem...");
    SPIFFS.format();
    Serial.println("done");
  }

  if (SPIFFS.begin() == false) {
    Serial.println("can't mount filesystem!");
    return;
  }
}

void fs_show()
{
  Dir dir = SPIFFS.openDir("/data");

  Serial.println("-- directory ------------------");
  while (dir.next()) {
    Serial.print(dir.fileName());
    Serial.print("  ");

    File f = dir.openFile("r");
    Serial.println(f.size());
    f.close();
  }
  Serial.println("-------------------------------");
  delay(500);
}

static char deviceID[8 + 1];
void generateDeviceID()
{
  char flashChipId[6 + 1];
  sprintf(flashChipId, "%06x", ESP.getFlashChipId());

  snprintf(deviceID, sizeof(deviceID), "%06x%s",
           ESP.getChipId(), flashChipId + strlen(flashChipId) - 2);

  Serial.print("deviceID=");
  Serial.println(deviceID);
}

void appSetup() {
  char payload[32];

  fs_prepare();
  fs_show();

  generateDeviceID();

  Homie.setNodeProperty(gpsNode, "fix", "false", true);

  pinMode(RXpin, INPUT);
  pinMode(TXpin, OUTPUT);

  nss.begin(9600);
}

void appLoop() {
  if (now() % 2) {
    digitalWrite(LED_ALIVE, HIGH);
  } else {
    digitalWrite(LED_ALIVE, LOW);
  }

  if (online) {
    unload_store();
  }

  unsigned long start = millis();
  do {
    while (nss.available() > 0) {
      gps.encode(nss.read());
    }
  } while (millis() - start < 500);

  unsigned long now = millis();

  if (gps.location.isValid()) {
    Serial.println("GPS");

    if (counter % 10 == 0) {
      gpstime_set(gps);
    }
    ++counter;

    if (!have_first_fix) {
      Homie.setNodeProperty(gpsNode, "fix", "true", true);

      serialize(gps, 'f');
      have_first_fix = true;
      last_lat = gps.location.lat();
      last_lon = gps.location.lng();
    } else {
      if (now - pingInterval >= maxInterval * 1000L) {
        serialize(gps, 'p');
        pingInterval = millis();
      }

      unsigned long meters = (unsigned long)TinyGPSPlus::distanceBetween(
                               gps.location.lat(), gps.location.lng(),
                               last_lat, last_lon
                             );

      meters_since_pub += meters;

      Serial.print("METERS=");
      Serial.print(meters);
      Serial.print(" SincePub=");
      Serial.println(meters_since_pub);

      if (meters_since_pub >= minDistance) {
        serialize(gps, 'v');
        meters_since_pub = 0;
      }
      last_lat = gps.location.lat();
      last_lon = gps.location.lng();
    }
  } else {
    Serial.println("no GPS");
  }
}

void onHomieEvent(HomieEvent event) {
  switch (event) {
    case HOMIE_CONFIGURATION_MODE:
      // Do whatever you want when configuration mode is started
      break;
    case HOMIE_NORMAL_MODE:
      // Do whatever you want when normal mode is started
      break;
    case HOMIE_OTA_MODE:
      // Do whatever you want when OTA mode is started
      break;
    case HOMIE_ABOUT_TO_RESET:
      // Do whatever you want when the device is about to reset
      break;
    case HOMIE_WIFI_CONNECTED:
      // Do whatever you want when Wi-Fi is connected in normal mode
      break;
    case HOMIE_WIFI_DISCONNECTED:
      // Do whatever you want when Wi-Fi is disconnected in normal mode
      break;
    case HOMIE_MQTT_CONNECTED:
      // Do whatever you want when MQTT is connected in normal mode
      online = true;
      break;
    case HOMIE_MQTT_DISCONNECTED:
      // Do whatever you want when MQTT is disconnected in normal mode
      online = false;
      break;
  }
}

bool minDistanceReceived(String value) {
  if (value != "?") {
    minDistance = atoi(value.c_str());
  }
  Homie.setNodeProperty(gpsNode, "minDistance", String(minDistance).c_str(), true);
  return true;
}

bool maxIntervalReceived(String value) {
  if (value != "?") {
    maxInterval = atol(value.c_str());
  }
  Homie.setNodeProperty(gpsNode, "maxInterval", String(maxInterval).c_str(), true);
  return true;
}

bool csvReceived(String value) {
  if (value != "?") {
    if (value == "true") {
      csv = true;
    } else {
      csv = false;
    }
  }
  Homie.setNodeProperty(gpsNode, "csv", csv ? "true" : "false", true);
  return true;
}

bool cmdReceived(String value) {
  Serial.println("cmdReceived");
  Homie.setNodeProperty(gpsNode, "out", "ok", false);
  serialize(gps, 'm');
  return true;
}

void setup()
{
  ESP.wdtEnable(WDTO_8S);
  Serial.begin(115200);
  pinMode(LED_ALIVE, OUTPUT);

  Homie.setBrand("pOwnTracks");
  Homie.setFirmware(FW_NAME, FW_VERSION);
  Homie.setSetupFunction(appSetup);
  Homie.setLoopFunction(appLoop);
  Homie.onEvent(onHomieEvent);
  Homie.registerNode(gpsNode);
  gpsNode.subscribe("minDistance", minDistanceReceived);
  gpsNode.subscribe("maxInterval", maxIntervalReceived);
  gpsNode.subscribe("cmd", cmdReceived);
  gpsNode.subscribe("csv", csvReceived);
  Homie.setup();
}

void loop()
{
  wdt_reset();
  Homie.loop();
}

void store(char *payload)
{
  File f = SPIFFS.open(STORE, "a");

  if (!f) {
    Serial.println("Can't open file for append");
    return;
  }

  f.println(payload);
  Serial.print("store(): pos=");
  Serial.println(f.size());
  f.close();
}

void unload_store()
{
  File f = SPIFFS.open(STORE, "r");
  int rc, n = 0;

  if (!f) {
    Serial.println("nothing to unload");
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');

    Homie.setNodeProperty(gpsNode, "csv", line.c_str(), true);
    if (!Homie.isReadyToOperate()) {
      Serial.println("can't publish!");
      return; // keep file intact for next unload (it will have dup publishes!
    }
    Serial.print(n++);
    Serial.print(" ");
    Serial.print(f.position());
    Serial.print(" ");
    Serial.println(line);
  }
  f.close();

  /* If we reach this, we ought to be able to remove the store */

  SPIFFS.remove(STORE);
}

static void serialize(TinyGPSPlus &gps, char t)
{
  char payload[1025];
  boolean rc;
  unsigned long meters = 0L;

  Serial.print("about to serialize t:");
  Serial.println(t);

  if (have_first_fix) {
    meters = (unsigned long)TinyGPSPlus::distanceBetween(
               gps.location.lat(), gps.location.lng(),
               last_lat, last_lon
             );
  }

  trip += meters;
  unsigned int km_trip = (trip + 500) / 1000;

  if (csv) {
# define MILL 1000000.0

    snprintf(payload, sizeof(payload), "%s,%X,%c,%ld,%ld,%d,%d,%d,%u,%u",
             deviceID + strlen(deviceID) - 2,  // TID
             now(),         // tst in HEX
             t,         // t
             int(gps.location.lat() * MILL),    // lat
             int(gps.location.lng() * MILL),    // lon
             int(gps.course.deg() / 10),    // cog
             int(gps.speed.kmph()),     // vel
             int(gps.altitude.meters() / 10), // alt
             meters,          // dist in meters
             km_trip);        // trip in km

    if (online) {
      Homie.setNodeProperty(gpsNode, "csv", payload, true);
    } else {
      store(payload);
    }

  } else {
    dtostrf(gps.location.lat(), 1, 6, payload);
    Homie.setNodeProperty(gpsNode, "lat", payload, true);

    dtostrf(gps.location.lng(), 1, 6, payload);
    Homie.setNodeProperty(gpsNode, "lon", payload, true);

    dtostrf(gps.altitude.meters(), 1, 0, payload);
    Homie.setNodeProperty(gpsNode, "alt", payload, true);

    dtostrf(gps.speed.kmph(), 1, 0, payload);
    Homie.setNodeProperty(gpsNode, "vel", payload, true);

    dtostrf(gps.course.deg(), 1, 0, payload);
    Homie.setNodeProperty(gpsNode, "cog", payload, true);

    snprintf(payload, sizeof(payload), "%ld", gps.satellites.value());
    Homie.setNodeProperty(gpsNode, "numsat", payload, true);

    snprintf(payload, sizeof(payload), "%ld", now());
    Homie.setNodeProperty(gpsNode, "tst", payload, true);

    snprintf(payload, sizeof(payload), "%s", deviceID + strlen(deviceID) - 2);
    Homie.setNodeProperty(gpsNode, "tid", payload, true);

    snprintf(payload, sizeof(payload), "%c", t);
    Homie.setNodeProperty(gpsNode, "t", payload, true);

    snprintf(payload, sizeof(payload), "%ld", meters);
    Homie.setNodeProperty(gpsNode, "trip", payload, true);
  }
}

static void gpstime_set(TinyGPSPlus &gps)
{
  unsigned long age;
  TimeElements t;
  time_t secs;

  t.Second	= gps.time.second();
  t.Minute	= gps.time.minute();
  t.Hour		= gps.time.hour();
  t.Day		= gps.date.day();
  t.Month		= gps.date.month();
  t.Year		= gps.date.year() - 1970;		// 2016

  secs = makeTime(t);
  setTime(secs);
}
