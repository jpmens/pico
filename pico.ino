#include <FS.h>
#include <TinyGPS++.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>

#include "pico.h"

const char* ssid = WIFISSID;
const char* password = WIFIPASSWORD;
const char* mqtt_server = MQTTHOST;
const int mqtt_port = MQTTPORT;



#define BASETOPIC	"owntracks/micro"	// no trailing slash!
#define RECONNECTINTERVAL	60000 // 300000			// ms (5m)
#define STORE		"/data/owntracks.log"
#define PINGINTERVAL		(60 * 60 * 1000)		// ms

/*
 * Define or undefine JSON to choose the * payload format to publish.
 * JSON will publish OwnTracks-type JSON payloads, but
 *      we have to limit number of elemements we add to
 *	the JSON in order to keep below PubSubClient's
 *	MAX_PACKET_SIZE.
 * CSV will publish in OwnTracks-type CSV (Greenwich)
 *	which is fully supported by the OwnTracks Recorder.
 */

#undef JSON


#ifdef JSON
# include <ArduinoJson.h>
#endif

#define LED_ALIVE	D5
#define RXpin	D2
#define TXpin	D1

TinyGPSPlus gps;
SoftwareSerial nss(RXpin, TXpin);	// RXpin, TXpin

static void smartdelay(unsigned long ms);
static void serialize(TinyGPSPlus &gps, char t);

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long counter = 0L;
unsigned long lastConnectAttempt = 0L;
unsigned long pingInterval = 0L;
bool is_online = false;		// true if WiFi and MQTT connected
double last_lat, last_lon;
unsigned long trip = 0L;
bool have_first_fix = false;
int mindist = 10;		// minimum distance in meters before 't'
unsigned long meters_since_pub;

static char deviceID[8 + 1];		// name of this device
static char pubtopic[128];
static char subtopic[128];

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


bool setup_wifi()
{
	bool wifi_connected = WiFi.status() == WL_CONNECTED;

	if (wifi_connected)
		return true;

	delay(10);

	Serial.println("Connecting to WiFi");

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	smartdelay(1000);

	wifi_connected = WiFi.status() == WL_CONNECTED;

	Serial.println(wifi_connected ? "WiFi connected" : "WiFi NOT connected");

	return (wifi_connected);
}

bool check_online()
{
	bool status = false;

	if (client.connected() && (WiFi.status() == WL_CONNECTED))
		status = true;
	return status;
}

void callback(char* topic, byte* payload, unsigned int length)
{

	// FIXME: handle cmd/ mindist
	// FIXME: handle cmd/list 	(list files)
	// FIXME: handle cmd/dump	(dump cache)

	Serial.print("Message arrived [");
	Serial.print(topic);
	Serial.print("] ");
}

bool MQTT_reconnect()
{
	char clientID[128];
	char *username = MQTTUSER;
	char *password = MQTTPASS;
	static char *willPayload = "{\"_type\":\"lwt\",\"tst\":0}";
	bool willRetain = false;
	char *willTopic = pubtopic;
	bool mqtt_connected = false;

	snprintf(clientID, sizeof(clientID), "micro-wifi-%s", deviceID);

	Serial.print("Attempting to connect to MQTT as ");
	Serial.println(clientID);

	client.disconnect();

	if (client.connect(clientID, username, password, willTopic, MQTTQOS1, willRetain, willPayload)) {
		Serial.println("Connected to MQTT");
		// Once connected, publish an announcement...
		client.publish(pubtopic, "hola!");
		client.subscribe(subtopic);
		mqtt_connected = true;
	} else {
		Serial.println("NOT connected to MQTT");
	}

	return (mqtt_connected);
}

/* swiped from Homie */
void generateDeviceID()
{
	char flashChipId[6 + 1];
	sprintf(flashChipId, "%06x", ESP.getFlashChipId());

	snprintf(deviceID, sizeof(deviceID), "%06x%s",
		ESP.getChipId(), flashChipId + strlen(flashChipId) - 2);
}

void setup()
{
	ESP.wdtEnable(WDTO_8S);
	Serial.begin(115200);
	nss.begin(9600);
	pinMode(LED_ALIVE, OUTPUT);

	fs_prepare();
	fs_show();

	generateDeviceID();

	Serial.print("deviceID=");
	Serial.println(deviceID);

	snprintf(pubtopic, sizeof(pubtopic), "%s/%s", BASETOPIC, deviceID);
	snprintf(subtopic, sizeof(subtopic), "%s/%s/cmd", BASETOPIC, deviceID);

	client.setServer(mqtt_server, mqtt_port);
	client.setCallback(callback);
}

void loop()
{
	digitalWrite(LED_ALIVE, HIGH);
	smartdelay(50);

	while (nss.available() > 0) {
		gps.encode(nss.read());
		// ESP.wdtFeed();
		yield();
	}


	is_online = check_online();

	unsigned long now = millis();
	if (is_online == false && (now - lastConnectAttempt >= RECONNECTINTERVAL || !lastConnectAttempt)) {
		lastConnectAttempt = now;

		bool have_wifi = setup_wifi();

		if (have_wifi) {
			is_online = MQTT_reconnect();
			if (is_online) {
				/* Transition from OFF- to ON-line; unload */
				unload_store();
			}
		}
	}

	if (gps.location.isValid() && gps.location.age() < 2000) {
		if (counter % 10 == 0) {
			gpstime_set(gps);
		}
		++counter;
		Serial.print("Age=");
		Serial.print(gps.location.age());
		Serial.print(" lat=");
		Serial.println(gps.location.lat());
		Serial.print(" lon=");
		Serial.println(gps.location.lng());

		Serial.print(" Y=");
		Serial.print(gps.date.year()); // Year (2000+) (u16)
		Serial.print(" M=");
		Serial.print(gps.date.month()); // Month (1-12) (u8)
		Serial.print(" D=");
		Serial.print(gps.date.day()); // Day (1-31) (u8)
		Serial.print(" T=");
		Serial.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
		Serial.print(" HH=");
		Serial.print(gps.time.hour()); // Hour (0-23) (u8)
		Serial.print(" MM=");
		Serial.print(gps.time.minute()); // Minute (0-59) (u8)
		Serial.print(" SS=");
		Serial.println(gps.time.second()); // Second (0-59) (u8)

		if (!have_first_fix) {
			serialize(gps, 'f');
			have_first_fix = true;
			last_lat = gps.location.lat();
			last_lon = gps.location.lng();
		} else {
			if (now - pingInterval >= PINGINTERVAL) {
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

			if (meters_since_pub >= mindist) {
				serialize(gps, 'v');
				meters_since_pub = 0;
			}
			last_lat = gps.location.lat();
			last_lon = gps.location.lng();
		}

	} else {
		Serial.println("invalid location");
	}

	digitalWrite(LED_ALIVE, LOW);
	smartdelay(5000);
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
		Serial.println("Can't open file for unloading");
		return;
	}

	while (f.available()) {
		String line = f.readStringUntil('\n');

#if 1
		/* If WiFi / MQTT goes offline now we're screwed */
		if ((rc = client.publish(pubtopic, line.c_str(), true)) == false) {
			Serial.println("can't publish!");

			return; // keep file intact for next unload (it will have dup publishes!

		}
#endif
		Serial.print(n++);
		Serial.print(" ");
		Serial.print(f.position());
		Serial.print(" ");
		Serial.println(line);

		if (client.connected()) {
			client.loop();
			delay(20);
		} else {
			is_online = false;
			Serial.println("client no longer online in unload(); abort");
			return;
		}
	}
	f.close();

	/* If we reach this, we ought to be able to remove the store */

	SPIFFS.remove(STORE);
}

static void serialize(TinyGPSPlus &gps, char t)
{
#ifdef JSON
	StaticJsonBuffer<500> jsonBuffer;
#endif
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

#ifdef JSON
	JsonObject& root = jsonBuffer.createObject();
	char t_str[2] = { t, 0 };

	root["_type"] = "location";

	root.set("n",		counter);
	root.set("lat",	double_with_n_digits(gps.location.lat(), 6));
	root.set("lon",	double_with_n_digits(gps.location.lng(), 6));
	// root.set("alt",	int(gps.altitude.meters()));
	// root.set("vel",	int(gps.speed.kmph()));
	// root.set("cog",	int(gps.course.deg()));
	// root.set("nsat",	gps.satellites.value());
	root.set<long>("tst",	now());
	root.set("tid",		deviceID + strlen(deviceID) - 2); // last 2 chars
	root.set("t",		t);


	root.printTo(payload, sizeof(payload));
#else /* not JSON */
# define MILL 1000000.0

	unsigned int km_trip = (trip + 500) / 1000;
	snprintf(payload, sizeof(payload), "%s,%X,%c,%ld,%ld,%d,%d,%d,%u,%u",
		deviceID + strlen(deviceID) - 2,	// TID
		now(),					// tst in HEX
		t,					// t
		int(gps.location.lat() * MILL),		// lat
		int(gps.location.lng() * MILL),		// lon
		int(gps.course.deg() / 10),		// cog
		int(gps.speed.kmph()),			// vel
		int(gps.altitude.meters() / 10),	// alt
		meters,					// dist in meters
		km_trip);				// trip in km
#endif

	Serial.print("Status: ");
	Serial.print(is_online ? "Online" : "OFFline");
	Serial.print(" free heap=");
	Serial.println(ESP.getFreeHeap());

	if (is_online) {

		if ((rc = client.publish(pubtopic, payload, true)) == false) {
			store(payload);
		}
		Serial.print(" publish = ");
		Serial.println(rc);
		if (client.connected())
			client.loop();
	} else {
		store(payload);
	}

}

static void smartdelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		if (client.connected())
			client.loop();
		while (nss.available())
			gps.encode(nss.read());

		yield();
	} while (millis() - start < ms);
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
