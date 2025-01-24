/*
MIT License

Copyright (c) 2018 esp-rfid Community
Copyright (c) 2017 Ömer Şiar Baysal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#ifdef ESPRFID_DEBUG
#include <RemoteDebug.h>
RemoteDebug Debug;

#ifndef DEBUG_ESP_PORT
#define DEBUG_ESP_PORT Serial
#endif
#define ESPRFID_DEBUG_PORT Debug
#define ESPRFID_LOG_INFO(fmt, ...) debugI(fmt, ##__VA_ARGS__)
#define ESPRFID_LOG_DEBUG(fmt, ...) debugD(fmt, ##__VA_ARGS__)
#define ESPRFID_LOG_WARN(fmt, ...) debugW(fmt, ##__VA_ARGS__)
#define ESPRFID_LOG_ERROR(fmt, ...) debugE(fmt, ##__VA_ARGS__)
#else
#define ESPRFID_LOG_INFO(...) do { (void)0; } while (0)
#define ESPRFID_LOG_DEBUG(...) do { (void)0; } while (0)
#define ESPRFID_LOG_WARN(...) do { (void)0; } while (0)
#define ESPRFID_LOG_ERROR(...) do { (void)0; } while (0)
#endif

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TimeLib.h>
#include <Ticker.h>
#include <time.h>
#include <AsyncMqttClient.h>
#include <Bounce2.h>
#include "magicnumbers.h"
#include "config.h"

Config config;

#include <Wiegand.h>

WIEGAND wg;

// relay specific variables
bool activateRelay[MAX_NUM_RELAYS] = {false, false, false, false};
bool deactivateRelay[MAX_NUM_RELAYS] = {false, false, false, false};

// these are from vendors
#include "webh/glyphicons-halflings-regular.woff.gz.h"
#include "webh/required.css.gz.h"
#include "webh/required.js.gz.h"

// these are from us which can be updated and changed
#include "webh/esprfid.js.gz.h"
#include "webh/esprfid.htm.gz.h"
#include "webh/index.html.gz.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker wsMessageTicker;
WiFiEventHandler wifiDisconnectHandler, wifiConnectHandler, wifiOnStationModeGotIPHandler;
Bounce openLockButton;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define LEDoff HIGH
#define LEDon LOW

#define BEEPERoff HIGH
#define BEEPERon LOW

// Variables for whole scope
unsigned long currentMillis = 0;
unsigned long deltaTime = 0;
bool doEnableWifi = false;
bool formatreq = false;
const char *httpUsername = "admin";
unsigned long keyTimer = 0;
unsigned long nextbeat = 0;
time_t epoch;
time_t lastNTPepoch;
unsigned long lastNTPSync = 0;
unsigned long openDoorMillis = 0;
unsigned long previousLoopMillis = 0;
unsigned long previousMillis = 0;
bool shouldReboot = false;
tm timeinfo;
unsigned long uptimeSeconds = 0;
unsigned long wifiPinBlink = millis();
unsigned long wiFiUptimeMillis = 0;

// rfid.h
void remoteValidAccess(int doorNumber, const char *newUsername, const char *newUid, const char *logId);
void remoteInvalidAccess(const char *newUsername, const char *newUid, const char *logId);

#include "led.esp"
#include "beeper.esp"
#include "log.esp"
#include "mqtt.esp"
#include "helpers.esp"
#include "wsResponses.esp"
#include "rfid.esp"
#include "wifi.esp"
#include "config.esp"
#include "websocket.esp"
#include "webserver.esp"

void ICACHE_FLASH_ATTR setup()
{
#ifdef ESPRFID_DEBUG
	DEBUG_ESP_PORT.begin(115200);
	DEBUG_ESP_PORT.println();

	DEBUG_ESP_PORT.print(F("[ INFO ] ESP RFID v"));
	DEBUG_ESP_PORT.println(ESPRFID_VERSION);

	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	FlashMode_t ideMode = ESP.getFlashChipMode();
	DEBUG_ESP_PORT.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
	DEBUG_ESP_PORT.printf("Flash real size: %u\n\n", realSize);
	DEBUG_ESP_PORT.printf("Flash ide  size: %u\n", ideSize);
	DEBUG_ESP_PORT.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
	DEBUG_ESP_PORT.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT"
																	: ideMode == FM_DIO	   ? "DIO"
																	: ideMode == FM_DOUT   ? "DOUT"
																						   : "UNKNOWN"));
	if (ideSize != realSize)
	{
		DEBUG_ESP_PORT.println("Flash Chip configuration wrong!\n");
	}
	else
	{
		DEBUG_ESP_PORT.println("Flash Chip configuration ok.\n");
	}
#endif

	if (!SPIFFS.begin())
	{
		if (SPIFFS.format())
		{
			writeEvent("WARN", "sys", "Filesystem formatted", "");
		}
		else
		{
#ifdef ESPRFID_DEBUG
			DEBUG_ESP_PORT.println(F(" failed!"));
			DEBUG_ESP_PORT.println(F("[ WARN ] Could not format filesystem!"));
#endif
		}
	}

	bool configured = false;
	configured = loadConfiguration(config);
	setupMqtt();
	setupWebServer();
	setupWifi(configured);
	writeEvent("INFO", "sys", "System setup completed, running", "");

#ifdef ESPRFID_DEBUG
	// Initialize RemoteDebug
	Debug.begin(config.deviceHostname);          // Initialize the WiFi server
	Debug.setResetCmdEnabled(true);  // Enable the reset command
	Debug.showProfiler(true);        // Profiler (Good to measure times, to optimize codes)
	Debug.showColors(true);          // Colors
	Debug.setSerialEnabled(true);
#endif
}

void ICACHE_RAM_ATTR loop()
{
	currentMillis = millis();
	deltaTime = currentMillis - previousLoopMillis;
	uptimeSeconds = currentMillis / 1000;
	previousLoopMillis = currentMillis;
	
	trySyncNTPtime(10);

	openLockButton.update();
	if (config.openlockpin != 255 && openLockButton.fell())
	{
		writeLatest(" ", "Button", 1);
		mqttPublishAccess(epoch, "true", "Always", "Button", " ");
		activateRelay[0] = true;
		beeperValidAccess();
		ledAccessGrantedOn();
		// TODO: handle other relays
	}

	ledWifiStatus();
	ledAccessDeniedOff();
	ledAccessGrantedOff();
	beeperBeep();

	rfidLoop();

	for (int currentRelay = 0; currentRelay < config.numRelays; currentRelay++)
	{
		if (config.lockType[currentRelay] == LOCKTYPE_CONTINUOUS) // Continuous relay mode
		{
			if (activateRelay[currentRelay])
			{
				if (digitalRead(config.relayPin[currentRelay]) == !config.relayType[currentRelay]) // currently OFF, need to switch ON
				{
					ESPRFID_LOG_DEBUG("mili : %lu", millis());
					ESPRFID_LOG_INFO("activating relay %d now", currentRelay);
					digitalWrite(config.relayPin[currentRelay], config.relayType[currentRelay]);
				}
				else // currently ON, need to switch OFF
				{
					ESPRFID_LOG_DEBUG("mili : %lu", millis());
					ESPRFID_LOG_INFO("deactivating relay %d now\n", currentRelay);
					digitalWrite(config.relayPin[currentRelay], !config.relayType[currentRelay]);
				}
				activateRelay[currentRelay] = false;
			}
		}
		else if (config.lockType[currentRelay] == LOCKTYPE_MOMENTARY) // Momentary relay mode
		{
			if (activateRelay[currentRelay])
			{
				ESPRFID_LOG_DEBUG("mili : %lu", millis());
				ESPRFID_LOG_INFO("activating relay %d now", currentRelay);
				digitalWrite(config.relayPin[currentRelay], config.relayType[currentRelay]);
				previousMillis = millis();
				activateRelay[currentRelay] = false;
				deactivateRelay[currentRelay] = true;
			}
			else if ((currentMillis - previousMillis >= config.activateTime[currentRelay]) && (deactivateRelay[currentRelay]))
			{
				ESPRFID_LOG_DEBUG("currentMillis=%lu previousMillis=%lu activateTime[%d]=%lu activateRelay[%d]=%u", currentMillis, previousMillis, currentRelay, config.activateTime[currentRelay], currentRelay, activateRelay[currentRelay]);
				ESPRFID_LOG_DEBUG("mili : %lu", millis());
				ESPRFID_LOG_INFO("deactivate relay after this");
				digitalWrite(config.relayPin[currentRelay], !config.relayType[currentRelay]);
				deactivateRelay[currentRelay] = false;
			}
		}
	}
	if (formatreq)
	{
		ESPRFID_LOG_WARN("Factory reset initiated...");
		SPIFFS.end();
		ws.enable(false);
		SPIFFS.format();
		ESP.restart();
	}

	if (config.autoRestartIntervalSeconds > 0 && uptimeSeconds > config.autoRestartIntervalSeconds)
	{
		writeEvent("WARN", "sys", "Auto restarting...", "");
		shouldReboot = true;
	}

	if (shouldReboot)
	{
		writeEvent("INFO", "sys", "System is going to reboot", "");
		SPIFFS.end();
		ESP.restart();
	}

	if (WiFi.isConnected())
	{
		wiFiUptimeMillis += deltaTime;
	}

	if (config.wifiTimeout > 0 && wiFiUptimeMillis > (config.wifiTimeout * 1000) && WiFi.isConnected())
	{
		writeEvent("INFO", "wifi", "WiFi is going to be disabled", "");
		disableWifi();
	}

	// don't try connecting to WiFi when waiting for pincode
	if (doEnableWifi == true && keyTimer == 0)
	{
		if (!WiFi.isConnected())
		{
			enableWifi();
			writeEvent("INFO", "wifi", "Enabling WiFi", "");
			doEnableWifi = false;
		}
	}

	if (config.mqttEnabled && mqttClient.connected())
	{
		if ((unsigned)epoch > nextbeat)
		{
			mqttPublishHeartbeat(epoch, uptimeSeconds);
			nextbeat = (unsigned)epoch + config.mqttInterval;
			ESPRFID_LOG_DEBUG("Nextbeat=%lu", nextbeat);
		}
		processMqttQueue();
	}

	processWsQueue();

	// clean unused websockets
	ws.cleanupClients();

#ifdef ESPRFID_DEBUG
	Debug.handle();
#endif
}
