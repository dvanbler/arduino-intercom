#include <Arduino.h>
#include <Serial.h>
#include <WiFiS3.h>

#include "SpeakerDriver.h"
#include "secrets.h"

#define UDP_PORT 6769
#define FREQ_HZ 32000.0

SpeakerDriver speaker(FREQ_HZ, A0);
SpeakerDriver::BufferPtr buffers = speaker.get_buffers();

IPAddress ip;
WiFiUDP udp;

int init_wifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        return -1;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        return -2;
    }

    int status = WL_IDLE_STATUS;
    do {
        status = WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
        if (status != WL_CONNECTED) {
            delay(10000);
        }
    } while (status != WL_CONNECTED);

    do {
        ip = WiFi.localIP();
    } while (ip == INADDR_NONE);

    udp.begin(UDP_PORT);

    return 0;
}

int setup_impl() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n[Intercom]");

    Serial.print("Init wifi...");
    int rv = init_wifi();
    if (rv != 0) {
        Serial.print("ERROR: initWifi returned: ");
        Serial.println(rv);
        return rv;
    }
    Serial.print(" [√] IP Address: ");
    Serial.println(ip);

    Serial.print("Init speaker driver...");
    auto speaker_rv = speaker.begin();
    if (speaker_rv != SpeakerDriver::BeginStatus::SUCCESS) {
        return -1;
    }
    Serial.println(" [√]");

    return 0;
}

void setup() {
    int rv = setup_impl();
    if (rv != 0) {
        while (true);
    }

    pinMode(D4, INPUT_PULLUP);
}

void loop() {
    static unsigned long last = 0;
    unsigned long now = millis();

    if (digitalRead(D4) == LOW) {
        int buffer_num = speaker.reserve_buffer();
        if (buffer_num >= 0) {
            int available = udp.parsePacket();
            if (available == SpeakerDriver::BUFFER_LEN) {
                udp.read(buffers[buffer_num], available);
                speaker.release_buffer(buffer_num, true);
            } else {
                speaker.release_buffer(buffer_num, false);
            }
        }
    }

    if (now - last > 1000) {
        last = now;
        // Serial.println(speaker.no_data_events);
    }
}
