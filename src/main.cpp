#include <Arduino.h>
#include <Serial.h>
#include <WiFiS3.h>

#include "CircularBuffer.h"
#include "MicDriver.h"
#include "SpeakerDriver.h"
#include "secrets.h"

#define RECEIVE_UDP_PORT 6769
#define SEND_UDP_PORT 6770
#define RECEIVE_FREQ 32000.0
#define SEND_FREQ 11025.0
#define PACKET_LEN 548

#define SPEAKER_PIN A0
#define MIC_PIN A1

#define PTT_PIN A2

CircularBuffer buffer;
SpeakerDriver speaker(RECEIVE_FREQ, SPEAKER_PIN, buffer);

IPAddress ip;
WiFiUDP udp;

IPAddress last_receive_from_ip;
bool has_ip = false;

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

    udp.begin(RECEIVE_UDP_PORT);

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
    auto speaker_rv = speaker.init();
    if (speaker_rv != SpeakerDriver::InitStatus::SUCCESS) {
        return -1;
    }
    Serial.println(" [√]");

    /*
    Serial.print("Init mic driver...");
    auto mic_rv = mic.init();
    if (mic_rv != MicDriver::InitStatus::SUCCESS) {
        return -1;
    }
    Serial.println(" [√]");
    */

    return 0;
}

void setup() {
    pinMode(PTT_PIN, INPUT_PULLUP);

    int rv = setup_impl();
    if (rv != 0) {
        while (true);
    }

    speaker.start();
}

void loop() {
    static uint8_t packet[PACKET_LEN] = {};

    static bool last_ptt_pressed = false;
    static unsigned long last_ip_check = 0;

    static unsigned long start = millis();
    static unsigned long drop_count = 0;
    static unsigned long last_report = millis();
    static int bytes_read = 0;

    unsigned long now = millis();

    bool ptt_pin_high = digitalRead(PTT_PIN) == HIGH;
    bool ptt_pressed = !ptt_pin_high;

    bool ptt_changed = (last_ptt_pressed != ptt_pressed);
    last_ptt_pressed = ptt_pressed;

    if (!ptt_pressed) {
        if (ptt_changed) {
            speaker.start();
        }

        // Default operation - play received audio stream from network
        int available = udp.parsePacket();
        if (available > 0) {
            if (!has_ip || now - last_ip_check > 10000) {
                // periodically update the ip address to the latest sending
                // remote
                last_receive_from_ip = udp.remoteIP();
                has_ip = true;
                last_ip_check = now;
                Serial.println(last_receive_from_ip);
            }

            int bytes_to_read = min(available, PACKET_LEN);
            bytes_read = udp.read(packet, bytes_to_read);
            if (available > PACKET_LEN) {
                // discard any remaining bytes in oversized packet
                udp.flush();
            }
            int bytes_buffered = speaker.play(packet, bytes_read);
            if (bytes_buffered != bytes_read) {
                drop_count++;
            }
            yield();
        }
    } else {
        // PTT button is pressed

        if (ptt_changed) {
            speaker.stop();
        }

        // Ensure we flush out incoming packets while we are streaming out
        while (udp.parsePacket() > 0) {
            udp.flush();
        }

        if (has_ip && now - last_ip_check < 60000) {
            // We have a target IP: Stream mic audio
            /*
            uint8_t* packet = mic.read_packet();
            if (packet != nullptr) {
                udp.beginPacket(last_receive_from_ip, SEND_UDP_PORT);
                udp.write(packet, MicDriver::BUFFER_LEN);
                udp.endPacket();
                yield();
            }
            */
        } else {
            /*
            speaker.buzz(300);
            */
        }
    }

    if (now - last_report >= 1000) {
        last_report = now;
        Serial.print((int) speaker.get_status());
        Serial.print(" - ");
        Serial.print(speaker.get_last_value());
        Serial.print(" - ");
        Serial.print(bytes_read);
        Serial.print(" - ");
        Serial.println(buffer.available());
    }
}
