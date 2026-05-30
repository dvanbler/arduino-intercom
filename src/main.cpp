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
MicDriver mic(SEND_FREQ, MIC_PIN, buffer);

IPAddress ip;
WiFiUDP udp_rx;
WiFiUDP udp_tx;

IPAddress broadcast_ip(192, 168, 143, 255);

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

    udp_rx.begin(RECEIVE_UDP_PORT);
    udp_tx.begin(SEND_UDP_PORT);

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

    Serial.print("Init mic driver...");
    auto mic_rv = mic.init();
    if (mic_rv != MicDriver::InitStatus::SUCCESS) {
        return -1;
    }
    Serial.println(" [√]");

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

    bool ptt_pin_high = digitalRead(PTT_PIN) == HIGH;
    bool ptt_pressed = !ptt_pin_high;

    bool ptt_changed = (last_ptt_pressed != ptt_pressed);
    last_ptt_pressed = ptt_pressed;

    if (!ptt_pressed) {
        if (ptt_changed) {
            mic.stop();
            speaker.start();
        }

        // Default operation - play received audio stream from network
        int available = udp_rx.parsePacket();
        if (available > 0) {
            int bytes_to_read = min(available, PACKET_LEN);
            int bytes_read = udp_rx.read(packet, bytes_to_read);
            if (available > PACKET_LEN) {
                // discard any remaining bytes in oversized packet
                udp_rx.flush();
            }
            speaker.play(packet, bytes_read);
        }
    } else {
        // PTT button is pressed

        if (ptt_changed) {
            speaker.stop();
            mic.start();
        }

        while (udp_rx.parsePacket() > 0) {
            udp_rx.flush();
            yield();
        }

        int bytes_read = mic.read(packet, PACKET_LEN);
        if (bytes_read > 0) {
            udp_tx.beginPacket(broadcast_ip, SEND_UDP_PORT);
            udp_tx.write(packet, bytes_read);
            udp_tx.endPacket();
        }
    }
    yield();
}