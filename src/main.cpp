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

#define HEARTBEAT_RX_UDP_PORT 6771

#define SPEAKER_PIN A0
#define MIC_PIN A1

#define PTT_PIN A2

CircularBuffer buffer;
SpeakerDriver speaker(RECEIVE_FREQ, SPEAKER_PIN, buffer);
MicDriver mic(SEND_FREQ, MIC_PIN, buffer);

IPAddress ip;
WiFiUDP udp_rx;
WiFiUDP udp_tx;
WiFiUDP udp_rx_heartbeat;

IPAddress broadcast_ip(192, 168, 143, 255);

constexpr int BUZZ_FREQ_HZ = 500;
constexpr int BUZZ_LEN = (int)RECEIVE_FREQ / BUZZ_FREQ_HZ;
uint8_t buzz_samples[BUZZ_LEN] = {};
uint8_t silence_samples[BUZZ_LEN] = {};

// (Re)associates with the AP and (re)binds the UDP sockets. Blocks until
// connected, retrying every 10s.
void connect_wifi() {
    // Tear down any prior association/sockets for a clean reconnect (no-ops on
    // first boot).
    WiFi.disconnect();
    udp_rx.stop();
    udp_tx.stop();
    udp_rx_heartbeat.stop();

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
    udp_rx_heartbeat.begin(HEARTBEAT_RX_UDP_PORT);
}

int init_wifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        return -1;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
        return -2;
    }

    connect_wifi();

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

    Serial.print("Init buzz samples...");
    for (int i = 0; i < BUZZ_LEN; i++) {
        buzz_samples[i] = (i < BUZZ_LEN / 2) ? (128 - 4) : (128 + 4);
        silence_samples[i] = 128;
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
    static unsigned long last_heartbeat_check = 0;
    static unsigned long last_heartbeat_active = 0;
    static bool heartbeat_active = false;
    static bool ptt_ready = false;
    static unsigned long last_wifi_check = 0;

    unsigned long now = millis();

    // Check the WiFi link once a second and recover if it has dropped.
    if (now - last_wifi_check >= 1000) {
        last_wifi_check = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi link lost - reconnecting...");

            // Put audio hardware into a known state before the blocking
            // reconnect (both are safe if already stopped).
            mic.stop();
            speaker.stop();

            connect_wifi();

            Serial.print("WiFi reconnected. IP: ");
            Serial.println(ip);

            // Resume receive mode and clear PTT/heartbeat state so the next
            // iteration re-evaluates from a clean slate.
            speaker.start();
            last_ptt_pressed = false;
            ptt_ready = false;
            heartbeat_active = false;
            last_heartbeat_active = 0;

            now = millis();
        }
    }

    // Check every second for a heartbeat packet
    if (now - last_heartbeat_check >= 1000) {
        last_heartbeat_check = now;
        while (udp_rx_heartbeat.parsePacket() > 0) {
            last_heartbeat_active = now;
            udp_rx_heartbeat.flush();
            yield();
        }

        // If we've received a heartbeat packet within the last 4 seconds, then
        // consider receiver active
        heartbeat_active =
            (last_heartbeat_active > 0) && (now - last_heartbeat_active) < 4000;
    }

    bool ptt_pin_high = digitalRead(PTT_PIN) == HIGH;
    bool ptt_pressed = !ptt_pin_high;

    bool ptt_changed = (last_ptt_pressed != ptt_pressed);
    last_ptt_pressed = ptt_pressed;

    if (!ptt_pressed) {
        if (ptt_changed) {
            ptt_ready = false;
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
            if (heartbeat_active) {
                speaker.stop();
                mic.start();
                ptt_ready = true;
            } else {
                // Stop playing and re-start to clear out the buffer
                ptt_ready = false;
                speaker.stop();
                speaker.start();

                // Buzz error sound

                // Initial ramp up
                for (uint8_t i = 0; i < 128; i += 2) {
                    speaker.play(&i, 1);
                }

                for (int i = 0; i < 42; i++) {
                    speaker.play(buzz_samples, BUZZ_LEN);
                }

                for (int i = 0; i < 42; i++) {
                    speaker.play(silence_samples, BUZZ_LEN);
                }

                for (int i = 0; i < 42; i++) {
                    speaker.play(buzz_samples, BUZZ_LEN);
                }
            }
        }

        while (udp_rx.parsePacket() > 0) {
            udp_rx.flush();
            yield();
        }

        if (ptt_ready) {
            int bytes_read = mic.read(packet, PACKET_LEN);
            if (bytes_read > 0) {
                udp_tx.beginPacket(broadcast_ip, SEND_UDP_PORT);
                udp_tx.write(packet, bytes_read);
                udp_tx.endPacket();
            }
        }
    }
    yield();
}