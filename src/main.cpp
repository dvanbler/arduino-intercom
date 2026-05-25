#include <Arduino.h>
#include <Serial.h>
#include <WiFiS3.h>

#include "MicDriver.h"
#include "SpeakerDriver.h"
#include "secrets.h"

#define RECEIVE_UDP_PORT 6769
#define SEND_UDP_PORT 6770
#define RECEIVE_FREQ 32000.0
#define SEND_FREQ 11025.0

#define SPEAKER_PIN A0
#define SPEAKER_DMA_CHANNEL 0

#define MIC_PIN A1
#define MIC_DMA_CHANNEL 1

#define PTT_PIN A2

SpeakerDriver speaker(RECEIVE_FREQ, SPEAKER_PIN, SPEAKER_DMA_CHANNEL);
SpeakerDriver::BufferPtr speaker_buffers = speaker.get_buffers();

MicDriver mic(SEND_FREQ, MIC_PIN, MIC_DMA_CHANNEL);

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
    auto speaker_rv = speaker.begin();
    if (speaker_rv != SpeakerDriver::BeginStatus::SUCCESS) {
        return -1;
    }
    Serial.println(" [√]");

    Serial.print("Init mic driver...");
    auto mic_rv = mic.begin();
    if (mic_rv != MicDriver::BeginStatus::SUCCESS) {
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

    pinMode(PTT_PIN, INPUT_PULLUP);
}

void loop() {
    static unsigned long last_ip_check = 0;

    unsigned long now = millis();

    bool ptt_pin_high = digitalRead(PTT_PIN) == HIGH;
    bool ptt_pressed = !ptt_pin_high;

    if (!ptt_pressed) {
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

            int buffer_num = speaker.reserve_buffer();
            if (buffer_num >= 0) {
                int bytes_to_read = min(available, SpeakerDriver::BUFFER_LEN);
                udp.read(speaker_buffers[buffer_num], bytes_to_read);
                if (available > SpeakerDriver::BUFFER_LEN) {
                    // discard any remaining bytes in oversized packet
                    udp.flush();
                }
                if (bytes_to_read < SpeakerDriver::BUFFER_LEN) {
                    for (int i = bytes_to_read; i < SpeakerDriver::BUFFER_LEN; i++) {
                        speaker_buffers[buffer_num][i] = 0;
                    }
                }

                speaker.release_buffer(buffer_num, true);
            } else {
                udp.flush();
            }
        }
    } else {
        // PTT button is pressed

        // Ensure we flush out incoming packets while we are streaming out
        while (udp.parsePacket() > 0) {
            udp.flush();
        }

        if (last_receive_from_ip != INADDR_NONE) {
            // We have a target IP: Stream mic audio
            uint8_t* packet = mic.read_packet();
            if (packet != nullptr) {
                udp.beginPacket(last_receive_from_ip, SEND_UDP_PORT);
                udp.write(packet, MicDriver::BUFFER_LEN);
                udp.endPacket();
                yield();
            }
        }
    }
}
