#include <Arduino.h>
#include <Serial.h>
#include <WiFiS3.h>

#include "SpeakerDriver.h"
#include "MicDriver.h"
#include "secrets.h"

#define RECEIVE_UDP_PORT 6769
#define SEND_UDP_PORT 6769
#define FREQ_HZ 32000.0

#define SPEAKER_PIN A0
#define SPEAKER_DMA_CHANNEL 0

#define MIC_PIN A1
#define MIC_DMA_CHANNEL 1

SpeakerDriver speaker(FREQ_HZ, SPEAKER_PIN, SPEAKER_DMA_CHANNEL);
SpeakerDriver::BufferPtr speaker_buffers = speaker.get_buffers();

MicDriver mic(FREQ_HZ, MIC_PIN, MIC_DMA_CHANNEL);
MicDriver::BufferPtr mic_buffers = mic.get_buffers();

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

    pinMode(D4, INPUT_PULLUP);
}

void loop() {
    static unsigned long last_ip_check = 0;

    static unsigned long last = 0;
    unsigned long now = millis();

    if (digitalRead(D4) == HIGH) {
        // Default operation - play received audio stream from network
        int buffer_num = speaker.reserve_buffer();
        if (buffer_num >= 0) {
            int available = udp.parsePacket();
            if (available == SpeakerDriver::BUFFER_LEN) {
                if (!has_ip || now - last_ip_check > 10000) {
                    // periodically update the ip address to the latest sending remote
                    last_receive_from_ip = udp.remoteIP();
                    has_ip = true;
                    last_ip_check = now;
                    Serial.println(last_receive_from_ip);
                }
                udp.read(speaker_buffers[buffer_num], available);
                speaker.release_buffer(buffer_num, true);
            } else {
                speaker.release_buffer(buffer_num, false);
            }
        }
    } else if (last_receive_from_ip != INADDR_NONE) {
        // Button is pressed, and we have a target IP: Stream mic audio
        int buffer_num = mic.reserve_buffer_for_read();
        if (buffer_num >= 0) {
            udp.beginPacket(last_receive_from_ip, SEND_UDP_PORT);
            udp.write(mic_buffers[buffer_num], MicDriver::BUFFER_LEN);
            udp.endPacket();
            mic.release_buffer(buffer_num, true);
        }
    }

    /*
    if (now - last > 1000) {
        last = now;
        mic.print_debug();
    }
    */
}
