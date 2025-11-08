/**
 * Collect images for Edge Impulse image
 * classification / object detection
 *
 * BE SURE TO SET "TOOLS > CORE DEBUG LEVEL = INFO"
 * to turn on debug messages
 */

#define WIFI_SSID "EricR"  // your open network
#define WIFI_PASS "password"
#define HOSTNAME "esp32cam"

#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/extra/esp32/wifi/sta.h>
#include <eloquent_esp32cam/viz/image_collection.h>

using eloq::camera;
using eloq::wifi;
using eloq::viz::collectionServer;

void setup() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("___IMAGE COLLECTION SERVER___");

    // Camera settings for AI Thinker module
    camera.pinout.aithinker();
    camera.brownout.disable();
    // Edge Impulse models use square images (240x240 typical)
    camera.resolution.face();
    camera.quality.high();

    // Initialize camera
    while (!camera.begin().isOk()) {
        Serial.println(camera.exception.toString());
        delay(1000);
    }

    // Connect to open WiFi (no password)
    while (!wifi.connect(WIFI_SSID, WIFI_PASS).isOk()) {
        Serial.println(wifi.exception.toString());
        delay(1000);
    }

    // Start the image collection HTTP server
    while (!collectionServer.begin().isOk()) {
        Serial.println(collectionServer.exception.toString());
        delay(1000);
    }

    Serial.println("Camera OK");
    Serial.println("WiFi OK");
    Serial.println("Image Collection Server OK");
    Serial.println(collectionServer.address());
}

void loop() {

}