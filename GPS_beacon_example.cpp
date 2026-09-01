#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// MicoAir M10 Ultra = genuine u-blox M10 GNSS, factory-configured to 230400 baud.
// It streams UBX (binary) + NMEA on UART. The SparkFun library decodes the
// UBX NAV-PVT message, which carries position, fix, satellites, speed,
// heading and UTC time in a single packet.

HardwareSerial GPS(1);   // UART1: RX = GPIO20, TX = GPIO21
SFE_UBLOX_GNSS myGNSS;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nMicoAir M10 Ultra - UBX parse");

  GPS.begin(230400, SERIAL_8N1, /*RX=*/20, /*TX=*/21, false);

  // Hand the serial port to the library and confirm it's a u-blox module.
  while (myGNSS.begin(GPS) == false) {
    Serial.println("u-blox not detected on UART - retrying...");
    delay(1000);
  }
  Serial.println("u-blox M10 connected.");

  myGNSS.setUART1Output(COM_TYPE_UBX);  // UBX only on UART1 (silence NMEA noise)
  myGNSS.setNavigationFrequency(1);     // 1 solution per second
  myGNSS.setAutoPVT(true);              // module pushes NAV-PVT automatically
  myGNSS.saveConfiguration();           // persist so it survives a power cycle
}

void loop() {
  // getPVT() returns true only when a fresh navigation solution has arrived.
  if (myGNSS.getPVT()) {
    double lat = myGNSS.getLatitude() / 1e7;       // deg
    double lon = myGNSS.getLongitude() / 1e7;      // deg
    double alt = myGNSS.getAltitudeMSL() / 1000.0; // m above mean sea level
    uint8_t siv = myGNSS.getSIV();                 // satellites used
    uint8_t fix = myGNSS.getFixType();             // 0=no,2=2D,3=3D,4=GNSS+DR
    double speed = myGNSS.getGroundSpeed() / 1000.0; // m/s
    double heading = myGNSS.getHeading() / 1e5;    // deg

    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d  ",
                  myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
                  myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());

    const char *fixName[] = {"no fix", "dead-reckon", "2D", "3D", "GNSS+DR", "time-only"};
    Serial.printf("fix=%s SIV=%u  ", (fix <= 5) ? fixName[fix] : "?", siv);

    Serial.printf("lat=%.7f lon=%.7f alt=%.1fm  spd=%.2fm/s hdg=%.1f\n",
                  lat, lon, alt, speed, heading);
  }
}
