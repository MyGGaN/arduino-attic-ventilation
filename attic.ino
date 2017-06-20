#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <Time.h>
#include <Ethernet.h>


// Configuration
#define SAFE_INSIDE_RH 70   // Mold risk if rh > 75%, set a little lower to be safe
#define THRESHOLD_AH 100    // Absolute Humidity (cg/m^3) that out air needs to be lower than in air
#define TOO_WARM 50         // If attic gets warmer we could fan out some excessive heat.

// Hardware setup, LEDs should be on ~pins
#define DHT_IN_PIN 7        // DHT11
#define DHT_OUT_PIN 4       // DHT22
#define FAN_PIN 2           // FAN OUT
#define LED_GREEN 6         // Fan is on
#define LED_YELLOW 9        // Power
#define LED_RED 5           // Attic RH not OK
#define SWITCH_A 3          // The switch on the foront panel
#define SWITCH_B 10         // have 3 poles so it needs two pins.

const unsigned long T_INTERVAL = 300000UL;  // How often to poll sensors, default 300000UL (5 minutes)
const int FORCE = 0xF;
const int AUTO = 0xA;
const int OFF = 0x0;
unsigned long tPrev = 0;

float tempIn;
float tempOut;
float rhIn;
float rhOut;
bool fanStatus = false;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

DHT_Unified dhtIn(DHT_IN_PIN, DHT11);
DHT_Unified dhtOut(DHT_OUT_PIN, DHT22);

/* Returns the amount of vapor air can hold at the given temperature in
 *  centi-grams per cubic meter (vg/m^3). This to avoid floating numbers.
 *
 *  @param t: Temperature in °C
 *
 *  Returns -1 if temperature is outside -20 to 70.
 */
short vaporMax(short t) {
  /* For temperatures -20°C to 70°C holds the value of maximum g/m^3*100
   * E.g. maxH[25]/100.0 gives the maximum vapor air can hold at t=5°C.
   */
  const short maxH[91] = {85,94,104,114,126,138,152,167,182,199,217,237,257,279,
          303,328,354,382,412,444,478,513,551,591,633,678,725,774,827,882,941,
          1002,1067,1136,1208,1284,1364,1448,1537,1631,1729,1832,1941,2055,2175,
          2300,2433,2571,2717,2870,3030,3198,3375,3559,3753,3956,4168,4390,4622,
          4865,5120,5386,5663,5954,6257,6574,6904,7249,7609,7984,8376,8783,9208,
          9651,10112,10592,11092,11612,12153,12715,13299,13907,14538,15194,15875,
          16582,17316,18077,18867,19686,20536};

  if (t < -20 || t > 70) {
    // Temperature outside allowed interval
    return -1;
  }
  // The maxH array is offset by 20.
  return maxH[t + 20];
}

int readSwitch() {
  int sw_a = digitalRead(SWITCH_A);
  int sw_b = digitalRead(SWITCH_B);

  if (sw_a == LOW && sw_b == HIGH) {
    return FORCE;
  } else if (sw_a == HIGH && sw_b == HIGH) {
    return AUTO;
  } else if (sw_a == HIGH && sw_b == LOW) {
    return OFF;
  } else {
    // Should never happen, indicates a short circuit or misconfigured pins
  }
}

void printSensorInfo(DHT_Unified &dht) {
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay / 1000); Serial.println("ms");

  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay / 1000); Serial.println("ms");
  Serial.println("------------------------------------");
}

bool readData(DHT_Unified* dht, float* temp, float* rh, char *sensorName) {
  sensors_event_t event;
  dht->temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.print("Error reading temperature "); Serial.println(sensorName);
    return false;
  } else {
    *temp = event.temperature;
  }

  dht->humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.print("Error reading humidity "); Serial.println(sensorName);
    return false;
  } else {
    *rh = event.relative_humidity;
  }

  // Print reading
  Serial.print(sensorName);
  Serial.print(": "); Serial.print(*temp);
  Serial.print("*C, "); Serial.print(*rh); Serial.println("%");

  return true;
}

void setFan(bool status) {
  // Fan is active on low
  fanStatus = status;
  if (status) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(FAN_PIN, LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(FAN_PIN, HIGH);
  }
}

void setAtticLED(bool status) {
  if (status) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
}

int updateSensorData() {
  // Names used for error message
  char inName[] = "in";
  char outName[] = "out";

  if (!readData(&dhtIn, &tempIn, &rhIn, inName) ||
      !readData(&dhtOut, &tempOut, &rhOut, outName)) {
    // Error occured, hope for better luck next time
    return 0;
  }
  return 1;
}

void actOnSensorData() {
  boolean fan = false;

  // Fan for mold protection if it's not too cold or if it's too warm
  if (rhIn > SAFE_INSIDE_RH && tempOut > -20) {
    /* There's risk for mold inside, if the absolute humidity outside is less
     * (with a margin of THRESHOLD_AH) than inside it should be beneficial to
     * start the fan.
     */
    unsigned short ahIn = vaporMax(tempIn) * rhIn;
    unsigned short ahOut = vaporMax(tempOut) * rhOut;
    if (ahOut + THRESHOLD_AH < ahIn) {
      fan = true;
    }

  // Fan to cool down attic, but only if we're in the safe zone of mold
  } else if (tempIn >= TOO_WARM && tempIn <= 70 && rhIn < SAFE_INSIDE_RH) {
    fan = true;
  }

  setFan(fan);
  setAtticLED(rhIn > SAFE_INSIDE_RH);
}

void setup() {
  // Pin configuration
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(SWITCH_A, INPUT);
  pinMode(SWITCH_B, INPUT);

  // Make sure the fan is off
  setFan(false);

  // Turn on all leds to indicate startup
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize sensors
  dhtIn.begin();
  dhtOut.begin();
  delay(500);

  // Print temperature and humidity details
  Serial.println("## INSIDE ##");
  printSensorInfo(dhtIn);
  Serial.println("## OUTSIDE ##");
  printSensorInfo(dhtOut);

  // Start ethernet connection and web server
  Ethernet.begin(mac, ip);
  server.begin();
  delay(500);
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  // Turn off LEDs (except Power LED)
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  updateSensorData();
}

void loop() {
  unsigned long tNow = millis();
  int sw_value = readSwitch();
  if (sw_value == FORCE) {
    setFan(true);
  } else if (sw_value == OFF) {
    setFan(false);
  }

  if (tNow - tPrev > T_INTERVAL) {
    int success = updateSensorData();
    if (success == 1 && sw_value == AUTO) {
      actOnSensorData();
    }
    tPrev = tNow;
  }

  listenForEthernetClients();
}

void listenForEthernetClients() {
  // Listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("Got a client");
    // An http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // If you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // Send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/json");
          client.println();

          unsigned long tNow = millis();
          unsigned long valueAge = tNow - tPrev;
          int sw_value = readSwitch();
          String switchPos;
          if (sw_value == FORCE) {
            switchPos = "Force";
          } else if (sw_value == OFF) {
            switchPos = "Off";
          } else {
            switchPos = "Auto";
          }

          // Print the latest readings, in JSON format:
          client.print("{\"fan\":" + String(fanStatus));
          client.print(",\"rhIn\":" + String(rhIn));
          client.print(",\"rhOut\":" + String(rhOut));
          client.print(",\"switch\":\"" + switchPos + "\"");
          client.print(",\"tempIn\":" + String(tempIn));
          client.print(",\"tempOut\":" + String(tempOut));
          client.print(",\"valueAge\":" + String(valueAge));  // In ms
          client.println("}");

          break;
        }
        if (c == '\n') {
          // You're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // You've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // Give the web browser time to receive the data
    delay(1);
    // Close the connection:
    client.stop();
  }
}
