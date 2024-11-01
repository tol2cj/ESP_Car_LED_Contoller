
// Zdroj: https://randomnerdtutorials.com/esp8266-web-server/

// Load Wi-Fi library
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266mDNS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Create an instance of the ADXL345 sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// Replace with your network credentials
const char* ssid     = "ImprezaRC";
const char* password = "CRazerpmI";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
// Output state variables
String outputHazardState = "off";
String outputTurnLHState = "off";
String outputTurnRHState = "off";
String outputLowBeamState = "off";
String outputHighBeamState = "off";
String outputPositionState = "off";
String outputReverseState = "off";
String outputBrakeState = "off";


const int hazardPins[4] = {3, 7, 11, 15};    // Channel 0
const int turnLHPins[2] = {3, 11};      // Channel 1
const int turnRHPins[2] = {7, 15};      // Channel 2
const int lowBeamPins[2] = {2, 6};     // Channel 3
const int highBeamPins[2] = {1, 5};    // Channel 4
const int positionPins[2] = {9, 13};    // Channel 5
const int reversePins[2] = {10, 14};     // Channel 6
const int brakePins[2] = {9, 13};       // Channel 7
const int chaserLpins[3] = {1, 2, 9};
const int chaserRpins[3] = {5, 6, 13};

// Brightness constants
const int hazardBrightness = 0; // Full brightness
const int turnLHBrightness = 0;  // Full brightness
const int turnRHBrightness = 0;  // Full brightness
const int lowBeamBrightness = 3800;  // Lower brightness
const int highBeamBrightness = 3800; // Lower brightness
const int positionBrightness = 3800; // Lower brightness
const int reverseBrightness = 3800;   // Lower brightness
const int brakeBrightness = 0;     // Full brightness
const int chaserBrightness = 0;

// Number of pins for each function
const int hazardSize = sizeof(hazardPins) / sizeof(hazardPins[0]);
const int turnLHSize = sizeof(turnLHPins) / sizeof(turnLHPins[0]);
const int turnRHSize = sizeof(turnRHPins) / sizeof(turnRHPins[0]);
const int lowBeamSize = sizeof(lowBeamPins) / sizeof(lowBeamPins[0]);
const int highBeamSize = sizeof(highBeamPins) / sizeof(highBeamPins[0]);
const int positionSize = sizeof(positionPins) / sizeof(positionPins[0]);
const int reverseSize = sizeof(reversePins) / sizeof(reversePins[0]);
const int brakeSize = sizeof(brakePins) / sizeof(brakePins[0]);
const int chaserLSize = sizeof(chaserLpins) / sizeof(chaserLpins[0]);
const int chaserRSize = sizeof(chaserRpins) / sizeof(chaserRpins[0]);


// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

bool blikat = false;
bool blikLH = false;
bool blikRH = false;
bool modeChaserActive = false;
bool usDRLactive = false;
unsigned long predchoziBlik = 0;
bool stavBlikani = false;

const float ACCEL_THRESHOLD = 0.3; // Negative threshold for brake light
const float ACCEL_UPPER_LIMIT = 5; // Upper limit for hazard lights
const unsigned long HAZARD_TIMEOUT = 5000; // 5 seconds timeout for hazard lights

unsigned long crashStartTime = 0; // Time when hazard lights were activated
bool crashActive = false; // Track if hazard lights are active


void setup() {
  Serial.begin(115200);

  // Vytvoření vlastní WiFi sítě
  Serial.println("Setting up WiFi Access Point...");
  WiFi.softAP(ssid, password);

  // Zobrazí IP adresu ESP jako Access Point
  Serial.println("WiFi Access Point started.");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start webového serveru
  server.begin();

  setupPWM();
  setupADXL();
}

void setupADXL()
{
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected!");
    while (1);
  }

}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  turnIndicator();
  modeChaser();

  sensors_event_t event;
  accel.getEvent(&event);

  // Check for negative acceleration in Y-axis for brake light
  if (event.acceleration.y > ACCEL_THRESHOLD) { // Adjust threshold as needed
    setBrakeLight(true); // Activate brake light
  } else {
    setBrakeLight(false); // Deactivate brake light
  }

  if (outputHazardState == "off")
  {
    // Check for high acceleration in any axis for hazard lights
    if (fabs(event.acceleration.x) > ACCEL_UPPER_LIMIT ||
        fabs(event.acceleration.y) > ACCEL_UPPER_LIMIT ) {
      if (!crashActive) {
        crashActive = true; // Activate hazard state
        crashStartTime = millis(); // Record the time
        setHazardLight(true); // Turn on hazard lights
      }
    } else {
      // If the hazard lights are currently active, check the timeout
      if (crashActive && (millis() - crashStartTime >= HAZARD_TIMEOUT)) {
        crashActive = false; // Reset hazard state
        setHazardLight(false); // Turn off hazard lights
      }
    }
  }


  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Turn on/off controls for different light functions
            if (header.indexOf("GET /hazard/on") >= 0) {
              Serial.println("Hazard lights on");
              setHazardLight(true);
              outputHazardState = "on";
              outputTurnLHState = "off";
              outputTurnRHState = "off";
            } else if (header.indexOf("GET /hazard/off") >= 0) {
              //              Serial.println("Hazard lights off");
              setHazardLight(false);
              outputHazardState = "off";
            } else if (header.indexOf("GET /turnLH/on") >= 0) {
              //              Serial.println("Left Turn Signal on");
              setTurnLH(true);
              outputHazardState = "off";
              outputTurnLHState = "on";
              outputTurnRHState = "off";
            } else if (header.indexOf("GET /turnLH/off") >= 0) {
              //              Serial.println("Left Turn Signal off");
              setTurnLH(false);
              outputTurnLHState = "off";
            } else if (header.indexOf("GET /turnRH/on") >= 0) {
              //              Serial.println("Right Turn Signal on");
              setTurnRH(true);
              outputHazardState = "off";
              outputTurnLHState = "off";
              outputTurnRHState = "on";
            } else if (header.indexOf("GET /turnRH/off") >= 0) {
              //              Serial.println("Right Turn Signal off");
              setTurnRH(false);
              outputTurnRHState = "off";
            } else if (header.indexOf("GET /lowbeam/on") >= 0) {
              //              Serial.println("Low Beam on");
              setLowBeam(true);
              outputLowBeamState = "on";
            } else if (header.indexOf("GET /lowbeam/off") >= 0) {
              //              Serial.println("Low Beam off");
              setLowBeam(false);
              outputLowBeamState = "off";
            } else if (header.indexOf("GET /highbeam/on") >= 0) {
              //              Serial.println("High Beam on");
              setHighBeam(true);
              outputHighBeamState = "on";
            } else if (header.indexOf("GET /highbeam/off") >= 0) {
              //              Serial.println("High Beam off");
              setHighBeam(false);
              outputHighBeamState = "off";
            } else if (header.indexOf("GET /position/on") >= 0) {
              //              Serial.println("Position Light on");
              setPositionLight(true);
              outputPositionState = "on";
            } else if (header.indexOf("GET /position/off") >= 0) {
              //              Serial.println("Position Light off");
              setPositionLight(false);
              outputPositionState = "off";
            } else if (header.indexOf("GET /reverse/on") >= 0) {
              //              Serial.println("Reverse Light on");
              setReverseLight(true);
              outputReverseState = "on";
            } else if (header.indexOf("GET /reverse/off") >= 0) {
              //              Serial.println("Reverse Light off");
              setReverseLight(false);
              outputReverseState = "off";
            } else if (header.indexOf("GET /brake/on") >= 0) {
              //              Serial.println("Brake Light on");
              setBrakeLight(true);
              outputBrakeState = "on";
            } else if (header.indexOf("GET /brake/off") >= 0) {
              //              Serial.println("Brake Light off");
              setBrakeLight(false);
              outputBrakeState = "off";
            } else if (header.indexOf("GET /modeChaser/on") >= 0) {
              setChaserActive(true);
            } else if (header.indexOf("GET /modeChaser/off") >= 0) {
              setChaserActive(false);
              //setHighBeam(false); // Ensure high beams are off when chaser mode is disabled
            }
            else if (header.indexOf("GET /usDRL/on") >= 0) {
              setUSdrl(true);
              setPositionLight(false);
            } else if (header.indexOf("GET /usDRL/off") >= 0) {
              setUSdrl(false);
              //setHighBeam(false); // Ensure high beams are off when chaser mode is disabled
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            //            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            //            client.println("<link rel=\"icon\" href=\"data:,\">");
            //            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            //            client.println(".button { background-color: #7f7f7f; border: none; color: white; padding: 16px 40px; border-radius: 12px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            //            client.println(".button2 {background-color: #FF8000;}</style></head>");
            // Head section with auto-refresh
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // client.println("<meta http-equiv=\"refresh\" content=\"5\">"); // Refresh every 5 seconds
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; max-width: 100%;}");
            client.println(".button { background-color: #7f7f7f; border: none; color: white; padding: 10px 20px; border-radius: 8px; text-decoration: none; font-size: 14px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #FF8000;}</style></head>");
            //            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            //            client.println(".button { background-color: #7f7f7f; border: none; color: white; padding: 16px 40px; border-radius: 12px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            //            client.println(".button2 {background-color: #FF8000;}</style></head>");

            // Web Page Heading
            client.println("<body>");
            client.println("<div style=\"text-align:center;\">");

            // Left Turn Signal
            if (outputTurnLHState == "off") {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/turnLH/on\"><button class=\"button\">LEFT</button></a></div>");
            } else {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/turnLH/off\"><button class=\"button button2\">LEFT</button></a></div>");
            }

            // Hazard Light
            if (outputHazardState == "off") {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/hazard/on\"><button class=\"button\">HAZARD</button></a></div>");
            } else {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/hazard/off\"><button class=\"button button2\">HAZARD</button></a></div>");
            }

            // Right Turn Signal
            if (outputTurnRHState == "off") {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/turnRH/on\"><button class=\"button\">RIGHT</button></a></div>");
            } else {
              client.println("<div style=\"display:inline-block; margin:5px;\"><a href=\"/turnRH/off\"><button class=\"button button2\">RIGHT</button></a></div>");
            }

            client.println("</div>");

            //US DRL
            if (usDRLactive)
            {
              client.println("<p><a href=\"/usDRL/off\"><button class=\"button button2\">US DRL</button></a></p>");
            }
            else
            {
              client.println("<p><a href=\"/usDRL/on\"><button class=\"button\">US DRL</button></a></p>");
            }

            // Low Beam
            if (outputLowBeamState == "off") {
              client.println("<p><a href=\"/lowbeam/on\"><button class=\"button\">LOW BEAM</button></a></p>");
            } else {
              client.println("<p><a href=\"/lowbeam/off\"><button class=\"button button2\">LOW BEAM</button></a></p>");
            }

            // High Beam
            if (outputHighBeamState == "off") {
              client.println("<p><a href=\"/highbeam/on\"><button class=\"button\">HIGH BEAM</button></a></p>");
            } else {
              client.println("<p><a href=\"/highbeam/off\"><button class=\"button button2\">HIGH BEAM</button></a></p>");
            }

            // Position Light
            if (outputPositionState == "off") {
              client.println("<p><a href=\"/position/on\"><button class=\"button\">POSITION LIGHT</button></a></p>");
            } else {
              client.println("<p><a href=\"/position/off\"><button class=\"button button2\">POSITION LIGHT</button></a></p>");
            }

            // Reverse Light
            if (outputReverseState == "off") {
              client.println("<p><a href=\"/reverse/on\"><button class=\"button\">REVERSE LIGHT</button></a></p>");
            } else {
              client.println("<p><a href=\"/reverse/off\"><button class=\"button button2\">REVERSE LIGHT</button></a></p>");
            }

            // Brake Light
            if (outputBrakeState == "off") {
              client.println("<p><a href=\"/brake/on\"><button class=\"button\">BRAKE LIGHT</button></a></p>");
            } else {
              client.println("<p><a href=\"/brake/off\"><button class=\"button button2\">BRAKE LIGHT</button></a></p>");
            }
            if (modeChaserActive) {
              client.println("<p><a href=\"/modeChaser/off\"><button class=\"button button2\">CHASER</button></a></p>");
            } else {
              client.println("<p><a href=\"/modeChaser/on\"><button class=\"button\">CHASER</button></a></p>");
            }

            client.println("</body></html>");

            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

}

int chaserState = 0;  //initial state
unsigned long previousChaserTime = 0; // Track time for chaser function
unsigned long chaserInterval = 0;
unsigned long currentMillis = 0;

void modeChaser()
{
  if (modeChaserActive) {
    currentMillis = millis();

    if (currentMillis - previousChaserTime >= chaserInterval)
    {
      previousChaserTime = currentMillis;
      switch (chaserState)
      {
        case 0:
          setLights(chaserLpins, chaserLSize, true, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 1:
          setLights(chaserLpins, chaserLSize, false, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 2:
          setLights(chaserLpins, chaserLSize, true, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 3:
          setLights(chaserLpins, chaserLSize, false, chaserBrightness);
          chaserInterval = 100;
          chaserState++;
          break;
        case 4:
          setLights(chaserRpins, chaserRSize, true, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 5:
          setLights(chaserRpins, chaserRSize, false, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 6:
          setLights(chaserRpins, chaserRSize, true, chaserBrightness);
          chaserInterval = 50;
          chaserState++;
          break;
        case 7:
          setLights(chaserRpins, chaserRSize, false, chaserBrightness);
          chaserInterval = 100;
          chaserState = 0;
          break;


      }

    }
  }
}



void setupPWM()
{
  pwm.begin();
  pwm.setOutputMode(false);
  pwm.setPWMFreq(500);  // Set to whatever you like, we don't use it in this demo!

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);

  for (uint8_t pin = 0; pin < 16; pin++) {
    pwm.setPWM(pin, 4096, 0);

  }
}

void turnIndicator()
{

  if ((blikat == true) || (blikLH == true) || (blikRH == true))
  {
    if ((millis() - predchoziBlik) >= 333)
    {
      //      Serial.println("blik");
      predchoziBlik += 333;

      if (stavBlikani == false)
      {

        zapnoutBlinkry();
        stavBlikani = true;
      }
      else {

        vypnoutBlinkry();
        stavBlikani = false;
      }
    }
  }
  else
  {
    predchoziBlik = millis();
    if (!usDRLactive) vypnoutBlinkry();
    stavBlikani = false;
  }
}

void vypnoutBlinkry()
{
  setLights(turnLHPins, turnLHSize, false || (usDRLactive && blikRH), turnLHBrightness);
  setLights(turnRHPins, turnRHSize, false || (usDRLactive && blikLH), turnRHBrightness);

}

void zapnoutBlinkry()
{

  setLights(turnLHPins, turnLHSize, blikLH || blikat || usDRLactive, turnLHBrightness);
  setLights(turnRHPins, turnRHSize, blikRH || blikat || usDRLactive, turnRHBrightness);

}

// Set function for each light
void setLights(const int* pinArray, int size, bool state, int brightness) {
  for (int i = 0; i < size; i++) {
    pwm.setPWM(pinArray[i], state ? 0 : 4096, state ? brightness : 0);
  }
}

void setChaserActive(bool state)
{
  modeChaserActive = state;
  if (true == state)
  {
    chaserState = 0;  //initalize cycle
    previousChaserTime = millis();
  }
  else
  {
    setLights(chaserLpins, chaserLSize, false, chaserBrightness);
    setLights(chaserRpins, chaserRSize, false, chaserBrightness);
  }
}



void setUSdrl(bool state)
{
  usDRLactive = state;

  setLights(turnLHPins, turnLHSize, state, turnLHBrightness);
  setLights(turnRHPins, turnRHSize, state, turnRHBrightness);
  setPositionLight(state);

}

void checkUSdrl()
{
  setUSdrl(usDRLactive);
}

void setHazardLight(bool state) {
  //setLights(hazardPins, hazardSize, state, hazardBrightness);

  blikat = state;
  blikLH = false;
  blikRH = false;
  if (false == state) checkUSdrl();
}

void setTurnLH(bool state) {
  //  setLights(turnLHPins, turnLHSize, state, turnLHBrightness);
  blikLH = state;
  blikRH = false;
  //  outputTurnRHState = "off";
  blikat = false;
  //  outputHazardState = "off";
  if (false == state) checkUSdrl();
}

void setTurnRH(bool state) {
  //  setLights(turnRHPins, turnRHSize, state, turnRHBrightness);
  blikRH = state;
  blikLH = false;

  blikat = false;

  if (false == state) checkUSdrl();
}

void setLowBeam(bool state) {
  setLights(lowBeamPins, lowBeamSize, state, lowBeamBrightness);
}

void setHighBeam(bool state) {
  setLights(highBeamPins, highBeamSize, state, highBeamBrightness);
}

void setPositionLight(bool state) {
  if ((outputBrakeState == "on") && (state == true))
  {
  }
  else
  {
    setLights(positionPins, positionSize, state, positionBrightness);
  }
}

void setReverseLight(bool state) {
  setLights(reversePins, reverseSize, state, reverseBrightness);
}

void setBrakeLight(bool state) {
  if ((outputPositionState == "on") && (state == false))
  {
    setLights(positionPins, positionSize, true, positionBrightness);
  }
  else
  {
    setLights(brakePins, brakeSize, state, brakeBrightness);
  }
}
