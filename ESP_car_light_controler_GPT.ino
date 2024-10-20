// Zdroj: https://randomnerdtutorials.com/esp8266-web-server/

// Load Wi-Fi library
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Replace with your network credentials
const char* ssid     = "CTYRLISTEK";
const char* password = "8ctyrlistek1";

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
const int turnLHPins[2] = {3, 7};      // Channel 1
const int turnRHPins[2] = {11, 15};      // Channel 2
const int lowBeamPins[2] = {2, 6};     // Channel 3
const int highBeamPins[2] = {1, 5};    // Channel 4
const int positionPins[2] = {9, 13};    // Channel 5
const int reversePins[2] = {10, 14};     // Channel 6
const int brakePins[2] = {9, 13};       // Channel 7

// Brightness constants
const int hazardBrightness = 0; // Full brightness
const int turnLHBrightness = 0;  // Medium brightness
const int turnRHBrightness = 0;  // Medium brightness
const int lowBeamBrightness = 3800;  // Lower brightness
const int highBeamBrightness = 3800; // Full brightness
const int positionBrightness = 3800; // Lower brightness
const int reverseBrightness = 3800;   // Full brightness
const int brakeBrightness = 0;     // Full brightness

// Number of pins for each function
const int hazardSize = sizeof(hazardPins) / sizeof(hazardPins[0]);
const int turnLHSize = sizeof(turnLHPins) / sizeof(turnLHPins[0]);
const int turnRHSize = sizeof(turnRHPins) / sizeof(turnRHPins[0]);
const int lowBeamSize = sizeof(lowBeamPins) / sizeof(lowBeamPins[0]);
const int highBeamSize = sizeof(highBeamPins) / sizeof(highBeamPins[0]);
const int positionSize = sizeof(positionPins) / sizeof(positionPins[0]);
const int reverseSize = sizeof(reversePins) / sizeof(reversePins[0]);
const int brakeSize = sizeof(brakePins) / sizeof(brakePins[0]);


// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

bool blikat = false;
bool blikLH = false;
bool blikRH = false;
unsigned long predchoziBlik = 0;
bool stavBlikani = false;


void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  //  pinMode(LED_BUILTIN, OUTPUT);
  //  pinMode(output4, OUTPUT);
  // Set outputs to LOW
  // digitalWrite(output5, LOW);
  // digitalWrite(output4, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  setupPWM();
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  turnIndicator();

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
            } else if (header.indexOf("GET /hazard/off") >= 0) {
              Serial.println("Hazard lights off");
              setHazardLight(false);
              outputHazardState = "off";
            } else if (header.indexOf("GET /turnLH/on") >= 0) {
              Serial.println("Left Turn Signal on");
              setTurnLH(true);
              outputTurnLHState = "on";
            } else if (header.indexOf("GET /turnLH/off") >= 0) {
              Serial.println("Left Turn Signal off");
              setTurnLH(false);
              outputTurnLHState = "off";
            } else if (header.indexOf("GET /turnRH/on") >= 0) {
              Serial.println("Right Turn Signal on");
              setTurnRH(true);
              outputTurnRHState = "on";
            } else if (header.indexOf("GET /turnRH/off") >= 0) {
              Serial.println("Right Turn Signal off");
              setTurnRH(false);
              outputTurnRHState = "off";
            } else if (header.indexOf("GET /lowbeam/on") >= 0) {
              Serial.println("Low Beam on");
              setLowBeam(true);
              outputLowBeamState = "on";
            } else if (header.indexOf("GET /lowbeam/off") >= 0) {
              Serial.println("Low Beam off");
              setLowBeam(false);
              outputLowBeamState = "off";
            } else if (header.indexOf("GET /highbeam/on") >= 0) {
              Serial.println("High Beam on");
              setHighBeam(true);
              outputHighBeamState = "on";
            } else if (header.indexOf("GET /highbeam/off") >= 0) {
              Serial.println("High Beam off");
              setHighBeam(false);
              outputHighBeamState = "off";
            } else if (header.indexOf("GET /position/on") >= 0) {
              Serial.println("Position Light on");
              setPositionLight(true);
              outputPositionState = "on";
            } else if (header.indexOf("GET /position/off") >= 0) {
              Serial.println("Position Light off");
              setPositionLight(false);
              outputPositionState = "off";
            } else if (header.indexOf("GET /reverse/on") >= 0) {
              Serial.println("Reverse Light on");
              setReverseLight(true);
              outputReverseState = "on";
            } else if (header.indexOf("GET /reverse/off") >= 0) {
              Serial.println("Reverse Light off");
              setReverseLight(false);
              outputReverseState = "off";
            } else if (header.indexOf("GET /brake/on") >= 0) {
              Serial.println("Brake Light on");
              setBrakeLight(true);
              outputBrakeState = "on";
            } else if (header.indexOf("GET /brake/off") >= 0) {
              Serial.println("Brake Light off");
              setBrakeLight(false);
              outputBrakeState = "off";
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #7f7f7f; border: none; color: white; padding: 16px 40px; border-radius: 12px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #FF8000;}</style></head>");

            // Web Page Heading
            client.println("<body>");

            // Hazard Light
            if (outputHazardState == "off") {
              client.println("<p><a href=\"/hazard/on\"><button class=\"button\">HAZARD</button></a></p>");
            } else {
              client.println("<p><a href=\"/hazard/off\"><button class=\"button button2\">HAZARD</button></a></p>");
            }

            // Left Turn Signal
            if (outputTurnLHState == "off") {
              client.println("<p><a href=\"/turnLH/on\"><button class=\"button\">LEFT TURN</button></a></p>");
            } else {
              client.println("<p><a href=\"/turnLH/off\"><button class=\"button button2\">LEFT TURN</button></a></p>");
            }

            // Right Turn Signal
            if (outputTurnRHState == "off") {
              client.println("<p><a href=\"/turnRH/on\"><button class=\"button\">RIGHT TURN</button></a></p>");
            } else {
              client.println("<p><a href=\"/turnRH/off\"><button class=\"button button2\">RIGHT TURN</button></a></p>");
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
  turnIndicator();
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

  Serial.println("setupPWM");
  //  pwm.setPWM(0, 0, 3800);
  //  pwm.setPWM(1, 0, 3800);
  //  pwm.setPWM(2, 0, 3800);
  //  pwm.setPWM(3, 0, 0); // LH TI
  //  pwm.setPWM(4, 0, 3800);
  //  pwm.setPWM(5, 0, 3800);
  //  pwm.setPWM(6, 0, 3800);
  //  pwm.setPWM(7, 0, 0); //RH TI
  //  pwm.setPWM(8, 0, 3800);
  //  pwm.setPWM(9, 0, 0);
  //  pwm.setPWM(10, 0, 3800); //LH reverse
  //  pwm.setPWM(11, 0, 0);
  //  pwm.setPWM(12, 0, 3800);
  //  pwm.setPWM(13, 0, 0);
  //  pwm.setPWM(14, 0, 3800);
  //  pwm.setPWM(15, 0, 0);
}

void turnIndicator()
{

  if (blikat == true)
  {
    if ((millis() - predchoziBlik) >= 333)
    {
      Serial.println("blik");
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
  else if (blikLH)
  {}
  else if (blikRH)
  {}
  else
  {
    predchoziBlik = millis();
    vypnoutBlinkry();
    stavBlikani = false;
  }
}

void vypnoutBlinkry()
{
  Serial.println("vypnoutBlinkry");
  pwm.setPWM(3, 4096, 0); // LH TI

  pwm.setPWM(7, 4096, 0); //RH TI

  pwm.setPWM(11, 4096, 0);

  pwm.setPWM(15, 4096, 0);
}

void zapnoutBlinkry()
{
  Serial.println("zapnoutBlinkry");
  pwm.setPWM(3, 0, 0); // LH TI

  pwm.setPWM(7, 0, 0); //RH TI

  pwm.setPWM(11, 0, 0);

  pwm.setPWM(15, 0, 0);
}

void dalkyZap()
{
  pwm.setPWM(1, 0, 3800);

  pwm.setPWM(5, 0, 3800);
}

void dalkyVyp()
{
  pwm.setPWM(1, 4096, 0);

  pwm.setPWM(5, 4096, 0);
}

// Set function for each light
void setLights(const int* pinArray, int size, bool state, int brightness) {
  for (int i = 0; i < size; i++) {
    pwm.setPWM(pinArray[i], state ? 0 : 4096, state ? brightness : 0);
  }
}

void setHazardLight(bool state) {
  //setLights(hazardPins, hazardSize, state, hazardBrightness);

  blikat = state;
}

void setTurnLH(bool state) {
  //  setLights(turnLHPins, turnLHSize, state, turnLHBrightness);
  blikLH = state;
}

void setTurnRH(bool state) {
  //  setLights(turnRHPins, turnRHSize, state, turnRHBrightness);
  blikRH = state;
}

void setLowBeam(bool state) {
  setLights(lowBeamPins, lowBeamSize, state, lowBeamBrightness);
}

void setHighBeam(bool state) {
  setLights(highBeamPins, highBeamSize, state, highBeamBrightness);
}

void setPositionLight(bool state) {
  setLights(positionPins, positionSize, state, positionBrightness);
}

void setReverseLight(bool state) {
  setLights(reversePins, reverseSize, state, reverseBrightness);
}

void setBrakeLight(bool state) {
  setLights(brakePins, brakeSize, state, brakeBrightness);
}
