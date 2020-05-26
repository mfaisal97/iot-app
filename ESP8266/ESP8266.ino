// Load Wi-Fi library
#include <ESP8266WiFi.h>

const char* ssid     = "IIM";
const char* password = "";

WiFiServer server(80);

String header;

unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

String led3State = "On";
String currentRTCReading = "00:00:00";

String readJunk = "";

void setup() {
  Serial.begin(115200);

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
}


void sendString(String str){
  for (int i = 0; i < str.length(); i++){
     delay(1);
     Serial.print(str[i]);
     delay(1);
    }
  }


void updateLedState(){
  readJunk = Serial.readString();
  sendString(";ATLED?;");
  delay(20);
  led3State = Serial.readString();
  if (led3State!="On"){
    led3State = "Off";
    }
  }

void updateTime(){
    readJunk = Serial.readString();
    sendString(";ATTIME?;");
    delay(20);
    currentRTCReading = Serial.readString();
  }

String getTimeFromHeader(String str){
  int timeStart = str.indexOf("set?time=");
  int timeEnd = str.indexOf(" HTTP/1.1");

  String tmStr = "00:00:00";

  if (timeEnd - (timeStart + 9) == 12 && timeStart >=0 && timeEnd >=0 && timeStart <=20){
      tmStr[0] = str[(timeStart + 9)];
      tmStr[1] = str[(timeStart + 9)+1];
      tmStr[3] = str[(timeStart + 9)+5];
      tmStr[4] = str[(timeStart + 9)+6];
      tmStr[6] = str[(timeStart + 9)+10];
      tmStr[7] = str[(timeStart + 9)+11];
    }

    return tmStr;
  }


void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
//    Serial.println("New Client.");           // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
//        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /led/on") >= 0) {
              sendString(";ATLED=1;");
            } else if (header.indexOf("GET /led/off") >= 0) {
              sendString(";ATLED=0;");
            } else if (header.indexOf("GET /time/refresh") >= 0) {
              updateTime();
            }else if (header.indexOf("time/set") >= 0 && header.indexOf("time/set") <=20) {
              sendString(";ATTIME=1");
              sendString(getTimeFromHeader(header));
              sendString(";");
            }else if (header.indexOf("alarm/set") >= 0 && header.indexOf("alarm/set") <=20) {
              sendString(";ATTIME=2");
              sendString(getTimeFromHeader(header));
              sendString(";");
            }

            updateLedState();

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");


            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");

            client.println("<body><h1>Simple iot-app</h1>");

            client.println("<p>Current Led3 State " + led3State + "</p>");
            if (led3State=="Off") {
              client.println("<p><a href=\"/led/on\"><button class=\"button button2\">TURN ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/led/off\"><button class=\"button\">TURN OFF</button></a></p>");
            }

            client.println("<p>Current Time: " + currentRTCReading + "</p>");
            client.println("<p><a href=\"/time/refresh\"><button class=\"button\">Refresh Time</button></a></p>");

            client.println("<form name=\"input\" action=\"time/set\" method=\"get\">");
            client.println("<label for=\"time\">Time:</label><br>");
            client.println("<input name=\"time\" id=\"time\" />");
            client.println("<input class=\"button\" type=\"submit\" value=\"Set time\" />");
            client.println("</form>");

             client.println("<form name=\"input\" action=\"alarm/set\" method=\"get\">");
            client.println("<label for=\"time\">Alarm:</label><br>");
            client.println("<input name=\"time\" id=\"time\" />");
            client.println("<input class=\"button\" type=\"submit\" value=\"Set Alarm\" />");
            client.println("</form>");


            /*
            client.println("<p>Current Time: " + currentRTCReading + "</p>");
            client.println("<p><a href=\"/time/refresh\"><button class=\"button button2\">Refresh Time</button></a></p>");
            */

            client.println("</body></html>");

            client.println();
            break;
          } else {
            currentLine = "";
          }

        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
//    Serial.println("Client disconnected.");
//    Serial.println("");
  }
}
