#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// Wi-Fi configuration
const char* ssid = "NUTECH-Staff";
const char* password = "Nut@ch@765";
const int ledPin = 0;
unsigned long previousTime =0;
unsigned long totalTime =0;
// UART configuration
const int UART_BAUD_RATE = 115200;
// TCP server configuration
WiFiServer server(8888);
// UART communication
SoftwareSerial uart(1, 3); // RX, TX (Connect ESP8266 TX to Arduino RX and vice versa)

void setup() 
{
  // Initialize UART communication
  pinMode(ledPin, OUTPUT);
  //uart.begin(UART_BAUD_RATE);
  Serial.begin(115200);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  // Start TCP server
  server.begin();
  Serial.println("TCP server started.");
  Serial.print("IP Address ");
  Serial.println(WiFi.localIP());
  digitalWrite(ledPin, true); 
}

void loop() 
{
  // Check for incoming client connections
  WiFiClient client = server.available();
  if (client) 
  {
    // Read data from the client
    while (client.connected()) 
    {
      while (client.available()) 
      {
        String data = client.readStringUntil('\n');
        data.trim();
        Serial.println(data);
      }

    }
  }
}
