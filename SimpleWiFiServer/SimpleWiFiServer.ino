#include <WiFi.h>
#include <MPU6050_light.h>
#include <Adafruit_MCP9808.h>

const char *ssid = "FASTWEB-24C673";
const char *password = "23YKNERG79";

NetworkServer server(80);
int ledPin = 16;
bool imuReadingActive = false;
MPU6050 mpu(Wire);
Adafruit_MCP9808 tempSensor = Adafruit_MCP9808();
float temperature = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);  // LED pin mode

  delay(10);

  // Connect to WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Wire.begin(21, 22);  // Initialize I2C with SDA=21, SCL=22

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully");

  // Initialize MCP9808
  if (!tempSensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808 sensor!");
    while (1);
  }
  Serial.println("MCP9808 initialized successfully");
}

void loop() {
  NetworkClient client = server.accept();

  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);

        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("Click <a href=\"/H\">here</a> to turn the LED ON.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn the LED OFF.<br>");
            client.print("Click <a href=\"/IMU\">here</a> to get the IMU and Temp reading.<br>");
            client.print("Click <a href=\"/IMUoff\">here</a> to stop getting IMU and Temp reading.<br>");
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }

        // Handle GET requests:
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(ledPin, HIGH);  // Turn LED ON
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(ledPin, LOW);  // Turn LED OFF
        }
        if (currentLine.endsWith("GET /IMU")) {
          imuReadingActive = true;  // Start continuous IMU readings
          temperature = tempSensor.readTempC(); 
        }
        if (currentLine.endsWith("GET /IMUoff")) {
          imuReadingActive = false;  // Stop IMU readings
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }

  // Continuously print IMU data if IMU reading is active
  if (imuReadingActive) {
    mpu.update();  // Update MPU readings
    temperature = tempSensor.readTempC();  // Update temperature reading

    // Print IMU data and temperature to Serial Monitor
    Serial.print("ACCEL X: ");
    Serial.print(mpu.getAccX());
    Serial.print(" Y: ");
    Serial.print(mpu.getAccY());
    Serial.print(" Z: ");
    Serial.print(mpu.getAccZ());
    Serial.print(" | GYRO X: ");
    Serial.print(mpu.getGyroX());
    Serial.print(" Y: ");
    Serial.print(mpu.getGyroY());
    Serial.print(" Z: ");
    Serial.print(mpu.getGyroZ());
    Serial.print(" | TEMP: ");
    Serial.print(temperature);
    Serial.println(" °C");

    delay(1000);  // delay to control the printing speed
  }
}
