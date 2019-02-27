#include <Wire.h>
#include <PubSubClient.h>
#include <WiFi.h>

// BMP280 I2C address is 0x76(108)
#define Addr 0x76

//Wifi Credentials
#define wifi_ssid "ESSID"
#define wifi_password "Password"

//Define MQTT server and topics
#define mqtt_server "iot.eclipse.org"
#define Ctemp_topic "TempC"
#define Ftemp_topic "TempF"
#define Prsr_topic "Pressure"
#define Altm_topic "MAltitude"
#define AltF_topic "FAltitude"

WiFiClient espClient;
PubSubClient client;

volatile float tempc, tempf, presr, altm, altf;
void setup()
{
    Wire.begin(21,22);
    Serial.begin(115200);
    Serial.println("RX Pin --->"+RX);
    Serial.println("TX Pin --->"+TX);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setClient(espClient);
}

//Wifi Setup
  void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);
  
    WiFi.begin(wifi_ssid, wifi_password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  
  //Reconnect
  void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
        Serial.println("connected");
      } 
      else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(500);
      }
    }
  }

 void loop()
  {
    bmp280();
    delay(100);

    if (!client.connected()) {
    reconnect();
    }
    
    //Mentioned below directly executed in String url
    
    Serial.print("C Temp: ");
    Serial.println(String(tempc).c_str());
    client.publish(Ctemp_topic, String(tempc).c_str(), true);
  
    Serial.print("F Temp: ");
    Serial.println(String(tempf).c_str());
    client.publish(Ftemp_topic, String(tempf).c_str(), true);
    
    Serial.print("Pressure: ");
    Serial.println(String(presr).c_str());
    client.publish(Prsr_topic, String(presr).c_str(), true);
  
    Serial.print("Altitude meters: ");
    Serial.println(String(altm).c_str());
    client.publish(Altm_topic, String(altm).c_str(), true);
  
    Serial.print("Altitude Feet: ");
    Serial.println(String(altf).c_str());
    client.publish(AltF_topic, String(altf).c_str(), true);
    client.loop();
    }

void bmp280()
{
  unsigned int b1[24];
  unsigned int data[8];
  for (int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((136 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }
  // Convert the data
  // temp coefficients
  unsigned int dig_T1 = (b1[0] & 0xFF) + ((b1[1] & 0xFF) * 256);
  int dig_T2 = b1[2] + (b1[3] * 256);
  int dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  unsigned int dig_P1 = (b1[6] & 0xFF) + ((b1[7] & 0xFF) * 256);
  int dig_P2 = b1[8] + (b1[9] * 256);
  int dig_P3 = b1[10] + (b1[11] * 256);
  int dig_P4 = b1[12] + (b1[13] * 256);
  int dig_P5 = b1[14] + (b1[15] * 256);
  int dig_P6 = b1[16] + (b1[17] * 256);
  int dig_P7 = b1[18] + (b1[19] * 256);
  int dig_P8 = b1[20] + (b1[21] * 256);
  int dig_P9 = b1[22] + (b1[23] * 256);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();

  for (int i = 0; i < 8; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((247 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  double cTemp = (var1 + var2) / 5120.0;
  double fTemp = cTemp * 1.8 + 32;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  float height = 44330 * (1 - pow((pressure / 1013.25), 0.1903));
  float h = height * 0.3048;
  
  // Output data to serial monitor
//  Serial.print("Altitude : ");
//  Serial.print(height);
//  Serial.println(" m");
//  Serial.print("Altitude in Feet : ");
//  Serial.println(h);
//  
//  Serial.print("Pressure : ");
//  Serial.print(pressure);
//  Serial.println(" hPa");
//  Serial.print("Temperature in Celsius : ");
//  Serial.print(cTemp);
//  Serial.println(" C");
//  Serial.print("Temperature in Fahrenheit : ");
//  Serial.print(fTemp);
//  Serial.println(" F");

  //volatile float tempc, tempf, presr, altim, altif

  tempc = cTemp;
  tempf = fTemp;
  presr = pressure;
  altm = height;
  altf = h;
  delay(1000);
}
