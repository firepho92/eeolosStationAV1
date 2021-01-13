#include <Arduino.h>

const char apn[]      = "internet.altan";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char simPIN[]   = ""; 

const char server[] = "eeolos.herokuapp.com";
const char resource[] = "/get";
const int  port = 80;


#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
//Energy management
#define I2C_SDA              18
#define I2C_SCL              19
// BME280 pins
#define I2C_SDA_2            21
#define I2C_SCL_2            22

#define SerialMon Serial
#define SerialAT Serial1


#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER   1024

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

TwoWire I2CPower = TwoWire(0);

TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme; 

TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  1800

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); 
  } else {
    I2CPower.write(0x35); 
  }
  return I2CPower.endTransmission() == 0;
}

float MQ135Read() {
  int adcMQ135 = analogRead(A0);
  float voltageMQ135 = adcMQ135 * (5.0 / 1023);
  return voltageMQ135;
}

void setup() {

  SerialMon.begin(115200);

  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);

  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  SerialMon.println("Iniciando modem");
  modem.restart();

  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  

  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("No se encuentra BME");
    while (1);
  }


  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
  }
  else {
    SerialMon.println(" OK");
    SerialMon.println(bme.readTemperature());
    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    if (!client.connect(server, port)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");
      
      SerialMon.println("Performing HTTP GET request...");
      float gas = MQ135Read();
      String httpRequestData = "?temperature=" + String(bme.readTemperature()) + "&humidity=" + String(bme.readHumidity()) + "&pressure=" + String(bme.readPressure() / 100.0F);

      client.print(String("GET ") + resource + httpRequestData + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);

      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 10000L) {
        
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      SerialMon.println();
    
      client.stop();
      SerialMon.println(F("Server disconnected"));
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
    }
  }
  esp_deep_sleep_start();
}