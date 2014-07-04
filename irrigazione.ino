/*
  remote control for Impianto di Irrigazione
 (C) Roberto Giungato
 03/07/2014
 */

#include <SPI.h>
#include <Ethernet.h>
#include <stdlib.h>
#include <PString.h>
#include <Time.h>

#include <DHT.h>
#include <PubSubClient.h>     // MQTT
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3  // sensore temperatura digitale DS18B20
#define LEDPIN 9      // attivazione conducibilità / illuminazione led
#define TMPPIN A0
#define LIGHTPIN A1
#define BLINKPIN 8
#define BATTPIN A2
#define CONDUPIN A1
#define RELAY1PIN 6
#define TILTPIN 7
#define MOISTUREA A2

#define DHTPIN 5
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

DeviceAddress Thermometer1 = { 
  0x28, 0xEE, 0x64, 0x03, 0x05, 0x00, 0x00, 0x97 }; // sensore di casa
// DeviceAddress Thermometer2 = { 
//  0x28, 0x4C, 0xF6, 0xBD, 0x05, 0x00, 0x00, 0x15};  // sensore Cumiana
DeviceAddress Thermometer2 = { 
  0x28, 0xEE, 0x64, 0x03, 0x05, 0x00, 0x00, 0x97 }; // sensore di casa 

//Creao un array di byte per specificare il mac address
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//creo un array di byte per specificare l'indirizzo ip
byte ip[] = {
  192, 168, 3, 173}; // modificate questo valore in base alla vostra rete - valido per FONERA
byte nameserver[] = {
  8,8,8,8};
byte gateway[] = {
  192,168,3,1 }; // indirizzo FONERA, porta COMPUTER, modificate questo valore in base alla vostra rete
byte subnet[] = {
  255,255,255,0 };

IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov NTP server
const int NTP_PACKET_SIZE= 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
const int timeZone = 1;     // Central European Time

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
unsigned int localPort = 8888;      // local port to listen for UDP packets

EthernetClient ethClient;
String mqttServer = "xxxx.yyyy.com";    
PubSubClient mqttClient("xxxx.yyyy.com", 1883, callback, ethClient); // oggetto per MQTT

unsigned long previousTime = 0;          // last time you connected to the server, in milliseconds
const unsigned long sendingInterval = 10000;  // 120-sec delay between updates to logging service, 720 per 24hrs to keep within the 1000 quota
unsigned long previousTimeL = 0;          // last time you connected to the server, in milliseconds
const unsigned long loopingInterval = 3000;

int lightVal = 0;
int battVal = 0;
int condVal = 0;
int sensorVal = 0;

float temperature;
float temperatureD = 0.0;
float temperatureD2 = 0.0;
int condStatus = 0;
int tiltForce = 0;

float batteria = 0.0;
float conducibilita = 0.0;
float humidity = 0.0;
float voltage = 0.0;
float dewpoint = 0.0;
float moisture = 0.0;

char buffer[30];
PString datetime(buffer, sizeof(buffer));

//creao un oggetto server che rimane in ascolto sulla porta specificata
// EthernetServer WebServer(80);
EthernetClient pc_client;

void setup()
{
  // analogReference(EXTERNAL);
  Serial.begin(9600);
  delay(10000);

  // Start up the OneWirelibrary
  sensors.begin();
  sensors.setResolution(Thermometer1, 10);
  sensors.setResolution(Thermometer2, 10);

  // Start up humidity and temperature sensor
  dht.begin();

  //inizializza lo shield con il mac e l'ip
  Ethernet.begin(mac, ip, nameserver,gateway,subnet);
  // Ethernet.begin(mac);  // per utilizzare DHCP

  Udp.begin(localPort);
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(3600);

  //al PIN light collegheremo la base del transistor BC547
  pinMode(LEDPIN, OUTPUT);
  pinMode(RELAY1PIN, OUTPUT);
  pinMode(TILTPIN, INPUT);
  pinMode(MOISTUREA, INPUT);
  digitalWrite(LEDPIN, LOW);
  digitalWrite(RELAY1PIN, LOW);
  Serial.print("setup terminato");

}

void loop()
{
  unsigned long currentTime = millis();

  // intervallo breve per leggere i valori sottoscritti
  if (currentTime - previousTimeL > loopingInterval) {
    mqttClient.loop(); 	// loop MQTT
    previousTimeL = currentTime;

    // spegni il relay e il led se il TILT è aperto (giù)
    if (digitalRead(TILTPIN) == LOW && tiltForce == 0) {
      digitalWrite(LEDPIN, LOW);
      digitalWrite(RELAY1PIN, LOW);
      condStatus = 0;
    }
  }

  // intervallo lungo per pubblicare
  if (currentTime - previousTime > sendingInterval) {
    previousTime = currentTime;

    // accendo/spengo led
    digitalWrite(BLINKPIN, HIGH);
    delay(200);
    digitalWrite(BLINKPIN, LOW);

    // leggo batteria
    battVal = analogRead(BATTPIN);
    delay(5);
    batteria = (battVal * 4.64 * 88.1) / (20.0 * 1024);
    Serial.print("batteria INT: V ");
    Serial.print(battVal);
    Serial.print("batteria: V ");
    Serial.println(batteria);

    // leggo conducibilita
    condVal = analogRead(CONDUPIN);
    delay(5);
    conducibilita = (condVal * 4.64) / 1024;
    Serial.print("conducibilita: V ");
    Serial.println(conducibilita);

    // leggo temperatura
    sensorVal = analogRead(TMPPIN);
    delay(5);
    voltage = (sensorVal/1024.0) * 5.0;
    temperature = (voltage - .5) * 100;
    Serial.print("temp: C ");
    Serial.println(temperature);

    // leggo umidità
    humidity = getHumidity();
    
    // leggo luminosita
    lightVal = analogRead(LIGHTPIN);
    delay(5);
    Serial.print("light: C ");
    Serial.println(lightVal);

    // leggo da ADC1
    // int valADC1_0 = adc.readADC(0); // read Chanel 0 from MCP3008 ADC
    // Serial.print("Light from MCP3008: ");
    // Serial.println(valADC1_0);

    // leggo temperature OneWire
    sensors.requestTemperatures();

    Serial.print("Temperature 1 is: ");
    temperatureD = printTemperature(Thermometer1);
    Serial.print("\n\r");
    Serial.print("Temperature 2 is: ");
    temperatureD2 = printTemperature(Thermometer2);
    Serial.print("\n\r");
    Serial.println(digitalRead(TILTPIN));

    // calcolo dewpoint
    Serial.print("Dew Point (°C): ");
    dewpoint = dewPoint(temperatureD, humidity);
    Serial.println(dewpoint);
    
    // leggo umidita del suolo
    moisture = analogRead(MOISTUREA);
    Serial.print("Moisture: ");
    Serial.println(moisture);

    datetime = year();
    if (month() < 10) {
      datetime += '0';
    }
    datetime += month();
    if (day() < 10) {
      datetime += '0';
    }
    datetime += day();
    datetime += '_';
    if (hour() < 10) {
      datetime += '0';
    }
    datetime += hour();
    datetime += ':';
    if (minute() < 10) {
      datetime += '0';
    }
    datetime += minute();
    datetime += ':';
    if (second() < 10) {
      datetime += '0';
    }
    datetime += second();
    Serial.println(datetime); 

    Serial.println("INVIO DATI A MQTT");
    do_send_mqtt();	// invio a MQTT
    mqttClient.loop(); 	// loop MQTT

    // delay(10000);
  }
}

void do_send_mqtt() {
  // if you're not connected
  //  then connect again and send data:
  static char stringToBeSent[15];

  char c[2];
  String cs;
  cs=String(condStatus);
  cs.toCharArray(c,2);
  
  Serial.println("pronto per inviare");

  if (mqttClient.connected()) {
    Serial.println("risulta connesso");
    dtostrf(temperatureD, 8,2, stringToBeSent);
    mqttClient.publish("/ics/temp",stringToBeSent);
    delay(100);
    dtostrf(lightVal, 8,2, stringToBeSent);
    mqttClient.publish("/ics/light",stringToBeSent);
    delay(100);
    dtostrf(batteria, 8,2, stringToBeSent);
    mqttClient.publish("/ics/A_batt1",stringToBeSent);
    delay(100);
    dtostrf(humidity, 8,2, stringToBeSent);
    mqttClient.publish("/ics/humi",stringToBeSent);
    delay(100);
    dtostrf(temperatureD2, 8,2, stringToBeSent);
    mqttClient.publish("/ics/temp",stringToBeSent);
    delay(100);
    dtostrf(dewpoint, 8,2, stringToBeSent);
    mqttClient.publish("/ics/dewpoint",stringToBeSent);
    delay(100);
    dtostrf(moisture, 8,2, stringToBeSent);
    mqttClient.publish("/ics/humi2",stringToBeSent);
    delay(100);
    static char datetimeA[18];
    strcpy(datetimeA, datetime);
    mqttClient.publish("/ics/datetime",datetimeA);
    delay(100);
    mqttClient.publish("/ics/pumpstatus", c);
    delay(100);
    mqttClient.publish("/ics/lightstatus", c);
    delay(100);   
    int tiltC = digitalRead(TILTPIN);
    cs=String(tiltC);
    cs.toCharArray(c,2);
    mqttClient.publish("/ics/tiltstatus", c);
    int tiltF = tiltForce;
    cs=String(tiltF);
    cs.toCharArray(c,2);
    mqttClient.publish("/ics/forcetiltstatus", c);
    Serial.println("appena eseguito publish");
  } 
  else {
    Serial.println("risulta NON connesso");
    if (mqttClient.connect("clientAqua", "uuuu", "pppp")) {
      Serial.println("si è connesso");
      mqttClient.publish("/ics/ics","REconnected");
      mqttClient.subscribe("/ics/setpump");
      mqttClient.subscribe("/ics/settiltforce");
    } 
    else {
      Serial.println("Connessione fallita NUOVAMENTE");
    }
  }
  //  } 
}


void callback(char* topic, byte* payload, unsigned int length) {
  // accende pompa e luce
  if (strcmp(topic,"/ics/setpump")==0) { 
        Serial.println("RICEVUTO SETCOND");
    if (payload[0] == '0') {
      digitalWrite(LEDPIN, LOW);
      digitalWrite(RELAY1PIN, LOW);
      condStatus = 0;
    } 
    else if (payload[0] == '1' && (digitalRead(TILTPIN) == HIGH || tiltForce == 1)) {
      digitalWrite(LEDPIN, HIGH);
      digitalWrite(RELAY1PIN, HIGH);
      condStatus = 1;
    }
  } 
  if (strcmp(topic,"/ics/settiltforce")==0) {   
    Serial.println("RICEVUTO TILTFORCE");
    if (payload[0] == '0') {
      tiltForce = 0;
    } 
    else if (payload[0] == '1') {
      tiltForce = 1;
    }
  }
}


float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
    return(0.0);
  } 
  else {
    Serial.print("C: ");
    Serial.print(tempC);
    Serial.print(" F: ");
    Serial.println(DallasTemperature::toFahrenheit(tempC));
    return(tempC);
  }
}

float getHumidity()
{

  float h = dht.readHumidity();

  Serial.println(h);
  return(h);
}


/*-------- NTP code ----------*/

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

float dewPoint(double celsius, double humidity)
{
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
	float T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}











