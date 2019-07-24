#include <EEPROM.h>
#include <n_Base64.h>
#include <n_AES.h>


/*
	Esp8266 Websockets Client

	This sketch:
        1. Connects to a WiFi network
        2. Connects to a Websockets server
        3. Sends the websockets server a message ("Hello Server")
        4. Prints all incoming messages while the connection is open

	Hardware:
        For this sketch you only need an ESP8266 board.

	Created 15/02/2019
	By Gil Maimon
	https://github.com/gilmaimon/ArduinoWebsockets

*/
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
#define RELEY D6


AES aes;
byte key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
byte iv [N_BLOCK] ;
byte my_iv[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte my_iv2[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
StaticJsonDocument<1000> jsonBuffer;
byte cipher[1000];
char b64data[200];
byte cleardata[200];
const char* ssid = "RR_Production"; //Enter SSID
const char* password = "RRakun_2019_Prod"; //Enter Password
const char* websockets_server_host = "10.10.90.100"; //Enter server adress
const uint16_t websockets_server_port = 443; // Enter server port
const char* websockets_connection_string = "wss://10.10.90.100:443"; //Enter server adress
const char* node_id = "0612650abd575ab9b94a26e0d29f20948e838d3812bc79285f45dcad";
const char* node_key = "";
String connected_addr = ""; 
int id = 0;
int state_change = 0;
int heating = 0;
float previous_temp = 16.0f;
float current_temp = 15.0f;
float home_temp = 16.0f;
float away_temp = 10.0f;
int looping = 0;
float distance = 0.0f;
bool atHome = false;
bool heat_manual = false;

using namespace websockets;
Adafruit_BME280 bme;
String msg = "";
WebsocketsClient client;

void setup() {
    EEPROM.begin(512);
    Serial.begin(115200);
    Wire.begin(2,0); //D4,D3
    Serial.println(EEPROM.read(0));
    Serial.println(EEPROM.read(1));
    home_temp = EEPROM.read(0);
    away_temp = EEPROM.read(1);
    bool status;
    status = bme.begin(0x76);  
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      msg = "Error with BMP";
    }
    // Connect to wifi
    WiFi.begin(ssid, password);

    // Wait some time to connect to wifi
    for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
        Serial.println(".");
        delay(1000);
    }

    // Check if connected to wifi
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("No Wifi!");
        return;
    }
    pinMode(RELEY, OUTPUT);
    digitalWrite(RELEY, HIGH);
    Serial.println("Connected to Wifi, Connecting to server.");
    aes.set_key( key , sizeof(key)); 
    // try to connect to Websockets server
//    bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
//    if(connected) {
//        Serial.println("Connecetd!");
//        client.send("Hello Server");
//    } else {
//        Serial.println("Not Connected!");
//    }
    connectWS();
    
    // run callback when messages are received
    client.onMessage([&](WebsocketsMessage message) {
        Serial.print("Got Message: ");
        Serial.println(message.data());
        DeserializationError error = deserializeJson(jsonBuffer, message.data());
          if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.c_str());
            return;
          }

          if (jsonBuffer["packet_type"]=="ACK_LOG_ON"){
            Serial.println("Connected to Broker");
            
          }else if (jsonBuffer["packet_type"]=="MSG") {
            decrypt(jsonBuffer["data"].as<String>(), key, my_iv);
            msg = String(b64data);
            DeserializationError error = deserializeJson(jsonBuffer, b64data);
            if (error) {
              Serial.print(F("deserializeJson() failed: "));
              Serial.println(error.c_str());
              return;
            }
            msg = jsonBuffer[0].as<String>();
            Serial.println(msg);
            if (jsonBuffer[0]["instruction"]=="HOME_set"){
              if (jsonBuffer[0]["home"] == "true"){
                atHome = true;
              }else{atHome = false;}
              distance = jsonBuffer[0]["distance"].as<String>().toInt();
              Serial.println("User at home: "+String(atHome)+" Distance: " +String(distance));
              sendData();
            }else if(jsonBuffer[0]["instruction"]=="SET"){
              home_temp = jsonBuffer[0]["home_temp"].as<String>().toFloat();
              EEPROM.write(0, int(home_temp));
              away_temp = jsonBuffer[0]["away_temp"].as<String>().toFloat();
              EEPROM.write(1, int(away_temp));
              EEPROM.commit();
              if (jsonBuffer[0]["reley"]=="true"){
              heat_manual = true;
              }else{heat_manual = false;}
              sendData();
            }
          }else if (jsonBuffer["packet_type"]=="START") {
            connected_addr = jsonBuffer["transmitter"].as<String>();
            sendData();
            Serial.println("START to "+connected_addr);
          }
    });
}
void connectWS(){
//  bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
  bool connected = client.connect(websockets_connection_string);
    if(connected) {
        Serial.println("Connecetd!");
        //client.send("Sign on as "+ String(node_id));
        //log on
        msg = "{\"packet_type\":\"LOG_ON\",\"transmitter\":\""+String(node_id)+"\",\"receiver\":\""+String(websockets_server_host)+"\"}";
        client.send(msg);
    } else {
        Serial.println("Not Connected!");
    }
}
void loop() {
    // let the websockets client check for incoming messages
    //here the data from sensor will be sent on regular intervals or when the changes happen
    if(client.available()) {
        client.poll();
        if (connected_addr != ""){
          check_temp();
          //sendDataAndDecide();
//          if (Serial.available() > 0) {
//            String readBuffer = Serial.readStringUntil('\n');
//            if (readBuffer=="1"){
//              msg = "{\"packet_type\":\"MSG\",\"transmitter\":\""+String(node_id)+"\",\"receiver\":\""+String(connected_addr)+"\",\"data\":\"";
//              String data = "[{\"name\":\"SEND\",\"value\":\""+String(looping)+"\",\"description\":\"Sending some randon data\"}]";
//              encrypt(data, key, my_iv);
//              client.send(msg+b64data+"\"}");
//              Serial.println("Data send");
//              looping = looping+1;
//              Serial.println("Delay");
//              delay(10);
//            }
//          }
        // Do send new readings
        }
    }else{
    //reconnect
      connectWS();
      check_temp();
    }
    //check themperature anyway so thermostat works
    id = id+1;
    delay(100);
    
}
void check_temp(){
   current_temp = bme.readTemperature();
   float wanted_temp = 10.0f;
   if (atHome || heat_manual){
      wanted_temp = home_temp;
   }else{
      wanted_temp = away_temp;
   }
   if (heating==1){
    wanted_temp += 0.5;
   }else{
    wanted_temp -= 0.5;
   }
   float difference = current_temp - wanted_temp;
   bool change = false;
   if (difference<0){
      digitalWrite(RELEY, LOW);
      if (heating == 0){
        change = true;
      } 
      heating = 1;
   }else{
      digitalWrite(RELEY, HIGH);
      if (heating == 1){
        change = true;
      } 
      heating = 0;
   }
   difference = current_temp - previous_temp;
   if (difference>0.35 || difference<-0.35 || change){
      previous_temp = current_temp;
      //change = true;
      if (connected_addr != ""){
        sendData();
      }
   }
}

void sendData(){
  //sestavi paket
  msg = "{\"packet_type\":\"MSG\",\"transmitter\":\""+String(node_id)+"\",\"receiver\":\""+String(connected_addr)+"\",\"data\":\"";
  String data = "[{\"instruction\":\"STATE\",\"current_temp\":\""+String(current_temp)+"\",\"home_temp\":\""+String(home_temp)+"\",\"away_temp\":\""+String(away_temp)+"\",\"reley\":\""+String(heating)+"\",\"home\":\""+String(atHome)+"\",\"manual\":\""+String(heat_manual)+"\"}]";
  encrypt(data, key, my_iv);
  client.send(msg+b64data+"\"}");
  Serial.println("Data send");
}
void sendDataAndDecide(){
        current_temp = bme.readTemperature();
        Serial.println("TEMP: "+String(current_temp));
        //check if there needs to be heating turn on. 
        float difference = current_temp - home_temp;
        if (difference<0){
          //send temp to server
          //String packet = "{\"Data\":"+String(current_temp)+"}";
          //client.send(packet);
          digitalWrite(RELEY, LOW);
          heating = 1;
          //String packet2 = "{\"Gretje\":\"on\"}";
          //client.send(packet2);
        }else{
          digitalWrite(RELEY, HIGH);
          heating = 0;
        }
        difference = current_temp - previous_temp;
        Serial.print("Difference "+String(difference));
        Serial.println("| Previous " +String(previous_temp));
        if (difference>0.5 || difference<-0.5){
          //send temp to server
          String packet="{\"packet_type\":\"measurment\", \"ID_client\":\""+String(node_id)+"\", \"ID_server\":\""+String(websockets_server_host)+":"+String(websockets_server_port)+"\", \"data\":{ \"measured_temperature\": "+String(current_temp)+", \"wanted_temperature\": "+String(home_temp)+", \"previous_temperature\": "+String(previous_temp)+", \"reley_on\": "+String(heating)+"}}";
          //String packet = "{\"Temp\":"+String(current_temp)+"}";
          client.send(packet);
          Serial.println("Packet Sent");
          previous_temp = current_temp;
          state_change = 0;
        }
}
void encrypt(String msg, byte* key, byte iv[N_BLOCK]){
    // Print the IV
    byte my_iv[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    //base64_encode( b64data, (char *)iv, N_BLOCK);
    int b64len = base64_encode((char*) b64data, (char *)msg.c_str() ,msg.length());
    Serial.println (" Message in B64: " + String(b64data) );
    Serial.println (" The lenght is:  " + String(b64len) );
    
    
    // Encrypt! With AES128, our key and IV, CBC and pkcs7 padding    
    aes.do_aes_encrypt((byte *)b64data, b64len , cipher, key, 128, my_iv);
    
    Serial.println("Encryption done!");
    
    Serial.println("Cipher size: " + String(aes.get_size()));
    
    base64_encode((char*)b64data, (char *)cipher, aes.get_size() );
    Serial.println ("Encrypted data in base64: " + String(b64data) );
    //return String(b64data);
}
void decrypt(String str_enc_data, byte* key, byte iv[N_BLOCK]){
    byte my_iv[N_BLOCK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    //base64_encode( b64data, (char *)iv, N_BLOCK);
    int b64len = base64_decode(b64data, (char *)str_enc_data.c_str(), str_enc_data.length());
    Serial.println("Cipher lenght: "+ String(b64len));
    
        // Decrypt! With AES128, our key and IV, CBC and pkcs7 padding    
    aes.do_aes_decrypt((byte *)b64data, b64len , cleardata, key, 128, my_iv);
    
    Serial.println("Decryption done!");
    
    Serial.println("Cipher size: " + String(aes.get_size()));
    
    base64_decode(b64data, (char *)cleardata, aes.get_size() );
    Serial.println ("Decrypted data in base64: " + String(b64data) );
      
    Serial.println("Done...");
    //return String(b64data);
}
