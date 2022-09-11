#include <elapsedMillis.h>
#include <iot_cmd.h>
#include <ESP8266WiFi.h>                                         //include esp8266 wifi library 
#include "ThingSpeak.h"                                          //include thingspeak library
#include <sequencer4.h>                                          //imports a 4 function sequencer 
#include <sequencer1.h>                                          //imports a 1 function sequencer 
#include <Ezo_i2c_util.h>                                        //brings in common print statements
#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>    //include arduinos i2c library
#include <WebSerial.h>
#include <LittleFS.h>
#include <EMailSender.h>

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Arduino.h>

#include <AsyncElegantOTA.h>;

AsyncWebServer server(8081);

// Create a WebSocket object
AsyncWebSocket ws("/ws");

// Web Server HTTP Authentication credentials
const char* http_username = "admin"; 
const char* http_password = "hpass";

//Email settings
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;
EMailSender emailSend("kingsolutionstech@gmail.com", "xszocigarwutmbep");


WiFiClient client;                                              //declare that this device connects to a Wi-Fi network,create a connection to a specified internet IP address

//----------------Fill in your Wi-Fi / ThingSpeak Credentials-------
const String ssid = "Braedens Wifi-2G";                                 //The name of the Wi-Fi network you are connecting to
const String pass = "airairair";                             //Your WiFi network password
const long myChannelNumber = 1421791;                            //Your Thingspeak channel number
const char * myWriteAPIKey = "3A64L7FRA9WOQXES";                 //Your ThingSpeak Write API Key
//------------------------------------------------------------------

elapsedMillis millisSinceLastPump = 0;

bool PH_UP_LIMIT_HIT = false;
bool EC_LIMIT_HIT = false;
bool PH_DOWN_LIMIT_HIT = false;


String PUMPS_DOSED = "";

const float LOW_PH_TARGET = 5.8;    //Target to dose down to if high limit hit (pump gap time of 5 mins, dose 2.5ml, diluted PH down, overshoots this by 0.2 pH)
const float HIGH_PH_TARGET = 6.2;
const float EC_TARGET = 1500;       //under this EC value will trigger EC pump/s
const float PUMP_GAP_TIME = 900000; //900000 = 15 mins,600000 = 10 mins, 300000 = 5 min; 60000 = 1 min//min time in milliseconds between pump runs
int NUMBER_OF_PUMPS;

Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"
Ezo_board RTD = Ezo_board(102, "RTD");    //create an RTD (temp?) circuit object who's address is 102 and name is "RTD"

Ezo_board EC_A_PUMP = Ezo_board(104, "EC_A_PUMP");    //create an PMP circuit object who's address is 103 and name is "EC_A_PUMP"
Ezo_board EC_B_PUMP = Ezo_board(105, "EC_B_PUMP");    //create an PMP circuit object who's address is 104 and name is "EC_B_PUMP"
Ezo_board PH_UP_PUMP = Ezo_board(106, "PH_UP_PUMP");    //create an PMP circuit object who's address is 105 and name is "pH_UP_PUMP"
Ezo_board PH_DOWN_PUMP = Ezo_board(107, "PH_DOWN_PUMP");    //create an PMP circuit object who's address is 106 and name is "pH_DOWN_PUMP"

const int EC_PUMP_COUNT = 2;

Ezo_board EC_PUMP_boards[EC_PUMP_COUNT] = {  //Array of EC pumps to allow passing both EC pumps OR 1 PH pump to single pump_function
  EC_A_PUMP,
  EC_B_PUMP
};

Ezo_board PH_DOWN_PUMP_boards[1] = {
  PH_DOWN_PUMP
};

Ezo_board PH_UP_PUMP_boards[1] = {
  PH_UP_PUMP
};

Ezo_board device_list[] = {   //an array of boards used for sending commands to all or specific boards
  PH,
  EC,
  RTD,
  EC_A_PUMP,
  EC_B_PUMP,
  PH_UP_PUMP,
  PH_DOWN_PUMP
};

Ezo_board* default_board = &device_list[0]; //used to store the board were talking to

//gets the length of the array automatically so we dont have to change the number every time we add new boards
const uint8_t device_list_len = sizeof(device_list) / sizeof(device_list[0]);

//enable pins for each circuit
const int EN_PH = 14;
const int EN_EC = 12;
const int EN_RTD = 15;
const int EN_AUX = 13;


const unsigned long reading_delay = 1000;                 //how long we wait to receive a response, in milliseconds
const unsigned long thingspeak_delay = 15000;             //how long we wait to send values to thingspeak, in milliseconds

unsigned int poll_delay = 2000 - reading_delay * 2 - 300; //how long to wait between polls after accounting for the times it takes to send readings

//parameters for setting the pump output
#define PUMP_EC_A_BOARD        EC_A_PUMP       //the pump that will do the output (if theres more than one)
#define PUMP_EC_B_BOARD        EC_B_PUMP       //the pump that will do the output (if theres more than one)
#define PH_PUMP_DOSE         10      //the dose that a pH pump will dispense in  milliliters
#define EC_PUMP_DOSE         2.5      //the dose that the nute pumps will dispense in  milliliters
#define EZO_EC_BOARD         EC        //the circuit that will be the target of comparison
#define EC_COMPARISON_VALUE  1400      //value must be under EC_TARGET, so pump function knows which comparison to do ( < / > )

#define PUMP_PH_DOWN_BOARD        PH_DOWN_PUMP       //the pump that will do the output (if theres more than one)s
#define EZO_PH_BOARD         PH        //the circuit that will be the target of comparison
#define PH_DOWN_COMPARISON_VALUE  6.5      //the threshold above or below which the pump is activated

#define PUMP_PH_UP_BOARD        PH_UP_PUMP       //the pump that will do the output (if theres more than one)s
#define PH_UP_COMPARISON_VALUE  5.5     //the threshold above or below which the pump is activated



float k_val = 1;                                          //holds the k value for determining what to print in the help menu

bool polling  = true;                                     //variable to determine whether or not were polling the circuits
bool send_to_thingspeak = true;                           //variable to determine whether or not were sending data to thingspeak

bool wifi_isconnected() {                           //function to check if wifi is connected
  return (WiFi.status() == WL_CONNECTED);
}

void(* resetFunc) (void) = 0; // create a standard reset function
 

void reconnect_wifi() {                                   //function to reconnect wifi if its not connected
  if (!wifi_isconnected()) {
    WiFi.begin(ssid, pass);
    Serial.println("connecting to wifi");
  }
}

//For basic auth
void notifyClients(String state) { 
  ws.textAll(state);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) { 
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char*)data, "bON") == 0) { 
      //ledState = 1;
      notifyClients("ON");
    }
    if (strcmp((char*)data, "bOFF") == 0) {
      //ledState = 0;
      notifyClients("OFF");
   }

  } 
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
break;
case WS_EVT_DISCONNECT:
Serial.printf("WebSocket client #%u disconnected\n", client->id()); break;
case WS_EVT_DATA: handleWebSocketMessage(arg, data, len); break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
break; }
}
void initWebSocket() { ws.onEvent(onEvent); server.addHandler(&ws);
}


void initFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS"); 
  }
  Serial.println("LittleFS mounted successfully"); 
}

// Replaces placeholder with LED state value, this is left over from basic auth
String processor(const String& var) { if(var == "STATE") {
if(1) { 
  //ledState = 0;
return "ON";
} else{
//ledState = 1;
      return "OFF";
    }
  }
  return String();
 
}


//WebSerial input, then act if inputed text = xyz
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  Serial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
//  if (d == "HELP"){     //If we type in help to webserial, do stuff
//    print_help();
//  }
//  if (d=="OFF"){
//    digitalWrite(LED, HIGH);
//  }
}


void thingspeak_send() {
  if (send_to_thingspeak == true) {                                                    //if we're datalogging
    if (wifi_isconnected()) {
      int return_code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if (return_code == 200) {                                                          //code for successful transmission
        Serial.println();
        Serial.println("sent to thingspeak");
      } else {
        Serial.println("couldnt send to thingspeak");
      }
      PUMPS_DOSED = "";
    }
  }
}

void step1();      //forward declarations of functions to use them in the sequencer before defining them
void step2();
void step3();
void step4();
Sequencer4 Seq(&step1, reading_delay,   //calls the steps in sequence with time in between them
               &step2, 300,
               &step3, reading_delay,
               &step4, poll_delay);

Sequencer1 Wifi_Seq(&reconnect_wifi, 10000);  //calls the wifi reconnect function every 10 seconds

Sequencer1 Thingspeak_seq(&thingspeak_send, thingspeak_delay); //sends data to thingspeak with the time determined by thingspeak delay



void setup() {

  Serial.begin(9600);
  pinMode(EN_PH, OUTPUT);                                                         //set enable pins as outputs
  pinMode(EN_EC, OUTPUT);
  pinMode(EN_RTD, OUTPUT);
  pinMode(EN_AUX, OUTPUT);
  digitalWrite(EN_PH, LOW);                                                       //set enable pins to enable the circuits
  digitalWrite(EN_EC, LOW);
  digitalWrite(EN_RTD, HIGH);
  digitalWrite(EN_AUX, LOW);

  Wire.begin();                           //start the I2C                    

  WiFi.mode(WIFI_STA);                    //set ESP8266 mode as a station to be connected to wifi network
  ThingSpeak.begin(client);               //enable ThingSpeak connection
  Wifi_Seq.reset();                       //initialize the sequencers
  Wifi_Seq.run();
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  AsyncElegantOTA.begin(&server, http_username, http_password);
  initFS();
  initWebSocket();

  EMailSender::EMailMessage message;

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { 
    if(!request->authenticate(http_username, http_password))
      return request->requestAuthentication();
    request->send(LittleFS, "/index.html", "text/html",false, processor);
  });
  server.on("/logged-out", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send(LittleFS, "/logged-out.html", "text/html",false, processor);
  });
  server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send(401);
  });
  server.serveStatic("/", LittleFS, "/").setAuthentication(http_username, http_password);


  server.begin();
  Seq.reset();
  Thingspeak_seq.reset();
  delay(5000);                           //delay to allow power up of sensor circuits, avoid trash first reading
  Serial.println("Device booted");
  WebSerial.println("Device booted");
  ThingSpeak.setField(5, "Device booted.");

  message.subject = "Arduino Booted!";
  message.message = "Email sent from void.setup()";

  EMailSender::Response resp = emailSend.send("kingsolutionstech@gmail.com", message);

  WebSerial.println("Sending status: ");

  WebSerial.println(resp.status);
  WebSerial.println(resp.code);
  WebSerial.println(resp.desc);



//Start of elegant OTA setup
//    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "Don't look at me!");
//    });
//    AsyncElegantOTA.begin(&server, http_username, http_username); // Start ElegantOTA server.begin();
//    Serial.println("HTTP server started");
//    
}


void loop() {
  String cmd;                            //variable to hold commands we send to the kit
  ws.cleanupClients();
  AsyncElegantOTA.loop(); 
  Wifi_Seq.run();                        //run the sequncer to do the polling

  if (receive_command(cmd)) {            //if we sent the kit a command via serial it gets put into the cmd variable
    polling = false;                     //we stop polling
    send_to_thingspeak = false;          //and sending data to thingspeak
    if (!process_coms(cmd)) {            //then we evaluate the cmd for kit specific commands
      process_command(cmd, device_list, device_list_len, default_board);    //then if its not kit specific, pass the cmd to the IOT command processing function
    }
  }

  if (polling == true) {                 //if polling is turned on, run the sequencer
    Seq.run();
    Thingspeak_seq.run();
  }
}

//function that controls the pumps activation and output
void pump_function(Ezo_board pump[], Ezo_board &sensor, float limit, float dose, float target, bool &LIMIT_HIT, int NUM_OF_PUMPS, String &PUMPS_DOSED) {
  if (sensor.get_error() == Ezo_board::SUCCESS) {                    //make sure we have a valid reading before we make any decisions
    bool comparison = false;                                        //variable for holding the reuslt of the comparison
    if (limit > target) {                                             //we do different comparisons depending on what the user wants
      comparison = (sensor.get_last_received_reading() >= limit);   //compare the reading of the circuit to the comparison limit to determine whether we actiavte the pump
    } else {
      comparison = (sensor.get_last_received_reading() <= limit);
    }


    if (comparison || LIMIT_HIT) {                                               //if the result of the comparison means we should activate the pump
      for (int i = 0; i < NUM_OF_PUMPS; i++) {
        pump[i].send_cmd_with_num("d,", dose);                           //dispense the dose
        delay(100);                                                   //wait a few milliseconds before getting pump results
        PUMPS_DOSED += String(pump[i].get_name());
        ThingSpeak.setField(4, String(PUMPS_DOSED));           //set field 4 to name of pump that dispensed
        Serial.print("PUMPS DOSED = " + String(PUMPS_DOSED));
        Serial.print("pump[i].get_name()" + String(pump[i].get_name()));                                //get pump data to tell the user if the command was received successfully
        Serial.print(" ");
        char response[20];
        if (pump[i].receive_cmd(response, 20) == Ezo_board::SUCCESS) {
          Serial.print("pump dispensed ");
        } else {
          Serial.print("pump error ");
        }
        Serial.println(response);

        if (limit > target) {                                              
          LIMIT_HIT = (sensor.get_last_received_reading() >= target);   //compare the reading of the circuit to the target to see if we have raised enough
        } else {
          LIMIT_HIT = (sensor.get_last_received_reading() <= target);
        }
        
      }
    } else {
      for (int i = 0; i < NUM_OF_PUMPS; i++) {
        pump[i].send_cmd("x");                                          //if we're not supposed to dispense, stop the pump
        Serial.println("stop pump sent");
      }
    }
  }
}

void step1() {
  //send a read command. we use this command instead of RTD.send_cmd("R");
  //to let the library know to parse the reading
  Serial.println();
  RTD.send_read_cmd();
  Serial.println();
}

void step2() {
  Serial.println();
  receive_and_print_reading(RTD);             //get the reading from the RTD circuit
  Serial.println();

  if ((RTD.get_error() == Ezo_board::SUCCESS) && (RTD.get_last_received_reading() > -1000.0)) { //if the temperature reading has been received and it is valid
    PH.send_cmd_with_num("T,", RTD.get_last_received_reading());                        //send  temp to pH sensor
    EC.send_cmd_with_num("T,", RTD.get_last_received_reading());                        //send  temp to EC sensor
    ThingSpeak.setField(3, String(RTD.get_last_received_reading(), 2));                 //assign temperature readings to the third column of thingspeak channel
    WebSerial.println("Readings from OTA"); 
    WebSerial.print("Temp: "); 
    WebSerial.println(RTD.get_last_received_reading());
  } else {                                                                                      //if the temperature reading is invalid
    PH.send_cmd_with_num("T,", 25.0);                                                           //send default temp = 25 deg C to pH sensor
    EC.send_cmd_with_num("T,", 25.0);                                                          //send default temp = 25 deg C to EC sensor
    ThingSpeak.setField(3, String(25.0, 2));                 //assign default temperature readings to the third column of thingspeak channel
  }
}

void step3() {
  //send a read command. we use this command instead of PH.send_cmd("R");
  //to let the library know to parse the reading
  PH.send_read_cmd();
  EC.send_read_cmd();
}

void step4() {
  receive_and_print_reading(PH);             //get the reading from the PH circuit
  if (PH.get_error() == Ezo_board::SUCCESS) {                                          //if the PH reading was successful (back in step 1)
    ThingSpeak.setField(1, String(PH.get_last_received_reading(), 2));                 //assign PH readings to the first column of thingspeak channel
    WebSerial.print("pH: "); 
    WebSerial.println(PH.get_last_received_reading());
  }
  Serial.println();
  receive_and_print_reading(EC);             //get the reading from the EC circuit
  if (EC.get_error() == Ezo_board::SUCCESS) {                                          //if the EC reading was successful (back in step 1)
    ThingSpeak.setField(2, String(EC.get_last_received_reading(), 0));                 //assign EC readings to the second column of thingspeak channel
    WebSerial.print("EC: "); 
    WebSerial.println(EC.get_last_received_reading());
  }

  Serial.println();
  WebSerial.println();



if (millisSinceLastPump > PUMP_GAP_TIME) {          //IF more than PUMP_GAP_TIME mins since last pump ran, check if pumps should run
  millisSinceLastPump = 0; 
   
  Serial.println("Calling EC pump_function");
  NUMBER_OF_PUMPS = 2;        
  pump_function(EC_PUMP_boards, EZO_EC_BOARD, EC_COMPARISON_VALUE, EC_PUMP_DOSE, EC_TARGET, EC_LIMIT_HIT, NUMBER_OF_PUMPS, PUMPS_DOSED);
   

  Serial.println("Calling PH UP pump_function");

  NUMBER_OF_PUMPS = 1;
  pump_function(PH_UP_PUMP_boards, EZO_PH_BOARD, PH_UP_COMPARISON_VALUE, PH_PUMP_DOSE, HIGH_PH_TARGET, PH_UP_LIMIT_HIT, NUMBER_OF_PUMPS, PUMPS_DOSED);
 
 
  Serial.println("Calling PH DOWN pump_function");
  NUMBER_OF_PUMPS = 1;
  pump_function(PH_DOWN_PUMP_boards, EZO_PH_BOARD, PH_DOWN_COMPARISON_VALUE, PH_PUMP_DOSE, LOW_PH_TARGET, PH_DOWN_LIMIT_HIT, NUMBER_OF_PUMPS, PUMPS_DOSED);

  }


}



void start_datalogging() {
  polling = true;                                                 //set poll to true to start the polling loop
  send_to_thingspeak = true;
  Thingspeak_seq.reset();
}

bool process_coms(const String &string_buffer) {      //function to process commands that manipulate global variables and are specifc to certain kits
  if (string_buffer == "HELP") {
    print_help();
    return true;
  }
  else if (string_buffer.startsWith("DATALOG")) {
    start_datalogging();
    return true;
  }
  else if (string_buffer.startsWith("POLL")) {
    polling = true;
    Seq.reset();

    int16_t index = string_buffer.indexOf(',');                    //check if were passing a polling delay parameter
    if (index != -1) {                                              //if there is a polling delay
      float new_delay = string_buffer.substring(index + 1).toFloat(); //turn it into a float

      float mintime = reading_delay * 2 + 300;
      if (new_delay >= (mintime / 1000.0)) {                                     //make sure its greater than our minimum time
        Seq.set_step4_time((new_delay * 1000.0) - mintime);          //convert to milliseconds and remove the reading delay from our wait
      } else {
        Serial.println("delay too short");                          //print an error if the polling time isnt valid
      }
    }
    return true;
  }
  else if (string_buffer == "reset") {
    resetFunc(); 
  }
  return false;                         //return false if the command is not in the list, so we can scan the other list or pass it to the circuit
}

void get_ec_k_value() {                                   //function to query the value of the ec circuit
  char rx_buf[10];                                        //buffer to hold the string we receive from the circuit
  EC.send_cmd("k,?");                                     //query the k value
  delay(300);
  if (EC.receive_cmd(rx_buf, 10) == Ezo_board::SUCCESS) { //if the reading is successful
    k_val = String(rx_buf).substring(3).toFloat();        //parse the reading into a float
  }
}

void print_help() {
  get_ec_k_value();
  Serial.println(F("Atlas Scientific I2C hydroponics kit                                       "));
  Serial.println(F("Commands:                                                                  "));
  Serial.println(F("datalog      Takes readings of all sensors every 15 sec send to thingspeak "));
  Serial.println(F("             Entering any commands stops datalog mode.                     "));
  Serial.println(F("poll         Takes readings continuously of all sensors                    "));
  Serial.println(F("                                                                           "));
  Serial.println(F("ph:cal,mid,7     calibrate to pH 7                                         "));
  Serial.println(F("ph:cal,low,4     calibrate to pH 4                                         "));
  Serial.println(F("ph:cal,high,10   calibrate to pH 10                                        "));
  Serial.println(F("ph:cal,clear     clear calibration                                         "));
  Serial.println(F("                                                                           "));
  Serial.println(F("ec:cal,dry           calibrate a dry EC probe                              "));
  Serial.println(F("ec:k,[n]             used to switch K values, standard probes values are 0.1, 1, and 10 "));
  Serial.println(F("ec:cal,clear         clear calibration                                     "));

  if (k_val > 9) {
    Serial.println(F("For K10 probes, these are the recommended calibration values:            "));
    Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
    Serial.println(F("  ec:cal,high,150000   calibrate EC probe to 150,000us                   "));
  }
  else if (k_val > .9) {
    Serial.println(F("For K1 probes, these are the recommended calibration values:             "));
    Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
    Serial.println(F("  ec:cal,high,80000    calibrate EC probe to 80,000us                    "));
  }
  else if (k_val > .09) {
    Serial.println(F("For K0.1 probes, these are the recommended calibration values:           "));
    Serial.println(F("  ec:cal,low,84        calibrate EC probe to 84us                        "));
    Serial.println(F("  ec:cal,high,1413     calibrate EC probe to 1413us                      "));
  }

  Serial.println(F("                                                                           "));
  Serial.println(F("rtd:cal,t            calibrate the temp probe to any temp value            "));
  Serial.println(F("                     t= the temperature you have chosen                    "));
  Serial.println(F("rtd:cal,clear        clear calibration                                     "));

}
