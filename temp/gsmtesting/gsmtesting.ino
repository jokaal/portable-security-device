#include <SoftwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>

#define SWITCH 3
#define PIR 2
#define MODEM_TX 5
#define MODEM_RX 6
#define DIVG 7
#define MODEM_RST 8
#define BATTERY_VOLTAGE A0
#define USB_VOLTAGE A1
#define CURRENT A2
#define LED 13

SoftwareSerial modem(MODEM_TX,MODEM_RX);
String SIM_PIN = String("0000");
String NUMBER = String("\"+37253640486\"");

void setup() {
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(PIR, INPUT);
  pinMode(MODEM_TX,INPUT);
  pinMode(MODEM_TX,INPUT);
  pinMode(DIVG,OUTPUT);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,0);
  digitalWrite(MODEM_RST,0);
  _delay_ms(200);
  digitalWrite(MODEM_RST,1);

  Serial.begin(9600);
  modem.begin(9600);

  modem.println("AT+CMEE=2");
  updateSerial();

  modem.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  modem.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  modem.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  modem.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();

  modem.println("AT+CPIN=0000");
  //modem.println(SIM_PIN);
  updateSerial();

 // modem.println("ATD+ +37253640486;"); //call
 // updateSerial();
 // delay(20000); // wait for 20 seconds...
 // modem.println("ATH"); //hang up
 // updateSerial();
  
  modem.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  modem.println("AT+CPMS=\"SM\",\"SM\",\"SM\"");
  updateSerial();
  modem.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  updateSerial();
  modem.println("AT+CSDH=1");
  updateSerial();
  modem.println("AT+CFUN=1");
  updateSerial();

  modem.println("AT+CUSD?");
  updateSerial();
  modem.println("AT+CSCS=\"GSM\"");
  updateSerial();
  modem.println("AT+CUSD?");
  updateSerial();
  modem.println("AT+CUSD=1,\"*143#\"");
  updateSerial();
  delay(10000);
  updateSerial();
}

bool sent = false;
void loop() {
  updateSerial();
  if (digitalRead(SWITCH) == 1 && !sent) {
    modem.print("AT+CMGS=");
    modem.println(NUMBER);
    updateSerial();
    modem.print("Test message"); //text content
    updateSerial();
    modem.write(26);
    sent = true;
  }
}

void updateSerial()
{
  delay(1000);
  while (Serial.available()) 
  {
    modem.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(modem.available()) 
  {
    Serial.write(modem.read());//Forward what Software Serial received to Serial Port
  }
}