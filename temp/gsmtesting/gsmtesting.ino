#include <SoftwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>

#define SWITCH 3
#define PIR 2
#define MODEM_TX 5
#define MODEM_RX 6
#define DIVG 7
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

  Serial.begin(9600);
  modem.begin(2400);

  modem.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  modem.print("AT+CPIN=");
  modem.println(SIM_PIN);
  updateSerial();

}

bool sent = false;
void loop() {
    if (digitalRead(SWITCH) == 1 && !sent) {
        modem.println("AT+CMGF=1"); // Configuring TEXT mode
        updateSerial();
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
  delay(500);
  while (Serial.available()) 
  {
    modem.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(modem.available()) 
  {
    Serial.write(modem.read());//Forward what Software Serial received to Serial Port
  }
}