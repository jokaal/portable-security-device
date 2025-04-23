#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avrserial.hpp>
#include <sim800.hpp>

#define PHONE_NUMBER "+37253640486"
#define SIM_PIN "0000"

#define MAX_MESSAGES 64 // in cache

#define MODEM_TX 6
#define MODEM_RX 5
#define MODEM_RESET 8
#define SWITCH 3
#define PIR 2
#define DIVG 7 // make low for measuring battery voltage, high for minimal power waste
#define BATTERY_VOLTAGE A0
#define USB_VOLTAGE A1
#define CURRENT A2 // only on one board for development
#define LED 13

#define INTER_DELAY 250
#define SWITCH_DELAY 10000 // delay when switch is turned on in ms
#define PIR_DELAY 60000 // delay between each notification in ms

unsigned long prevMillis;
int time_count = 0;
int sample_array[8];
int i = 0;
int pirStateCurrent = 0;
int pirStatePrev = 0;

volatile int16_t adc[9];
volatile uint8_t adc_complete;
volatile uint8_t tick;
volatile uint8_t pir_state;
volatile uint8_t switch_state;
volatile uint8_t nsamples;
uint8_t ltick;

bool lowMsgSent = false; // battery low
bool criticalMsgSent = false; // battery critical

unsigned long lastMotionTime = 0;
bool pirArmed = false;
unsigned long switchOnTime = 0;

uint16_t bitcount;
uint8_t modem_tick;

DateTime syncedTime;
bool startCount = false;

// FIFO
DateTime message_cache[MAX_MESSAGES];
uint8_t messageHead = 0;
uint8_t messageTail = 0;
uint8_t pendingMessages = 0;

UART uart;

class : public SIM800
{
   void writebit(uint8_t b) { digitalWrite(MODEM_TX,b); }
   
   uint8_t readbit() { return digitalRead(MODEM_RX); }

   void hardware_reset()
   {
      digitalWrite(MODEM_RESET,0);
      _delay_ms(100);
      digitalWrite(MODEM_RESET,1);      
   }

   virtual const char *get_pin()
   {
      return SIM_PIN; 
   }
   
   virtual void sync_time(DateTime& ts)
   {
     uart.printf("Network time: %d-%d-%d %d:%d:%d\r\n",ts.year,
       ts.month,ts.day,ts.hour,ts.minute,ts.second);
     syncedTime = ts;
     startCount = true;
   }
   
   void user_message(const char *msg)
   {
     uart.puts(msg);
     uart.puts("\r\n");
   }
   
   void user_message_P(const char *msg) 
   {
     uart.puts_P(msg);
     uart.puts("\r\n");
   }

   bool has_power() { return true; }
   
   void on_incoming_sms(const char *oa,const char *message)
   {
     uart.printf("SMS from %s : %s\r\n",oa,message);
   }

   virtual void on_response(const char *buf)
   {
      if (buf) {
        uart.puts(buf);
        uart.puts("\r\n");
      }
      SIM800::on_response(buf);
   }

   virtual const char * send_command_P(const char* s)
   {
      const char *t=s;
      char c;
      while (t) {
         c=pgm_read_byte(t);
         if (c && c!=';') {
            uart.put_char(c);
         }
         else {
           uart.puts("\r\n");
           break;
         }
         t++;
      }
      return SIM800::send_command_P(s);
   }

} modem;

void adc_start()
{
  digitalWrite(DIVG, LOW);
  adc_complete=0;
  nsamples=18;
  ADMUX=0xc6;  // start from unused channel 6
  ADCSRA=0xdc; // 16x clock prescaler  
}

void sample()
{
  sample_array[i] = digitalRead(PIR);
  i++;
  if(i > 8){
    i = 0;
  }
}

int full_arr(int arr[]){
  for(int j = 0;j<8;j++){
    if(arr[j] == 0){
      return 0;
    }
  }
  return 1;
}

int readPIR(int pirState)
{
  pirStatePrev = pirStateCurrent;
  pirStateCurrent = pirState;

  return pirStateCurrent && !(pirStatePrev);
}

volatile uint32_t secondsCounter = 0;

// Combined timer interrupt for both modem and PIR sensor
ISR(TIMER1_OVF_vect)
{
  // Modem runs at 9600Hz (8x 1200bps)
  TCNT1=0xffff-1600;
  modem.run();
  uart.run();
  bitcount++;
  if (bitcount==9600) {
    modem_tick=1;
    bitcount=0;   

    if (startCount) {
      // Count seconds (9600Hz / 9600 = 1Hz)
      secondsCounter++;
      if(secondsCounter % 1 == 0) { // Every second
        syncedTime.second++;
        if(syncedTime.second >= 60) {
          syncedTime.second = 0;
          syncedTime.minute++;
          if(syncedTime.minute >= 60) {
            syncedTime.minute = 0;
            syncedTime.hour++;
            if(syncedTime.hour >= 24) {
              syncedTime.hour = 0;
              syncedTime.day++;
              // Simple month handling (no leap years)
              uint8_t daysInMonth = 31;
              if(syncedTime.month == 4 || syncedTime.month == 6 || syncedTime.month == 9 || syncedTime.month == 11) {
                daysInMonth = 30;
              } else if(syncedTime.month == 2) {
                daysInMonth = 28;
              }
              
              if(syncedTime.day > daysInMonth) {
                syncedTime.day = 1;
                syncedTime.month++;
                if(syncedTime.month > 12) {
                  syncedTime.month = 1;
                  syncedTime.year++;
                }
              }
            }
          }
        }
      }
    }
  }
  
  // PIR sensor tick
  tick++;
}

ISR(WDT_vect) 
{
}

ISR(ADC_vect)
{
  uint8_t ch=ADMUX&15;
  adc[ch]=ADC;
  ch+=1;
  if (ch>8)
    ch=0;
  nsamples--;
  if (nsamples) {
    ADMUX=0xc0|(ch&15);
    ADCSRA=0xdc; // 16x clock prescaler  
  }
  else {
    ADCSRA=0x80;
    adc_complete=1;
  }
}

void setup() {
  MCUSR = 0;
  wdt_enable(WDTO_4S);
  
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(PIR, INPUT);
  pinMode(MODEM_TX,OUTPUT);
  pinMode(MODEM_RX,INPUT);
  pinMode(MODEM_RESET,OUTPUT);
  pinMode(DIVG,OUTPUT);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,0);

  // configure hardware UART for 9600 bps
  UBRR0H=((F_CPU/(16UL*9600))-1)>>8;
  UBRR0L=((F_CPU/(16UL*9600))-1)&0xff;
  UCSR0B=(1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); // enable rx,tx, enable rx int
  
  // set up timer interrupts for 9600 hz 
  TCCR1A=0x00;
  TCCR1B=0x01;
  TCNT1=0xffff-1600;
  TIMSK1=1;

  uart.printf("Initializing system\r\n");
  modem.reset();
  uart.printf("Ready\r\n");

  //wdt_enable(WDTO_1S);
  //set_sleep_mode(SLEEP_MODE_IDLE);
}

SIM800::MODEMSTATE oldstate=SIM800::M_NONE;
uint8_t motion_detected = 0;
uint8_t last_pir_state = 0;

// Have a premade FIFO class in case needed for EEPROM
//FIFO<DateTime,60> mcache;
//syncedTime.dow=msg;
//mchache.push(syncedTime);

void queueMessage(uint8_t msg) {
  if (pendingMessages >= MAX_MESSAGES) {
    uart.printf("Message queue full!\r\n");
    return;
  }

  message_cache[messageHead] = syncedTime;
  message_cache[messageHead].dow = msg; // currently using unused 'dow' variable in DateTime for messages, should probably rename this

  messageHead = (messageHead + 1) % MAX_MESSAGES;
  pendingMessages++;
}

void processMessageQueue() {
  static bool sendingMessage = false;
  static uint32_t lastAttempt = 0;
  const uint32_t retryInterval = 30000; // 30 seconds between retries
  
  uint32_t now = millis();

  if (modem.get_state() == SIM800::M_ERROR && (lastAttempt == 0 || (now - lastAttempt) > retryInterval)) { // modem goes into error mode
    modem.reset();
    sendingMessage = false;
    lastAttempt = now;
    uart.printf("Modem in error mode, resetting\r\n");
  }

  if (pendingMessages <= 0) return; // no messages

  if (sendingMessage) {
    int8_t result = modem.get_sms_result();
    
    if (result == -1) return; // in progress
    
    if (result == 0) { // success
      sendingMessage = false;
      messageTail = (messageTail+1) % MAX_MESSAGES;
      pendingMessages--;
      lastAttempt = 0;
      uart.printf("Message sent from queue\r\n");
    }

    if (result > 0) { // error
      sendingMessage = false;
      lastAttempt = now;
      uart.printf("Failed to send message from queue\r\n");
    }

  } else {
      // Only try sending if modem is ready and either:
      // - First attempt, or
      // - Retry interval has passed
      if (modem.get_state() == SIM800::M_READY && (lastAttempt == 0 || (now - lastAttempt) > retryInterval)) {
        
        DateTime current = message_cache[messageTail];
        char msg[200];

        switch (current.dow) {
          case 10: // Motion detected message
            createMessage(msg, "Motion detected!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case 20: // Battery critical message
            createMessage(msg, "Battery critically low!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case 21: // Battery low message
            createMessage(msg, "Battery getting low!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          default:
            uart.printf("ERROR: UNKNOWN MESSAGE TYPE IN QUEUE!\r\n");
            break;
        }


    }
  }
}

void createMessage(char* outStr, const char* text) {
    // format battery voltage (1 decimal place)
    char batStr[6];
    float bat_voltage = adc[0] * 1.1 / 1023 * ((22 + 7.5) / 7.5);
    dtostrf(bat_voltage, 4, 1, batStr);

    // format USB voltage (1 decimal place)
    char usbStr[6];
    float usb_voltage = adc[1] * 1.1 / 1023 * ((22 + 5.1) / 5.1);
    dtostrf(usb_voltage, 4, 1, usbStr);

    // format timestamp
    char timeStr[20];
    snprintf_P(timeStr, sizeof(timeStr), PSTR("%02d/%02d/%02d %02d:%02d:%02d"),
              syncedTime.year,
              syncedTime.month,
              syncedTime.day,
              syncedTime.hour,
              syncedTime.minute,
              syncedTime.second);

    // Compose final message
    snprintf_P(outStr, 160, PSTR("%s\n%s\nBAT =%sV USB =%sV"),
               timeStr,
               text,
               batStr,
               usbStr);
}

void loop() {

  //const uint32_t armingInterval = 10000; //ms
  //const uint32_t rearmingInterval = 60000; //ms
  //uint32_t now = millis();
  
  //sleep_mode();
  wdt_reset(); // watchdog timer
  WDTCSR|=(1<<WDIE);
  
  switch_state = digitalRead(SWITCH);
  digitalWrite(LED,switch_state);

  unsigned long currMillis = millis();

  // Handle PIR arming delay when switch is turned on
  if (switch_state && !pirArmed) {
      if (switchOnTime == 0) {
          switchOnTime = currMillis;
          uart.printf("Switch turned on, waiting %d ms to arm PIR\r\n", SWITCH_DELAY);
      } else if (currMillis - switchOnTime >= SWITCH_DELAY) {
          pirArmed = true;
          uart.printf("PIR sensor now armed\r\n");
      }
  } else if (!switch_state) {
      pirArmed = false;
      motion_detected = 0;
      switchOnTime = 0;
      lastMotionTime = 0;
  }

  //unsigned long currMillis = millis();
  if(currMillis - prevMillis >= INTER_DELAY) {
    prevMillis = currMillis;
    sample();
  }

  modem.get_response();
  if (modem_tick) {
    modem_tick=0;
    modem.pulse();
    adc_start();
  }
  
  if (oldstate!=modem.get_state()) {
    oldstate=modem.get_state();
    uart.printf("Modem state: %d\r\n",oldstate);
  }

  if (adc_complete) {
    digitalWrite(DIVG, HIGH); // disconnect voltage divider

    // now is time to notify about battery low
    float bat_voltage = adc[0]*1.1/1023*((22+7.5)/7.5);
    float usb_voltage = adc[1]*1.1/1023*((22+5.1)/5.1);

    if (usb_voltage > 4.0) { // if device is charging then don't need to send battery low messages
      if (bat_voltage > 3.45 && criticalMsgSent) criticalMsgSent = false; // reset flag if device has been charged
      if (bat_voltage > 3.55 && lowMsgSent) lowMsgSent = false;
    } else {
      if (bat_voltage < 3.45 && !criticalMsgSent) { // device stops working around 3.3 to 3.4 V
        queueMessage(20);
        criticalMsgSent = true;
      } else if (bat_voltage < 3.55 && !lowMsgSent) {
        queueMessage(21);
        lowMsgSent = true;
      }
    }
  }
  
  if (switch_state && pirArmed && adc_complete) {
    
    pir_state = readPIR(full_arr(sample_array));

    // Check if we're in the rearm period
      if (lastMotionTime > 0 && (currMillis - lastMotionTime) < PIR_DELAY) {
        // Still in rearm period, ignore motion
        pir_state = 0;
        motion_detected = 0;
      }
    
    // Check for motion start (rising edge)
    if (pir_state && !motion_detected) {
      motion_detected = 1;
      queueMessage(10);
      lastMotionTime = currMillis;
      uart.printf("Motion detected, will rearm in %d ms\r\n", PIR_DELAY);
    }
    
    // Check for motion end (falling edge)
    if (last_pir_state && !pir_state) {
      motion_detected = 0;
      uart.printf("Motion ended. Ready for new detection after rearm period.\r\n");
    }
    
    last_pir_state = pir_state;
  }

  if (pendingMessages > 0) {
    processMessageQueue();
  }
}