#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avrserial.hpp>
#include <sim800.hpp>

#define COUNTOF(x) (sizeof(x)/sizeof(x[0]))

#define PHONE_NUMBER ""
#define SIM_PIN "0000"

#define MAX_MESSAGES 50 // in cache

enum { MOTION_DETECTED = 10, BATTERY_CRITICAL = 20, BATTERY_LOW = 21, BATTERY_NOT_CRITICAL = 22, BATTERY_CHARGING = 23 };

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

#define INTER_DELAY 50/10
#define SWITCH_DELAY 10000/10 // delay when switch is turned on in 10 ms ticks
#define PIR_DELAY 60000/10 // delay between each notification in 10 ms ticks
#define ADC_DELAY 100/10 // ADC sampling interval in 10 ms ticks
#define SECOND_DELAY 1000/10

// these counters are counted down in timer interrupt
// with 10 ms interval
uint16_t pir_timer = INTER_DELAY;
uint16_t switch_timer = SWITCH_DELAY;
uint16_t notification_timer = PIR_DELAY;
uint16_t adc_timer = ADC_DELAY;
uint16_t second_timer = SECOND_DELAY;

SIM800::MODEMSTATE oldstate=SIM800::M_NONE;

uint8_t motion_detected;
uint8_t last_pir_state;

int sample_array[8];
int pirStateCurrent;
int pirStatePrev;

volatile int16_t adc[9];
volatile uint8_t adc_complete;
volatile uint8_t pir_state;
volatile uint8_t switch_state;
volatile uint8_t nsamples;

bool lowMsgSent = false; // battery low
bool criticalMsgSent = false; // battery critical
bool pirArmed = false;
uint16_t bitcount;
DateTime syncedTime;
bool startCount = false;

// message FIFO
DateTime message_cache[MAX_MESSAGES];
uint8_t messageHead;
uint8_t messageTail;
uint8_t pendingMessages;

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

void set_timer(uint16_t& timer,uint16_t ms)
{
  timer=ms;
}

bool expired(uint16_t timer)
{
  return !timer;
}

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
static uint8_t i=0;
  sample_array[i] = digitalRead(PIR);
  i++;
  if(i >= COUNTOF(sample_array)){
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

// Combined timer interrupt for both modem and PIR sensor
ISR(TIMER1_OVF_vect)
{
  // Modem runs at 9600Hz (8x 1200bps)
  TCNT1=0xffff-1600;
  modem.run();
  uart.run();
  bitcount++;
  if (bitcount==96) {
    bitcount=0;   
    if (pir_timer)
      pir_timer--;
    if (switch_timer)
      switch_timer--;
    if (notification_timer)
      notification_timer--;
    if (adc_timer)
      adc_timer--;
    if (second_timer)
      second_timer--;
  }
}

ISR(WDT_vect) 
{
}

ISR(ADC_vect)
{
  uint8_t ch=ADMUX&15;
  adc[ch]=ADC; // (adc[ch]+adc[ch]+adc[ch]+ADC)>>2; // 0.75*old+0.25*new
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

void queueMessage(uint8_t msg) {
  if (pendingMessages >= MAX_MESSAGES) {
    uart.printf("Message queue full!\r\n");
    return;
  }
  uart.printf("Adding message %d to queue\r\n",(int16_t)msg);
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
          case MOTION_DETECTED: // Motion detected message
            createMessage(msg, "Motion detected!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case BATTERY_CRITICAL: // Battery critical message
            createMessage(msg, "Battery critically low!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case BATTERY_LOW: // Battery low message
            createMessage(msg, "Battery getting low!");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case BATTERY_CHARGING:
            createMessage(msg, "Battery is charging");
            modem.send_sms(PHONE_NUMBER, msg);
            sendingMessage = true;
            break;
          case BATTERY_NOT_CRITICAL:
            createMessage(msg, "Battery no longer critically low");
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

void check_battery()
{
  if (adc_complete) {
    digitalWrite(DIVG, HIGH); // disconnect voltage divider

    // now is time to notify about battery low
    float bat_voltage = adc[0]*1.1/1023*((22+7.5)/7.5);
    float usb_voltage = adc[1]*1.1/1023*((22+5.1)/5.1);

    uart.printf("%d:%d:%d bat: %d usb: %d low: %d critical: %d pending messages: %d\r\n",syncedTime.hour,syncedTime.minute,syncedTime.second,
      (int)(bat_voltage*1000),(int)(usb_voltage*1000),lowMsgSent?1:0,criticalMsgSent?1:0,pendingMessages); // debug

    if (usb_voltage > 4.0) { // if device is charging then don't need to send battery low messages
      if (bat_voltage > 3.65) {
        if (lowMsgSent) {
          lowMsgSent = false;
          criticalMsgSent = false; // reset flag if device has been charged
          queueMessage(BATTERY_CHARGING);
        }
      }
      if (bat_voltage > 3.55) {
        if (criticalMsgSent) {
          criticalMsgSent = false; // reset flag if device has been charged
          queueMessage(BATTERY_NOT_CRITICAL);
        }
      }
    } 
    else {
      if (bat_voltage < 3.45) {
        if (!criticalMsgSent) { // device stops working around 3.3 to 3.4 V
          queueMessage(BATTERY_CRITICAL);
          criticalMsgSent = true;
          lowMsgSent = true;
        }
      }
      if (bat_voltage < 3.55) {
        if (!lowMsgSent) {
          queueMessage(BATTERY_LOW);
          lowMsgSent = true;
        }
      }
    }
  }
}

void count_time()
{
  if (startCount) {
    // Count seconds (9600Hz / 9600 = 1Hz)
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

void check_switch()
{
  static uint8_t state=0;
  switch_state = digitalRead(SWITCH);
  digitalWrite(LED,switch_state);

  switch (state) {
    default:
      state=0;
      // fall through
    case 0: // not armed, check for switch on
      if (switch_state) {
        set_timer(switch_timer,SWITCH_DELAY);
        state=1;
        uart.printf("Switch turned on, waiting %d ms to arm PIR\r\n",SWITCH_DELAY*10);
      }
      break;
    case 1:
      if (!switch_state) {
        state=0;
        break;
      }
      if (expired(switch_timer)) {
        pirArmed = true;
        set_timer(notification_timer,0);
        motion_detected = 0;
        uart.printf("PIR sensor now armed\r\n");
        state=2;
        break;
      }
      break;
    case 2:
      if (!switch_state) {
        pirArmed = false;
        motion_detected = 0;
        state = 0;
        uart.printf("PIR sensor disarmed\r\n");
      }
      break;
  }
}

void check_pir()
{
  sample();
  pir_state = readPIR(full_arr(sample_array));
  if (pirArmed) {

    if (!expired(notification_timer)) {
      pir_state = 0;
      motion_detected = 0;
    }

    if (pir_state && (!motion_detected)) {
      motion_detected = 1;
      queueMessage(MOTION_DETECTED);
      set_timer(notification_timer,PIR_DELAY);
      uart.printf("Motion detected, will rearm in %d seconds\r\n", PIR_DELAY/100);
    }

    // Check for motion end (falling edge)
    if (last_pir_state && !pir_state) {
      motion_detected = 0;
      uart.printf("Motion ended. Ready for new detection after rearm period.\r\n");
    }
    
  }
  else {
    motion_detected = 0;
  }
  last_pir_state = pir_state;
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

void loop() {

  //sleep_mode();
  wdt_reset(); // watchdog timer
  WDTCSR|=(1<<WDIE);

  if (expired(adc_timer)) {  
    set_timer(adc_timer,ADC_DELAY);
    adc_start();
    check_switch();
  }

  if (expired(pir_timer)) {
    set_timer(pir_timer,INTER_DELAY);
    check_pir();
  }

  modem.get_response();
  
  if (oldstate!=modem.get_state()) {
    oldstate=modem.get_state();
    uart.printf("Modem state: %d\r\n",oldstate);
  }

  if (expired(second_timer)) {
    set_timer(second_timer,SECOND_DELAY);
    check_battery();
    modem.pulse();
    if (pendingMessages) {
      processMessageQueue();
    }
  }
}
