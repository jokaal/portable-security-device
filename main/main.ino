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

volatile int16_t adc[9];
volatile uint8_t adc_complete;
volatile uint8_t tick;
volatile uint8_t pir_state;
volatile uint8_t nsamples;
uint8_t ltick;

void adc_start()
{
  digitalWrite(DIVG, LOW);
  adc_complete=0;
  nsamples=18;
  ADMUX=0xc6;  // start from unused channel 6
  ADCSRA=0xdc; // 16x clock prescaler  
}

void setup() {

  MCUSR = 0;
  wdt_enable(WDTO_4S );
  
  Serial.begin(9600);
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(PIR, INPUT);
  pinMode(MODEM_TX,INPUT);
  pinMode(MODEM_TX,INPUT);
  pinMode(DIVG,OUTPUT);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,0);

  TCCR1A=0x00;
  TCCR1B=0x04; // prescaler to 256
  TIMSK1=1;  // enable overflow interrupts
  
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

ISR(TIMER1_OVF_vect)
{
  tick++;
}


void loop() {
    wdt_enable(WDTO_1S);
    WDTCSR|=(1<<WDIE);
    set_sleep_mode(SLEEP_MODE_IDLE); // SLEEP_MODE_IDLE);  
    sleep_mode();
    wdt_reset();
    if (tick!=ltick) {
      ltick=tick;
      if (!adc_complete)
        adc_start();
    }
    if (adc_complete) {

      digitalWrite(DIVG, HIGH);
      Serial.print("Battery:");
      Serial.print(adc[0]*1.1/1023*((22+7.5)/7.5));
      Serial.print(" ");

      Serial.print("Supply:");
      Serial.print(adc[1]*1.1/1023*((22+5.1)/5.1));
      Serial.print(" ");
  
      pir_state=digitalRead(PIR);
      Serial.print(" PIR:");
      Serial.print(pir_state);
      digitalWrite(LED,pir_state);

      Serial.print(" Switch:");
      Serial.println(digitalRead(SWITCH));
      adc_complete=0;
    }
    Serial.flush();
}