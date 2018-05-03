/*Created 20 Nov 2017
 * Author: Achim Merath
 * last modified 2018 03 11
 * circuit Arduino micro plus
 * with 
 * HX711 Sensors and weightcell 1kg up to xxxkg
 * Motordriver 
 * LDR 
 * 
 * measure the weight of a plant and drives a waterpump if weight is under MIN until MAX is reached
 * to programm use the SW1 Led blinks and pump is running until user releases the SW1
 * Min and Max are saved in the Eeprom
 * to program the pflamiku you need at first 
 * a plant in a potery which should be watered
 * put this plant in place
 * switch on the SW1 
 * power on the device
 * wait until the LED is blinking and then wait until enough water is pumped
 * then put the SW1 in OFF position 
 * thats it your pflamiku is now programmed
 * It may be that this Steps must be repeated if the Plant is growing
 * in case the plant is growing slowly the cycle to do this steps may be half a year
 * in case of fast growing this time may be shorter
 * during the night a ldr detects darkness and so the motor is disabled
 * 
 * have fun with your pflamiku
 * 
 * 
 * 
 * ------------------ TODO ------------------
 * clean up the code
 * 
 * ------------------------------------------
 * Micro, Leonardo, other 32u4-based boards : irq is on pin 0, 1, 2, 3, 7
 * 
 * ---------------Licence
 * GNU GPL 3.0
 * 
 */

 
/*
 * Options like debuging and serial out
 */

//#define ldr   //during the night a ldr detects darkness and so the motor is disabled
#define debuging 0 // a lot of messages over serial
#define measure 1   // only meassurement values over serial


#define delay_beginn_LowPower 20000 // this is for Programming 
#define delay_run 2000// 1000 my be good - this is for pause

//#include "LowPower.h"
#if defined ldr
#define ldr_PIN A6
int ldr_val=0;
int ldr_val_grenze=150;
#endif
#if defined measure
#define INIT_SERIAL(x) Serial.begin(x)
#endif

#if defined tempsensor
#include <LM75A.h>

LM75A lm75a_sensor/*(false, //A0 LM75A pin state
                   false, //A1 LM75A pin state
                   false, //A2 LM75A pin state)*/; // Create I2C LM75A instance
volatile float temperature_in_degrees;
volatile float Mittelwert_Temp=0;
volatile float TempBuff[16];
#endif

#include <HX711.h>
HX711 scale0;

//Sensor max min
const long w_min = 100;
const long w_max = 8388000;
//                 8388607 signed 24 bit as integer
//                16777216

// New Version with waightcell
// same Board --- keep smiling ---
// HX711 beginn
const int SCK_PIN = A1; // sck  -> weisbraun
const int DIN_PIN = A0; // dt  -> braun
//const int Power_PIN = A10; // VCC 
                          // GND 
                          // HX711 <1,7mA end
HX711 scale1;
#define SCK_PIN1  2 // sck
#define DIN_PIN1  3 // dt
HX711 scale2;
#define SCK_PIN2  4 // sck
#define DIN_PIN2  5 // dt


const byte sw1 = 7; //is now a switch
const byte SW2 = 6; // is used fore LED extern OFF
const byte led_ep = 8;
int RXLED = 17;  // The RX LED has a defined Arduino pin


//werte im eeprom
#include <EEPROM.h>
// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;
byte hi;
byte lo;
int var=0;
int x = 0;
//unsigned int time_run;
//unsigned int vol_time;

//Measurement value
signed long Messwert_min=0;
signed long Messwert=0;
signed long Messwert0=0;
signed long Messwert1=0;
signed long Messwert2=0;
//measurement faults
#define FBreite 4 
signed long MesswertF0[FBreite];
signed long MesswertF1[FBreite];
signed long MesswertF2[FBreite];
signed long mittel_tmp;
volatile signed long Messwert_long_time_average=0;
volatile boolean flag_first_run_messwert=0;
volatile int MLTAnr=36; // how many values goes into Messwert_long_time_average


// used for converting long <> byte 
//volatile byte buffer[4];
volatile byte Eeprom_buf[4];
volatile byte Eeprom_buf_max[4];
volatile long eeprom_value;
volatile long eeprom_value_max;

union v
{
    long         n_l;
    byte Eeprom_buf[4];
};

volatile union v val;
volatile long n_l;

union vm
{
    long         n_lm;
    byte Eeprom_buf_max[4];
};

volatile union vm valm;
volatile long n_lm;



/*
 * MAX14870 H-Bridge driving DC motor on Arduino
*/

int PWM = 9; // MCU PWM  MAX14870 Board
int DIR = A2;  // MCU MAX14870 Board
int ENM = A3;  // MCU MAX14870 Board


// this is for interrupthandling
//const byte interruptPin = 7;

volatile boolean state;
volatile boolean sw_flag=false;


// Interrupt Service Routine (ISR)
void sw_isr ()
{
  sw_flag = true;
 
}  // end of isr

                          
//FSM :::
volatile int sig = 0;
#define RESET_SIG 0          

#define ENTRY_SIG         10
#define Messwert_SIG      20
#define Analyse_SIG       30
#define EepromRead_SIG    40                      
#define EepromWrite_SIG   41
#define MotorOn_SIG       60
#define MotorOff_SIG      61

#define IRQ_SIG           90

#define DONE_SIG         254
#define ERROR_SIG        255


//*********************** SETUP ********************************
void setup() {
  // put your setup code here, to run once:
#if defined measure
  INIT_SERIAL(9600);
#endif
#if defined ldr
  pinMode(ldr_PIN, INPUT);
#endif
  // put your setup code here, to run once:
  // pinMode(SCK_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(DIN_PIN, INPUT);
  pinMode(SCK_PIN1, OUTPUT);
  pinMode(DIN_PIN1, INPUT);
  pinMode(SCK_PIN2, OUTPUT);
  pinMode(DIN_PIN2, INPUT);
  //pinMode(Power_PIN, OUTPUT);
  //digitalWrite(Power_PIN, HIGH);
  delay(100);
  
  //Serial.println("setup 1: \t\t");
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
 // TX LED is set as an output behind the scenes

  digitalWrite(RXLED, HIGH);    // set the LED off
  TXLED0;
  
  //scale0.begin(DIN_PIN, SCK_PIN);
  scale0.begin(A0, A1);
  scale0.power_up();
  delay(500);
  scale0.set_scale();
  delay(500);
  scale0.tare();
  //Serial.print("read average: \t\t");
  //Serial.println(scale0.read_average(20));   // print the average of 20 readings from the ADC
  scale0.set_scale(1910.f);
  //scale0.tare();  
  
  scale1.begin(DIN_PIN1, SCK_PIN1);
  scale1.power_up();
  delay(500);
  scale1.set_scale();
  delay(500);
  scale1.tare();
  //Serial.print("read average: \t\t");
  //Serial.println(scale1.read_average(20));   // print the average of 20 readings from the ADC
  scale1.set_scale(1910.f);
  
  scale2.begin(DIN_PIN2, SCK_PIN2);
  scale2.power_up();
  delay(500);
  scale2.set_scale();
  delay(500);
  scale2.tare();
  //Serial.print("read average: \t\t");
  //Serial.println(scale2.read_average(20));   // print the average of 20 readings from the ADC
  scale2.set_scale(1910.f);
  
  pinMode(PWM, OUTPUT); //
  pinMode(DIR, OUTPUT);
  pinMode(ENM, OUTPUT);
  
  pinMode(sw1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  // irq comes from switch pin 7
  attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, FALLING);
  pinMode(led_ep, OUTPUT);
  digitalWrite(led_ep, HIGH);

  
}
//*********************** FUNCTIONS ********************************
//There is only one channel
void DRIVE_STOP(void)
{
  analogWrite(PWM, 0);
  
  digitalWrite(DIR, LOW); // Turn the motor off
  digitalWrite(ENM, LOW); // Turn the motor off
  //digitalWrite(PWM, LOW); // Turn the motor off
 
}

void DRIVEONE(unsigned int time_vol)
{
#if defined debuging
    Serial.println("drive motor on: ");
#endif 
   // prepare for irq to indicate if switch is pulled to stop pump
   // detachInterrupt(sw1);
   // sw_flag = false;
   // attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, FALLING);
  
  // Run the motors 
  digitalWrite(ENM, LOW); // Turn HIGH motor A
  digitalWrite(DIR, LOW);
  // analogWrite(ENA, 200); // TO set the turning speed to 200 out of possible range 0 to 255
  for (x = 0; x < 256; x++)   // Motor will accelerate from 0 to maximum speed
  {
    analogWrite(PWM, x);
    //analogWrite(ENB, x);
    delay(1);
  }
  // ----------------------------

   
  
  // volume based motorcontroll 
  // should be calculated out of progsetting the max value <---------------<----TODO
  int dt=0; // max runtime motor 1200 sec becourse of 1 Liter max
  while (Messwert < eeprom_value_max && dt < 1200)
  {
     //Serial.print("dt is: \t\t");
     //Serial.println(dt);
#if defined debuging
    Serial.println("run motor and get measurement value: ");
#endif     
      var = Get_MeassurementValue();   
//     scale0.power_up();
//     
//     Messwert=scale0.read_average(10);
//     scale1.power_up();
//     Messwert+=scale1.read_average(10);
//     scale2.power_up();
//     Messwert+=scale2.read_average(10);
//     Messwert=Messwert/3;
//     Serial.print("read is: \t\t");
//     Serial.println(Messwert); 
//     Serial.print(" max is: \t\t");
//     Serial.println(eeprom_value_max);       // print the average of 20 readings from the ADC
     //delay(800);
     if(sw_flag==true){
      //Serial.println("sw1: \t\t");
#if defined debuging
    Serial.println("sw1 pressed: ");
#endif 
      sw_flag = false;
      dt=6500;
     }
     dt++;
  }
  

  // Changing the direction of the motor
//  for (x = 255; x >= 0; --x)  // Motor will decelerate from maximum speed to 0
//  {
//    analogWrite(PWM, x);
//    //analogWrite(ENB, x);
//    delay(1);
//  }
//  // if NOT called from new VolumeSet in IRQ_SIG Handler
//  if (time_vol > 0)
//  {
//    DRIVE_STOP();
//  }
//  if(dt >= 6500){
//  // prepare for irq to indicate if switch is pulled back to NULL
//   detachInterrupt(sw1);
//   sw_flag = false;
//   attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, RISING);
//    sig=IRQ_SIG;
//    while(!sw_flag){
//      if (state == LOW)
//        {
//          digitalWrite(led_ep, state);
//          delay(100);
//        }
//        else
//        {
//          digitalWrite(led_ep, state);
//          delay(100);
//        }
//        state = !state;
//    }
//    delay(10000);
//   }
//   // prepare for irq 
//   detachInterrupt(sw1);
//   sw_flag = false;
//   attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, FALLING);
//   sw_flag = false;
  Messwert_long_time_average=Messwert;
}
void read_eeprom(void){


      // min weight
      Eeprom_buf[0] = EEPROM.read(10);
      Eeprom_buf[1] = EEPROM.read(11);
      Eeprom_buf[2] = EEPROM.read(12);
      Eeprom_buf[3] = EEPROM.read(13);
      //max weight
      Eeprom_buf_max[0] = EEPROM.read(14);
      Eeprom_buf_max[1] = EEPROM.read(15);
      Eeprom_buf_max[2] = EEPROM.read(16);
      Eeprom_buf_max[3] = EEPROM.read(17);
      
      //ergebnis=val.n_l;
      eeprom_value =0;
      eeprom_value_max =0;
      // this works !!!
      eeprom_value = *(unsigned long *)Eeprom_buf;
      // NOT// eeprom_value = (unsigned long)(Eeprom_buf[3] << 24) | (Eeprom_buf[2] << 16) | (Eeprom_buf[1] << 8) | Eeprom_buf[0];
      
      // this works !!!
      eeprom_value_max = *(unsigned long *)Eeprom_buf_max; 
      // NOT // eeprom_value_max = (unsigned long)(Eeprom_buf_max[3] << 24) | (Eeprom_buf_max[2] << 16) | (Eeprom_buf_max[1] << 8) | Eeprom_buf_max[0];
      
      //Serial.print("\t|F Messwert Eeprom_buf:\t");
      //Serial.print(Eeprom_buf[3], 1);//Serial.print(" \t");
      //Serial.print(Eeprom_buf[2], 1);//Serial.print(" \t");
      //Serial.print(Eeprom_buf[1], 1);//Serial.print(" \t");
      //Serial.println(Eeprom_buf[0], 1);
      //Serial.print("\t|F Messwert Eeprom:\t");
      //eeprom_value=val.n_l;
      //Serial.println(eeprom_value, 1);
     
      //Serial.print("\t|F Messwert Eeprom_buf_max:\t");
      //Serial.print(Eeprom_buf_max[3], 1);//Serial.print(" \t");
      //Serial.print(Eeprom_buf_max[2], 1);//Serial.print(" \t");
      //Serial.print(Eeprom_buf_max[1], 1);//Serial.print(" \t");
      //Serial.println(Eeprom_buf_max[0], 1);
      //Serial.print("\t|F Messwert Eeprom Max:\t");
      //eeprom_value_max=valm.n_lm;
      //Serial.println(eeprom_value_max, 1);

}


void write_eeprom(void)
{
      EEPROM.write(10,val.Eeprom_buf[0]);
      EEPROM.write(11,val.Eeprom_buf[1]);
      EEPROM.write(12,val.Eeprom_buf[2]);
      EEPROM.write(13,val.Eeprom_buf[3]);
      //Serial.print("\t| write Eeprom buf:\t");
      //Serial.print(val.Eeprom_buf[3], 1);//Serial.print(" \t");
      //Serial.print(val.Eeprom_buf[2], 1);//Serial.print(" \t");
      //Serial.print(val.Eeprom_buf[1], 1);//Serial.print(" \t");
      //Serial.println(val.Eeprom_buf[0], 1);
}

void write_eeprom_max(void)
{
      EEPROM.write(14,valm.Eeprom_buf_max[0]);
      EEPROM.write(15,valm.Eeprom_buf_max[1]);
      EEPROM.write(16,valm.Eeprom_buf_max[2]);
      EEPROM.write(17,valm.Eeprom_buf_max[3]);
      //Serial.print("\t| write Eeprom bufm:\t");
      //Serial.print(valm.Eeprom_buf_max[3], 1);//Serial.print(" \t");
      //Serial.print(valm.Eeprom_buf_max[2], 1);//Serial.print(" \t");
      //Serial.print(valm.Eeprom_buf_max[1], 1);//Serial.print(" \t");
      //Serial.println(valm.Eeprom_buf_max[0], 1);
      read_eeprom();
}


int Get_MeassurementValue(void)
{
    uint8_t ret=0;
    uint8_t testflag=0;

    while(testflag==0){
       scale0.power_up();
       //delay(10);
       
       Messwert0=scale0.read_average(3);
       MesswertF0[2]=scale0.read_average(3);
       scale1.power_up();
       //delay(10);
       Messwert1=scale1.read_average(3);
       MesswertF1[2]=scale1.read_average(3);
       scale2.power_up();
       //delay(100);
       Messwert2=scale2.read_average(3);
       MesswertF2[2]=scale2.read_average(3);
       
       // newest value in
       MesswertF0[(0)]=MesswertF0[2]-(MesswertF0[2]/100*2); //min
       MesswertF0[(1)]=MesswertF0[2]+(MesswertF0[2]/100*2); //max
       
       MesswertF1[(0)]=MesswertF1[2]-(MesswertF1[2]/100*2); //min
       MesswertF1[(1)]=MesswertF1[2]+(MesswertF1[2]/100*2); //max
       
       MesswertF2[(0)]=MesswertF2[2]-(MesswertF2[2]/100*2); //min
       MesswertF2[(1)]=MesswertF2[2]+(MesswertF2[2]/100*2); //max
       
//       if (MesswertF0[4] != 0){
//       }
       if ( !(Messwert0 < MesswertF0[0] && Messwert0 > MesswertF0[1]) ){
        testflag++;
       
       }
       
       if ( !(Messwert1 < MesswertF1[0] && Messwert1 > MesswertF1[1]) ){
        testflag++;
       
       }
       if ( !(Messwert2 < MesswertF2[0] && Messwert2 > MesswertF2[1])){
        testflag++;
       
       }
       if(testflag < 3){
        testflag = 0;
       }
      // shift oldest value out
       
//         Serial.print(MesswertF0[0]);       // print the average of 20 readings from the ADC
//         Serial.print("\t");
//         Serial.print(MesswertF0[1]);       // print the average of 20 readings from the ADC
//         Serial.print("\t");
//         Serial.print(MesswertF0[2]);       // print the average of 20 readings from the ADC
//         Serial.print("\t");
//         Serial.print(Messwert0);       // print the average of 20 readings from the ADC
//         Serial.println("\t");
       // dedect if measurement fault has happend or if user does unwanted things with the plant
       // if Messwert < or > 3% ch 0 or 1 or 2 get a new measurement Value
       //signed long MesswertDif=0;
       
     
    }
#if defined measure
    Serial.print(eeprom_value, 1);
     Serial.print("\t"); 
     Serial.print(eeprom_value_max, 1);
     Serial.print("\t");
#endif     
     
     if((Messwert0 > 1000) && (Messwert1 > 1000) && (Messwert2 > 1000)){
#if defined measure      
       Serial.print(Messwert0);       // print the average of 20 readings from the ADC
       Serial.print("\t");
       
       
       //Messwert1=scale1.read_average(10);
       
       Serial.print(Messwert1);       // print the average of 20 readings from the ADC
       Serial.print("\t");
        
       //Messwert2=scale2.read_average(10);
       
       Serial.print(Messwert2);       // print the average of 20 readings from the ADC
       Serial.print("\t");
#endif       
       Messwert=Messwert0+Messwert1+Messwert2;
       Messwert=(Messwert/3);
      //Serial.print("Messwerte 1+2+3: \t");
#if defined measure
      Serial.print(Messwert);       // print the average of 20 readings from the ADC
      Serial.print("\t");
#endif  
     
  
  //pay attention ... test if value is out of range
      
      
      if (w_min < Messwert < w_max)
      {
        mittel_tmp = Messwert_long_time_average*MLTAnr;
        Messwert_long_time_average = (mittel_tmp-Messwert_long_time_average+Messwert)/MLTAnr;
        //Serial.print("\t| Messwert_long_time_average:\t");
#if defined measure
        Serial.print(Messwert_long_time_average, 1);
        Serial.print("\t");
#endif  
        
        
      }
      else
      {
        ret=1;
      }
/*my be tempsensor is now no more needed
*/      
#if defined tempsensor      
      temperature_in_degrees = lm75a_sensor.getTemperatureInDegrees();
  
      if (temperature_in_degrees == INVALID_LM75A_TEMPERATURE) {
        //Serial.println("Error while getting temperature");
#if defined measure 
          float errorvalue=22.11;
          Serial.print(errorvalue); 
          Serial.print("\t");        
          Serial.println(errorvalue);
#endif         
      } else {
        //Serial.print("Temperature: ");
#if defined measure        
        Serial.print(temperature_in_degrees);
        Serial.print("\t");
        
        //Serial.println(" degrees (");
        //Serial.print(LM75A::degreesToFahrenheit(temperature_in_degrees));
        //Serial.println(" fahrenheit)");
#endif
        if(Mittelwert_Temp != 0.0){
          for(int i=0;i<15;i++){
            TempBuff[i]=TempBuff[(i+1)];
          }
          TempBuff[15]=temperature_in_degrees;
          float mittel_temp_tmp = Mittelwert_Temp*MLTAnr;
          Mittelwert_Temp = (mittel_temp_tmp-Mittelwert_Temp + temperature_in_degrees)/MLTAnr;
         
        }
        else
        { 
          //this should only happen when first run
          //Serial.println("Temperature first run: ");
          Mittelwert_Temp = temperature_in_degrees;
        }
#if defined measure          
          Serial.println(Mittelwert_Temp);
#endif         
      }
#else
     Serial.println("\t"); 
#endif
    }
//    scale0.power_down();
//    scale1.power_down();
//    scale2.power_down();
    return ret;
}
/*
 * Funktion reads in measurement value first without watering and then waits for SW goes HIGH
 * If HIGH then read in measurement value and write it to eeprom
 * This funktion is called when in IRQ_SIG SW1_IRQ is dedected
 * @param returns 0 or 1
 * 1 = timeout
 * 0 = ok sw = high
 */
int waitSW_and_blink(void)
{
    uint32_t tcounter=0;
    //first read value then save it to Messwert_long_time_average 
    //this before customer begins to water the plant
    // so we can besure that this is the lowest level
    var = Get_MeassurementValue();
#if defined debuging
    Serial.println("waitsw_and_blink: ");
#endif    
    Messwert_min=Messwert;
#if defined debuging    
    Serial.print("Messwert min: ");Serial.println(Messwert_min);
#endif    
    //switch IRQ to rising edge to dedect when SW1 goes HIGH
    detachInterrupt(sw1);
    sw_flag = false;
    attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, RISING);
    
    //while (digitalRead(sw1) == LOW || tcounter == 1000 )
    while (!sw_flag || tcounter == 1000 )
      {
        // led_ep gives user feedback
        if (state == LOW)
        {
          if(digitalRead(SW2)==HIGH) {digitalWrite(led_ep, state);}
          delay(delay_run);
          digitalWrite(RXLED, HIGH);   // set the LED off if on
          TXLED0; //TX LED is not tied to a normally controlled pin
        }
        else
        {
          if(digitalRead(SW2)==HIGH) {digitalWrite(led_ep, state);}
          delay(delay_run);
        }
        state = !state;
        tcounter++;
        if (tcounter == 1000)
        {
           digitalWrite(led_ep, LOW); 
           //switch IRQ to falling to dedect SW1 goes LOW (normal wait state of IRQ)
           detachInterrupt(sw1);
           sw_flag = false;
           attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, FALLING); 
           return 1;
        }
#if defined debuging
    Serial.println("waitsw_and_blink while schleife: ");
#endif 
        var = Get_MeassurementValue();
     } 
     digitalWrite(RXLED, HIGH);   // set the LED off if on
     TXLED0; //TX LED is not tied to a normally controlled pin
     
     digitalWrite(led_ep, LOW); // gives user feedback
     
     //switch IRQ to falling to dedect SW1 goes LOW (normal wait state of IRQ)
     detachInterrupt(sw1);
     sw_flag = false;
     attachInterrupt(digitalPinToInterrupt(sw1), sw_isr, FALLING);  
      
     digitalWrite(led_ep, LOW);
     return 0;
}

void wait_and_blink(void)
{
    uint32_t tcounter=0;
    state = LOW;
#if defined debuging
    Serial.println("wait_and_blink:\t");
#endif 
    digitalWrite(RXLED, HIGH);    // set the LED off
    //TXLED0;
    while (tcounter < 5 )
      {
        //Serial.print("state:\t");
        //Serial.println(state);
        if (state == LOW)
        {
          // using the onboard LEDs
          digitalWrite(RXLED, HIGH);   // set the LED off
           TXLED0; //TX LED is not tied to a normally controlled pin
          
          //digitalWrite(led_ep, HIGH);
          if(digitalRead(SW2)==HIGH) {digitalWrite(led_ep, HIGH);}
          delay(200);
        }
        else
        {
          //Serial.println("wait_and_blink else:\t");
          // using the onboard LEDs
          
           
          digitalWrite(led_ep, LOW);
          for ( uint32_t t=0; t<6; t++)
          {
            //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            //Serial.println("wait_and_blink for:\t");
            digitalWrite(RXLED, HIGH);    // set the LED off
            TXLED0;
            delay(delay_run);
            if (sw_flag)
            {
                t=65000;
                tcounter=5000000;
            }
              
          }
          //delay(600000);//1h
        }
        state = !state;
        tcounter++;
        
     }
    digitalWrite(led_ep, LOW); 
}

//*********************** MAIN LOOP ********************************

void loop() {
  // put your main code here, to run repeatedly:
    switch (sig)
  { 
    case RESET_SIG:
      //Serial.println("RESET_SIG:\t");
      //reset all power down outputs
      DRIVE_STOP();
      //digitalWrite(Power_PIN, LOW);
#if defined debuging   
      Serial.println("RESET_SIG:\t");
#endif 
      delay(2000);//20000 this is for not entering  sleepmode so progmode is posible after reset
      digitalWrite(RXLED, HIGH);   // set the LED off if on
     TXLED0; //TX LED is not tied to a normally controlled pin
     //Serial.println("RESET_SIG:\t");
      //scale0.tare(); 
      
           
      sig=ENTRY_SIG;
      if (digitalRead(sw1) == LOW){
#if defined debuging   
      Serial.println("\t set sig=IRQ_SIG");
#endif 
        sig=IRQ_SIG;
      }
      break;

    case ENTRY_SIG:
#if defined debuging   
      Serial.println("ENTRY_SIG:\t");
#endif      
      
      // switch on measurement unit
      //digitalWrite(Power_PIN, HIGH);
      //delay(5000);
      digitalWrite(RXLED, HIGH);   // set the LED off if on
     TXLED0; //TX LED is not tied to a normally controlled pin //TXLED1; //TX LED macro to turn LED ON
      sig=Messwert_SIG;
      break;


    case Messwert_SIG:
#if defined debuging   
      Serial.println("Messwert_SIG:\t");
#endif      
      // test if first run
      // this is the case if power was down !!!!
      // for debug only - > must be in IRQ_SIG handler triggerd by switch
      
      if (flag_first_run_messwert==0)
      {
#if defined debuging        
        Serial.println("first run:\t");
#endif
         read_eeprom();
        var = Get_MeassurementValue();
        Messwert_long_time_average=Messwert;
        //read_eeprom();
     
      

        if(0<eeprom_value<4294960000)
        {
          // case NOT write Value to Eeprom and then next stage is IRQ_SIG
          val.n_l=Messwert_long_time_average;
          
          sig=IRQ_SIG;
        }
        else
        {
          //val.n_l=Messwert_long_time_average;
          val.n_l=Messwert;
          write_eeprom();
          //sig=EepromWrite_SIG; // ???? TODO
        }
        flag_first_run_messwert=1;
      }
      else
      {
           var = Get_MeassurementValue();
          //Serial.print("\t|Messwert_SIG: Messwert_long_time_average:\t");
          //Serial.println(Messwert_long_time_average, 1);
          val.n_l=Messwert_long_time_average;
          if (var == 1)
          {
            sig=ERROR_SIG;
          }
          else
          {
            sig=EepromRead_SIG;
          }
      }
      
 
      //-------------------
      //sig=EepromRead_SIG;
      break;
 

    case Analyse_SIG:
#if defined debuging    
      Serial.println("Analyse:\t");
#endif
      if (Messwert_long_time_average < eeprom_value && digitalRead(sw1) == HIGH){
        sig=MotorOn_SIG;
        }
       else{
         sig=IRQ_SIG;
        }
      
      
     
      break;
 
    case EepromWrite_SIG:
#if defined debuging    
      Serial.println("Eeprom write:\t");
#endif      
      write_eeprom();
      write_eeprom_max();
      sig=EepromRead_SIG;
      break;

    case EepromRead_SIG:
#if defined debuging    
      Serial.println("EepromRead_SIG:\t");
#endif      
      read_eeprom();

      
      sig=Analyse_SIG;
      break;
      
    case MotorOn_SIG:
#if defined debuging    
      Serial.println("Motor on:\t");
#endif      
      //sig=DONE_SIG;
      sig=MotorOff_SIG;
      Messwert_long_time_average=eeprom_value_max;
#if defined ldr   
      #if defined debuging    
        Serial.print("ldr value:\t");
      #endif      
     ldr_val = analogRead(ldr_PIN);//ldr_PIN
     #if defined debuging 
     Serial.println(ldr_val);
     #endif 
     if (ldr_val < ldr_val_grenze){ 
      DRIVEONE(1);
     }
#else
      DRIVEONE(1);
#endif
      break;
      
    case MotorOff_SIG:
#if defined debuging    
      Serial.println("MotorOff_SIG:\t");
#endif      
      DRIVE_STOP();
      sig=DONE_SIG;
      break;

    case IRQ_SIG:
#if defined debuging    
      Serial.println("IRQ_SIG:\t");
#endif      
      
      //new VolumeSet
      // 2 stages  lower and upper 
      // write lower value to eeprom
      if (digitalRead(sw1)==LOW){ // prog
        var = Get_MeassurementValue();
#if defined debuging        
        Serial.print("IRQ_SIG: Switch start with Messwert\t");
        Serial.println(Messwert_min);
#endif        
        Messwert_min=Messwert;
        sw_flag=1;
        }
      int rret;
      // ask if user will program new weight
      if (sw_flag)
      {
#if defined debuging        
        Serial.println("IRQ_SIG: Switch\t");
#endif        
        // blink and wait for switch goes low
        rret=waitSW_and_blink();
  
        // if 0 then SW else rret = 1 = timeout
        if(rret == 0)
        {
          val.n_l=Messwert_min;
          write_eeprom();
#if defined debuging          
          Serial.print(" Messwert_long_time_average: \t\t");
          Serial.println(Messwert);
#endif          
          valm.n_lm = Messwert;   //scale0.read_average(20); //--
#if defined debuging          
          Serial.print("valm.n_lm: \t\t");
          Serial.println(valm.n_lm);
#endif          
          write_eeprom_max();
          // write upper value to eeprom
        }
        
      }
      read_eeprom();
      wait_and_blink();
      sig=DONE_SIG;
      break;
      
    case DONE_SIG:
#if defined debuging    
      Serial.println("DONE:\t");
#endif      
      sig=ENTRY_SIG;
      break;
            
   case ERROR_SIG:
      DRIVE_STOP();
      //digitalWrite(Power_PIN, LOW);
      
      while(1){
        //Serial.println("ERROR_SIG:\t");
        delay(500);
        if (state == LOW)
        {
          digitalWrite(led_ep, state);
          delay(500);
        }
        else
        {
          digitalWrite(led_ep, state);
          delay(500);
        }
        state = !state;
        }
      sig=ERROR_SIG;
      break;
    
    default:
      sig=RESET_SIG;
  }   
}

