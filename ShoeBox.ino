#include "DigitalInputHandler.h"
#include "AnalogInputHandler.h"
#include "DigitalOutputHandler.h"
#include "RFID_Handler.h"
#include "AnalogOutputHandler.h"
#include "General_Cfg.h"

#include <SPI.h>
#include <MFRC522.h>
#include "LedControl.h"
#include "Led_Matrix.h"
#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Wire.h>
#include <TimerOne.h>

#define ADC_CHANNEL 0

/* System init delay*/
int System_Init = 10;

/* Audio sampling */
int numSamples=0;

int16_t capture[FFT_N];
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer

const Digital_Input_Cfg Sb_Digital_Input_Cfg[No_Digital_Pins] = 
    {
      {18,1000}, // 18
      {19,1000}, // 19
      {20,1000}, // 20
      {25,1000}, // 25
    };
    
Digital_Input_St Sb_Digital_Input_St[No_Digital_Pins] = 
    {
      {0,0,0,0,0,5000},
      {0,0,0,0,0,5000},
      {0,0,0,0,0,5000},
      {0,0,0,0,0,5000},
    };
/* Analog input read */

Analog_Input_St Sb_Motion_Detection = {1,0,730,0,0} ;

/* Red led variable */
Red_Led_control Sb_Led_Cfg[No_Led_Pins] = 
  {
    {0,0,0}
  };
    
/* RDID reader and RFID led variable */    
 RFID_Test_Led_control Sb_RFID_Test_Led[1] = 
 {
  {0,0,Pin_RFID_Test_Led}
 };   
 
MFRC522 mfrc522(SS_PIN, RST_PIN); 
const int TAG_UID[4] = {246,101,82,73};

/* Matrix led variable */

LedControl lc=LedControl(LED_MX_DIN,LED_MX_CLK,LED_MX_CS,1); 

Led_Matrix_control Sb_Matrix_Led = {Output_ShoeBox_Move,0};

const byte Moving[] =
{
  B00001110,
  B00001110,
  B00001110,
  B00001110,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

const byte Lid_Open[] =
{
  B11101110,
  B11101110,
  B11101110,
  B00000000,
  B11101110,
  B11101110,
  B11101110,
  B00000000
};

const byte Led_Off[] =
{
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

const byte Clappig[] =
{
  B11101110,
  B11101110,
  B11101110,
  B00000000,
  B11101110,
  B11101110,
  B11101110,
  B00000000
};

 
/* Clapping detection variable*/
volatile int Clapping_Check = 0;
int Clapping_Sample_Counter = 0;
int Clapping_Delay_Counter = 0;
int Clapping_Sample_Recieve = 0;
int Clapping_Sample_Period = 0;
bool Clapping_Start_Check = 0;

bool Clapping_Start_Check_t = 0;

int lower[8] = {0,  8,  16, 24, 32, 40, 48, 56};
int upper[8] = {8, 16,  24, 32, 40, 48, 56, 64};

/* Led matrix operation variable*/
volatile int Led_Matrix_Dis = 0;
volatile int Led_Matrix_Delay = 0;
int Led_Matrix_Vertical[8] ={0};
uint8_t Led_Matrix_Horizontal[8] ={0};
int Led_Matrix_Index = 0;
int Led_Matrix_Counter = 0;
int Max_Amp;
int Resolution;

bool First_Time_Picking_up = 0;

static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 };

/* Audio sampling interrupt handling */
ISR(ADC_vect)
{
  static const int16_t noiseThreshold = 4;
  volatile int16_t     sample         = ADCL | (ADCH << 8); //0 -> 1028
  
  capture[numSamples] = 
      ((sample > (512-noiseThreshold)) &&
        (sample < (512+noiseThreshold))) ? 0 :
        sample - 512; // Sign-convert for FFT; -512 to +511
        
  if(++numSamples >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}

      
void setup() {

  uint8_t i, j, nBins, binNum, *data;
  
  Serial.begin(115200);
  /* Init Red led */
  pinMode(Pin_Red_Led, OUTPUT);
  
  /* Init RFID reader*/
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  mfrc522.PCD_DumpVersionToSerial();
  
  pinMode(Pin_RFID_Test_Led,OUTPUT);
  
  /*Init Led Matrix*/
  lc.shutdown(0,false);  // Wake up displays
  lc.setIntensity(0,5);  // Set intensity levels
  lc.clearDisplay(0);  // Clear Displays

  /* Flaming led*/
  pinMode(FLAME_RED_LED_1, OUTPUT);
  pinMode(FLAME_RED_LED_2, OUTPUT);
  pinMode(FLAME_YELOW_LED_1, OUTPUT);

  /* Free sampling ADC */
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= ADC_CHANNEL;   // set A0 analog input pin
  ADMUX &= B11011111;     // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX |= (1 << REFS0);  // set reference voltage
  ADCSRA |= (1 << ADPS0) |(1 << ADPS1) |(1 << ADPS2);  // 9.6 kHz
  //ADCSRA |= (1 << ADPS1) |(1 << ADPS2);  // 9.6 kHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
  ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off

 /* Timer1 Init */
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

  /* Tilt sensor init */
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), Tilt_sensor_hdl, RISING);
}


void Led_Matrix_Decide()
{
    if(Sb_Digital_Input_St[Reed_SW].Current_State == 0)
    {
      Sb_Matrix_Led.Mode = Output_Flicker;
      return;
    }
    if(Sb_Digital_Input_St[Lid_Detection].Current_State == 1)
    {
      Sb_Matrix_Led.Mode = Output_Lid_Open;
      return;
    }

    if(Sb_Motion_Detection.Current_State == 1)
    {
      Sb_Matrix_Led.Mode = Output_ShoeBox_Move;
      return;
    }
    Sb_Matrix_Led.Mode = Output_Off;
}

void Tilt_sensor_hdl() {
  Sb_Motion_Detection.Current_State = 1;
  Sb_Motion_Detection.Pin_State_Change = 1;
  //detachInterrupt(digitalPinToInterrupt(3));
  //Serial.println("Moving");
  Sb_Motion_Detection.Counter_delay = 0;
}

void loop() {
  
  int counter;
  uint8_t x,i, L;

  /* Read input state and apply deboucing input pins */
  for(counter =0; counter < No_Digital_Pins; counter++)
  {
      Sb_Digital_Input_St[counter].Reading_State = digitalRead(Sb_Digital_Input_Cfg[counter].Pin_Number);

       if(Sb_Digital_Input_St[counter].check == 0){
            if(Sb_Digital_Input_St[counter].Reading_State != Sb_Digital_Input_St[counter].Current_State){
              Sb_Digital_Input_St[counter].lastDebounceTime = millis();
              Sb_Digital_Input_St[counter].check = 1;
            }      
      }
      else{
          if(millis() - Sb_Digital_Input_St[counter].lastDebounceTime 
                          > Sb_Digital_Input_Cfg[counter].Deboucing_Time){
                  if(Sb_Digital_Input_St[counter].Reading_State 
                          != Sb_Digital_Input_St[counter].Current_State){                  
    
                      Sb_Digital_Input_St[counter].Previous_State = 
                          Sb_Digital_Input_St[counter].Current_State;
                      Sb_Digital_Input_St[counter].Current_State = 
                          Sb_Digital_Input_St[counter].Reading_State;
                          
                      Sb_Digital_Input_St[counter].Pin_State_Change = 1;
                  }
                  Sb_Digital_Input_St[counter].check = 0;
           }
      }
    
  }

    // Tilt sensor reading 

  if(Sb_Motion_Detection.Current_State == 1)
  {
    if(Sb_Motion_Detection.Counter_delay ++ > 200)
    {
      attachInterrupt(digitalPinToInterrupt(3), blink, FALLING);
      Sb_Motion_Detection.Current_State = 0;
      Sb_Motion_Detection.Pin_State_Change = 1;
    }
  }

  /* Led matrix logic control*/
        
  if(System_Init-- > 2)
  {
     Led_Matrix_Decide();
     return; 
  }
  else
  {
    System_Init = 1; // start Led matrix control
  }
  /*End Read input state and apply deboucing input pins */

  if(Sb_Digital_Input_St[Mode_SW].Current_State == 0)
  {
          First_Time_Picking_up = 0;
          /* Normal mode - compare with Picking up music mode*/
          
          //ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
           
          /*After that, we collect all input pins's state, we apply signal logic */
        
          /* Red led logic control*/
          
          if((Sb_Digital_Input_St[Hand_Detection].Current_State == 0) || (Sb_Digital_Input_St[Lid_Detection].Current_State == 0))
          {
            Sb_Led_Cfg[Pin_Red_Led_Id].Output = 0;
            Sb_Led_Cfg[Pin_Red_Led_Id].Timer_Counter = 0;
            Sb_Led_Cfg[Pin_Red_Led_Id].Is_Count_Up = 0;
          }
          else
          {
            if(Sb_Digital_Input_St[Lid_Detection].Current_State == 1)
            {
              Sb_Led_Cfg[Pin_Red_Led_Id].Is_Count_Up = 1;
            }
          }
         
          /* End Red led logic control*/
        
          /* RFID reading*/
          /*
          if(Sb_RFID_Test_Led[0].Timer_Counter < 35)
          {
              if (mfrc522.PICC_IsNewCardPresent()) {
                  if (mfrc522.PICC_ReadCardSerial()){
                    Serial.print(" UID : ");
            
                    Sb_RFID_Test_Led[0].Timer_Counter = 50;
                    for(counter = 0; counter < mfrc522.uid.size; counter++)
                    {
                        //Serial.println(mfrc522.uid.uidByte[counter]);
                        if(mfrc522.uid.uidByte[counter] != TAG_UID[counter])
                          return;
                    }
                    Sb_RFID_Test_Led[0].Output = 1;
                  }
              }
          }
          */
          
          if(Sb_Digital_Input_St[Reed_SW].Pin_State_Change == 1)
          {
            Sb_Digital_Input_St[Reed_SW].Pin_State_Change = 0 ;
            if(Sb_Digital_Input_St[Reed_SW].Current_State == 0)
            {
                Sb_Matrix_Led.Mode = Output_Flicker;
                Sb_Digital_Input_St[Lid_Detection].Pin_State_Change = 0;
                Sb_Motion_Detection.Pin_State_Change = 0;
                return;
            }
            else
            {
              Led_Matrix_Decide();
            }
          }
          
        
          if(Sb_Digital_Input_St[Lid_Detection].Pin_State_Change == 1)
          {
            //Serial.println("Lid_Detection change status");
            Sb_Digital_Input_St[Lid_Detection].Pin_State_Change = 0;
            if(Sb_Digital_Input_St[Lid_Detection].Current_State == 1)
            {
              Sb_Matrix_Led.Mode = Output_Lid_Open;
              Sb_Motion_Detection.Pin_State_Change = 0;
              return;
            }
            else
            {
              Led_Matrix_Decide();
            }
          }
        
        
          if(Sb_Motion_Detection.Pin_State_Change == 1)
          {
            Sb_Motion_Detection.Pin_State_Change = 0;
            if(Sb_Motion_Detection.Current_State == 1)
            {
              Sb_Matrix_Led.Mode = Output_ShoeBox_Move;
              return;
            }
            else
            {
              Led_Matrix_Decide();
            }
          }
          
          /* End Led matrix logic control*/
        
          /* Led matrix decision */
          
          //Serial.println(Sb_Matrix_Led.Mode);
          switch(Sb_Matrix_Led.Mode)
          {
            case Output_ShoeBox_Move:
                for (int i = 0; i < 8; i++)  
                {
                  lc.setColumn(0,i,Moving[i]);
                }
               
                break;
            case Output_Lid_Open:
                 for (int i = 0; i < 8; i++)  
                {
                  lc.setColumn(0,i,Lid_Open[i]);
                }
                
                break;
                
            case Output_Flicker:
            {
              static int toogling_on = 0;
              static int toogling_off = 0;
        
              if(toogling_on++ > 0)
              {
                   for (int i = 0; i < 8; i++)  
                  {
                    lc.setColumn(0,i,Lid_Open[i]);
                  }
                  toogling_on = 0;
              }
              else
              {
                   for (int i = 0; i < 8; i++)  
                  {
                    lc.setColumn(0,i,Led_Off[i]);
                  }
              }
        
              analogWrite(FLAME_RED_LED_1, random(100)+120);
              analogWrite(FLAME_RED_LED_2, random(100)+130);
              analogWrite(FLAME_YELOW_LED_1, random(100)+120);
                
                break;
            }
            case Output_Clapping:
                for(int j = 0; j < 3; j++)
                {
                     for (int i = 0; i < 8; i++)  
                    {
                      lc.setColumn(0,i,Lid_Open[i]);
                    }
                    delay(500);
                   for (int i = 0; i < 8; i++)  
                    {
                      lc.setColumn(0,i,Led_Off[i]);
                    }
                    delay(500);
                }
                Clapping_Sample_Recieve = 0;
                Clapping_Start_Check_t  = 0;
                Sb_Matrix_Led.Mode = Output_Off;
                return;
            break;
            
            default:
              for (int i = 0; i < 8; i++)  
              {
                lc.setColumn(0,i,Led_Off[i]);
              }
        
              analogWrite(FLAME_RED_LED_1, 0);
              analogWrite(FLAME_RED_LED_2, 0);
              analogWrite(FLAME_YELOW_LED_1, 0);
            break;
          }
          
          
          // Red led output decision
          
          if(Sb_Led_Cfg[Pin_Red_Led_Id].Is_Count_Up == 1)
          {
              if(Sb_Led_Cfg[Pin_Red_Led_Id].Timer_Counter++ > 300){
                Sb_Led_Cfg[Pin_Red_Led_Id].Timer_Counter = 300;
                Sb_Led_Cfg[Pin_Red_Led_Id].Output = 1;
              }
          }
          digitalWrite(Pin_Red_Led, Sb_Led_Cfg[Pin_Red_Led_Id].Output);
          
            
          if(Sb_Matrix_Led.Mode == Output_Off)
          {
              if(Clapping_Start_Check_t == 0)
              {
                  numSamples = 0;
                  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
                  Clapping_Start_Check_t = 1;
                  return;
              }
              
              //Serial.println("Read Analog ");
              
              while(ADCSRA & _BV(ADIE));
    
              fft_input(capture, bfly_buff);   // Samples -> complex #s
          
              fft_execute(bfly_buff);          // Process complex data
              
              fft_output(bfly_buff, spectrum); // Complex -> spectrum
          
              // Remove noise and apply EQ levels
              for(x=0; x<FFT_N/2; x++) {
                L = pgm_read_byte(&noise[x]);
                spectrum[x] = (spectrum[x] <= L) ? 0 :
                  (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
              }

              
    
              if((Clapping_Check == 0) || (Clapping_Check == 1))
              {      
                  for(numSamples = 0; numSamples < 30; numSamples++)
                  {
                    if(spectrum[numSamples] > 30)
                    {
                      //Serial.print(spectrum[numSamples]);   
                      //Serial.print(" -- ");
                      //Serial.println(numSamples);
                      //Serial.println("Over threshold ");
                      Clapping_Check = 1;
                      Clapping_Delay_Counter = 3;
                      //break;
                    }
                  }
              }
    
              if(Clapping_Check == 2)
              { 
                  //Serial.print("---------------------------------");
                  //Serial.println("Clapping_Sample_Counter");
                  Clapping_Check = 3;
                  Clapping_Sample_Recieve ++;
                  Clapping_Delay_Counter = 1;
                  if(Clapping_Start_Check == 0)
                  {
                    Clapping_Sample_Period = 0;
                    Clapping_Start_Check = 1;
                  }
              }

              numSamples = 0;
              ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
          
          }
          else
          {
              Clapping_Sample_Recieve = 0;
              Clapping_Start_Check_t  = 0;
          }
         
    }
    else
    {
        if(First_Time_Picking_up == 0)
         {
              numSamples = 0;
              ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
              First_Time_Picking_up = 1;
              analogWrite(FLAME_RED_LED_1, 0);
              analogWrite(FLAME_RED_LED_2, 0);
              analogWrite(FLAME_YELOW_LED_1, 0);
              return;
              
         }

        while(ADCSRA & _BV(ADIE));       // Resume sampling interrupt
         
        /* Picking-up music mode*/
        Led_Matrix_Dis = 1;
    
        for(numSamples = 0; numSamples < FFT_N; numSamples++)
        {
            if(capture[numSamples] > 30)
            {
              
                  Led_Matrix_Dis = 0;
                  break;
            }
        }
    
        if(Led_Matrix_Dis == 0)
        {
     
            fft_input(capture, bfly_buff);   // Samples -> complex #s
        
            fft_execute(bfly_buff);          // Process complex data
            
            fft_output(bfly_buff, spectrum); // Complex -> spectrum
        
            // Remove noise and apply EQ levels
            for(x=0; x<FFT_N/2; x++) {
              L = pgm_read_byte(&noise[x]);
              spectrum[x] = (spectrum[x] <= L) ? 0 :
                (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
            }
            
            //Serial.print("Raw Spectrum Value :");
            //Serial.println(counter++);
            
            
    
            //for(Led_Matrix_Index = 0;Led_Matrix_Index<64;Led_Matrix_Index++)
              //Serial.println(spectrum[Led_Matrix_Index]);
              
            Max_Amp = 0;
            Led_Matrix_Index = 0;
            
            while(Led_Matrix_Index < 8)
            {
              Led_Matrix_Vertical[Led_Matrix_Index] = 0;
              
              for(Led_Matrix_Counter = lower[Led_Matrix_Index];Led_Matrix_Counter < upper[Led_Matrix_Index];Led_Matrix_Counter++)
                Led_Matrix_Vertical[Led_Matrix_Index] += spectrum[Led_Matrix_Counter];
        
              if(Led_Matrix_Vertical[Led_Matrix_Index] > Max_Amp)
                Max_Amp = Led_Matrix_Vertical[Led_Matrix_Index];
                
              //Serial.println(Led_Matrix_Vertical[Led_Matrix_Index]);
              Led_Matrix_Index++;
            }
            
            //Serial.println("-------------------------------");
    
            Resolution = Max_Amp / 8;
    
            for(Led_Matrix_Index = 0; Led_Matrix_Index < 8; Led_Matrix_Index++)
              Led_Matrix_Horizontal[Led_Matrix_Index] = Led_Matrix_Vertical[Led_Matrix_Index] / Resolution;
    
            for(Led_Matrix_Index = 0; Led_Matrix_Index < 8; Led_Matrix_Index++)
            {
              switch(Led_Matrix_Horizontal[Led_Matrix_Index])
              {
                  case 8:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 255;
                  break;
    
                  case 7:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 127;
                  break;
    
                  case 6:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 63;
                  break;
    
    
                  case 5:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 31;
                  break;
    
    
                  case 4:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 15;
                  break;
    
    
                  case 3:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 7;
                  break;
    
    
                  case 2:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 3;
                  break;
    
                  default:
                      Led_Matrix_Horizontal[Led_Matrix_Index] = 1;
                  break;
                
              }
    
            }
    
            for(counter = 0; counter < 8; counter++)
            {
                  lc.setRow(0,counter,Led_Matrix_Horizontal[counter]);
            }
    
            delay(30);
            
            //Led_Matrix_Dis = 1;
            //Led_Matrix_Delay = 3;
            for(counter = 0; counter < 8; counter++)
            {
                  lc.setRow(0,counter,0);
            }
            
        }
        else
        {
          for(counter = 0; counter < 8; counter++)
          {
                Led_Matrix_Horizontal[counter] = (Led_Matrix_Horizontal[counter]) / 2; 
                lc.setRow(0,counter,Led_Matrix_Horizontal[counter]);
          }
           delay(30);
          for(counter = 0; counter < 8; counter++)
          {
                lc.setRow(0,counter,0);
          }
        }
        
        Led_Matrix_Dis = 1;
        numSamples = 0;
        ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
    }
}
void timerIsr()
{
  int counter;
  static int Serial_counter = 0;

 if(Serial_counter++ == 30)
  {
      //Serial.println("Current state change:");
      for(counter =0; counter < No_Digital_Pins; counter++)
      {
        //Serial.print(" ");
        //Serial.print(Sb_Digital_Input_St[counter].Pin_State_Change);
        Sb_Digital_Input_St[counter].Pin_State_Change = 0;
      }
      //Serial.println(" ");
      //Serial.println("Current state :");
      for(counter =0; counter < No_Digital_Pins; counter++)
      {
        //Serial.print(" ");
        //Serial.print(Sb_Digital_Input_St[counter].Current_State);
      }
      //Serial.println(millis());

      Serial_counter = 0;
  }

  
  if(Sb_Digital_Input_St[Mode_SW].Current_State == 0)
  {
      if(Sb_Matrix_Led.Mode == Output_Off)
      {
        if(Clapping_Check == 1)
          if(Clapping_Delay_Counter-- == 0)
              Clapping_Check = 2;
        
            if(Clapping_Check == 3)
            {
              if(Clapping_Delay_Counter-- == 0)
              {
                Clapping_Sample_Counter++;
                Clapping_Check = 0;
              }
            }
        if(Clapping_Start_Check == 1)
        {
          if(Clapping_Sample_Period++ == 35)
           {
                if((Clapping_Sample_Recieve > 2) && (Clapping_Sample_Recieve < 5))
                {
                  Serial.println("Correct pattern");
                  Sb_Matrix_Led.Mode = Output_Clapping;
                }
                else
                {
                  Serial.println("Incorrect pattern"); 
                }
                Clapping_Sample_Period = 0;
                Clapping_Start_Check = 0;
                Clapping_Sample_Recieve = 0;
           }
        }
      }
  }

}

