/* We use PWM output pin 7 to emulate output in LED matrix*/
#define Test_Pin 43

#define Pin_RFID_Test_Led_Id           1
#define Pin_RFID_Test_Led              7


/* RFID pins */
#define RST_PIN         5          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above

struct RFID_Test_Led_control{
  bool Output;
  int Timer_Counter;
  int Pin_Id;
};

