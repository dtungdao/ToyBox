/* Analog input read*/
#define Tilt_Sensor         4

struct Analog_Input_St{
  int  Counter_delay;
  int  Current_Reading_Value;
  int  Output_Threshold;
  bool Pin_State_Change;
  bool Current_State;
};

