/* Digital Input */
#define No_Digital_Pins     4

#define Reed_SW             0
#define Lid_Detection       1
#define Hand_Detection      2
#define Mode_SW             3

struct Digital_Input_Cfg{
  int Pin_Number;
  long Deboucing_Time;
};
struct Digital_Input_St{
  bool check;
  bool Reading_State;
  bool Current_State;
  bool Previous_State;
  bool Pin_State_Change;
  long lastDebounceTime;
};

