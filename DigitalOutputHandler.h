/* Red led*/
#define No_Led_Pins         1
#define Pin_Red_Led_Id      0
#define Pin_Red_Led         39


struct Red_Led_control{
  bool Output;
  int Timer_Counter;
  bool Is_Count_Up;
};

