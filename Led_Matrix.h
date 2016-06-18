#define LED_MX_CS    10
#define LED_MX_CLK   11
#define LED_MX_DIN   12

enum Led_Matrix_Mode {
  Output_Off = 0,
  Output_ShoeBox_Move,
  Output_Lid_Open,
  Output_Flicker,
  Output_Clapping,
  Output_Picking_Up_Music
};

struct Led_Matrix_control{
  Led_Matrix_Mode Mode;
  int Timer_Counter;
};



