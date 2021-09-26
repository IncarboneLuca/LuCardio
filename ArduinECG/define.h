#define crono_STOP_RECORDING   5
#define crono_RUN_cardio_REC_start  4
#define crono_RUN_cardio_REC  3
#define crono_START_pressed  2
#define crono_RUN  1
//Treshold to get pick in "RSwave" search
#define alpha 5
#define beta 3
#define treshold 30
#define tmpAddress 0b1001011
#define ResolutionBits 8
#define chip_select 10
#define weel_circonference 2.1

//SD library constant dipendency
#define MAX_CHARACTER_FOR_FILE 13

//button and pin
#define 1
#define 2
#define 3
#define 4
#define 5
#define 6
#define 7
#define  8
#define  9
#define pin_Vbutton A0
#define pin_green_but A1
#define pin_QRS A2
#define pin_ECG A3 
#define pin_I2C_SDA A4
#define pin_I2C_SCL A5
#define pin_Battery A6
#define pin_red_but A7

//constants
//treshold for Low Battery indicator --->  7V * 1.2k/3k = 2.8 --->   1023*2.8/5 = 572
#define limit_battery 800 
//#define secDot 0.0034375 //Constant to compute the "bpm"
#define size_pipe_ecg 14
#define half_size_pipe_ecg 7
#define TRESHOLD 3
#define TAU 10 

//ADD0 Vdd ADD1 GND
#define TMP101 0b1001000

//Distance between two analysed dots in "RSwave" search
#define delta 5 

#define BUTTON_touch
//#define BLUETOOTH
#define TERMOMETRE

#define RAMSIZE 2048 //you can probably get this from another define somewhere
//#define INSTALLATION_PROCEDURE 1

FLASH_STRING(t_lucardio, "LuCardio 2013 ")
FLASH_STRING(t_sdfailure, "SD card FAILURE")
FLASH_STRING(t_sdcarddetected, "SD card detected")
FLASH_STRING(t_rtcnotrunning, "RTC NOT running!")
FLASH_STRING(t_rtcisrunning, "RTC is running!")
FLASH_STRING(t_lucardiotwo, "->>>>      ")
FLASH_STRING(t_downloading, "downloading")
FLASH_STRING(t_lowbattery, "LB")
FLASH_STRING(t_kilometer, "km")
FLASH_STRING(t_meter, "m")
FLASH_STRING(t_speedbike, "speedBike")
FLASH_STRING(t_bpm, "bpm")
FLASH_STRING(t_distance_float,"distance_float")
FLASH_STRING(t_temperature, "temperature")
FLASH_STRING(t_r, "R")
FLASH_STRING(t_filesavedon,"File Saved on:")
FLASH_STRING(t_minus,"-")
FLASH_STRING(t_celsius, "C")
FLASH_STRING(t_beginoffile, "bpm;speed bike;distance float;temperature;time")
FLASH_STRING(t_empty,"    ")
FLASH_STRING(t_dotcomma,";")
FLASH_STRING(t_dotdot,":")
FLASH_STRING(t_dot,".")
