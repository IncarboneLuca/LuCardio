/**
LuCardio UNO
  CardiOpen, project version by Luca 
  ---------------------------------------------------------------
  Author         : Luca Incarbone <luca@incarbone.com>
  First revision : 10/03/2012
  Last revision  : 20/04/2013
---------------------------------------------------------------
---------------------------------------------------------------
  LuCardioUNO v1
      -ATMEGA328P-AU
      -NOKIA LCD display 
      -microSD card
      -ecg detector (LT1496 + R + C)
      -Bluetooth module comunication
      -GPS module EB-356
      -Bike speed detection / charging battery
      -Pedal rate detection
      -1 cell Li-ion battery Power Supply
      -LTC4053 Li-ion charging battery
      -LTC1844 - 3.3V - 150mA 
      -TMP100 temperature sensor
      -Gyroscope MAX21000

Digital	PD0	RX - Bluetoth -(or possible connction to GPS)
	PD1	TX - Bluetoth -(or possible connction to GPS)
	PD2	Wheel sensor interrupt 3.3k pulldown
	PD3	Bluetooth on/off (set pin to pull-down)
	PD4	Interrupt Gyroscope
	PD5	LCD NOKIA - SCE
	PD6	LCD NOKIA - RST
	PD7	LCD NOKIA - D/C
	PB0     LCD NOKIA - DN(MOSI) INPUT GREEN button 1k2 pulldown	- output LEFT button
	PB1     SCLK  INPUT RED button 1k2 pulldown 	- output RIGHT button
	PB2	CS 	SD 
	PB3	MOSI 	SD 
	PB4	MISO 	SD 
	PB5	SCK 	SD 
        PB6     Pedal sensor with 3.3k pull-up
        PB7     Backlight Control (via MOS) (set pin to pull-up)
Analog	PC0	Vbutton - input button (Left/Right) to get finger capacity
	PC1	Green Button
	PC2	ECG filtred 
	PC3     ECG
	PC4	SDA 	- GPS - temperature sensor TMP100 - Gyroscope
	PC5	SCLA 	- GPS - temperature sensor TMP100 - Gyroscope
	PC6	RESET 
        ADC6    Battery check
        ADC7    Red Button

*/
#include "Arduino.h"
#include "Flash.h"
#include "define.h"


//Value read from ADC sensorValue_*
  int sensorValue_ECG;
  int sensorValue_QRS;
  int sensorValue_Vbutton;

//buttons counters
  uint8_t counter_green_but;
  uint16_t counter_red_but;

//Counter to compute speed and distance
  uint8_t bike_loop;//counter time to make loop. 5 to 255 (2 to 113Km/h)  ====> minbikellop = (75*3.6*2.1)/113 = 5
  uint8_t bikeLoop;//to compute value 

//boolean variable
  boolean new_val;//boolean became true with 75Hz interrupt NEW VALUE is available
  boolean red_or_green;//to control alternatively two button
  boolean green_pushed;//boolean true if green is pushed
  boolean red_pushed;//boolean true if red is pushed
  boolean REC; //true if the system is recording
  boolean CRONO; //true if crono is running

//Value for mode selection (changed the display output)
  boolean MODE_1;

//Value changed by serial command change mode ECG/Cardio 
  boolean BT_BPM; //send BPM over bluetooth
  boolean BT_ECG; //send ECG over bluetooth
  boolean ECG; //send ECG over serial
  boolean GET_SD_FILES;//Send the file names find in SD card
  boolean R_FILE;//Receve name from bluetooth
  boolean S_FILE;//Read file in SD card from bluetooth

//Cardio
  uint8_t bpm;//Pulse par minute
  uint16_t lastPik;//Variable to store counter value
  uint16_t counterRR;//Counter to measure the distance between two Rwave R-R
  int ref_1, ref_10;

//Temperature
  int temperature;

//Bike variable
  float distance_float;//m
  uint8_t speedBike;//km/h

//crono variable
  boolean old_refresh;//true each second
  uint8_t state;

//name of file for recording
  char filename[MAX_CHARACTER_FOR_FILE];//"mmddhhmm.csv" month-day-hour-minute
  
//retroillumination
  boolean retroLight;
  uint8_t retroLight_count;

//Variables for communication
  uint8_t reciving_index;
  char com_in;
  
#include "Wire.h"

//Time
  int second,minute, hour;
  DateTime moment;
  DateTime start_crono_time;//time when Start button is pressed
  
#include "SD.h"
  
//SD variable
  boolean SDbegin;
  File logfile;

void timers_setup();
void SetResolution();
int8_t getTemperature();
void get_command_viaRS232();
void image_refresh();
void data_processing();
void button_irq();
void button_check();
void LCD_refresh();
void getFilename(char *);
//int freeRam();
/*
  Setup fonction:
	-setup Serial comunication
	-setup ECG variable default value
	-counter setup
*/

//int main(void){
void setup(){
  //interrupt for weel sensor
  attachInterrupt(0, weel, RISING);
  //variable
  state = 6;
  old_refresh = false;
  retroLight = true;
  BT_BPM = false;
  BT_ECG = false;
  ECG = false;
  R_FILE = false;
  S_FILE = false;
  GET_SD_FILES = false;
  counterRR = 0;
  
  ///// pin ///////
  pinMode(chip_select, OUTPUT);//CS for SPI
  pinMode(pin_Battery, INPUT);//Battery Checker
  pinMode(pin_BackLight, OUTPUT);digitalWrite(pin_BackLight, HIGH);// set pull-up resistor for pin 3, retroillumination via MOS
#ifdef BUTTON_touch
  pinMode(pin_green_but, INPUT);//GREEN button
  pinMode(pin_red_but, INPUT);//RED button
#else 
  pinMode(pin_green_but, OUTPUT);//GREEN button
  pinMode(pin_red_but, OUTPUT);//RED button
#endif
  pinMode(pin_ECG, INPUT);//ECG not filtred
  pinMode(pin_QRS, INPUT);//ECG filtred
  pinMode(pin_Vbutton, INPUT);//Touch button sense

  ////////////  LCD //////////////
 //http://playground.arduino.cc/Code/PCD8544
  ////// I2C ////////
  Wire.begin();        

  //////  Temperature Sensor /////
  SetResolution();

  ////// Serial (bluetooth) //////
  //Serial.begin(2400);//Bluetooth connection
  Serial.begin(115200);//USB debug connection

  lcd.clear();lcd.setCursor(0,0);
  ///////////// Memory SD ////////////
  if (!SD.begin(chip_select)) {
	  lcd<<t_sdfailure;SDbegin = false;
	  return;
  }

  SDbegin = true;

  lcd<<t_sdcarddetected;
  lcd.clear();lcd.setCursor(0,0);

  RTC.begin();
  if (!RTC.isrunning()) {
	lcd<<t_rtcnotrunning;delay(1000);
  }else{
	lcd<<t_rtcisrunning;
  }
  delay(100);
  
#ifdef INSTALLATION_PROCEDURE
  //RTC.adjust(DateTime(__DATE__, __TIME__));
#endif

  /////////////////  Timer Interrupt ////////////////
  cli();//stop interrupts
  
  /////////////////  Timer Interrupt ////////////////
  timers_setup();

  sei();//allow interrupts
//StartUP image

}


/*
  Loop function:
		-get value from ADC0
		-check serial data (to change mode)
		  -In mode ECG send the value and wait 5ms
		  -In mode Cardio fill the buffer, detect the pick and compute the bpm
*/
void loop(){

	    if(!S_FILE){
			get_command_viaRS232();

			data_processing();

			image_refresh(); 

			button_check();
		}else{
			lcd.clear();lcd.setCursor(0,0);lcd<<t_downloading;
			Serial.println(filename);
			logfile = SD.open(filename);
			if(logfile){
				// read from the file until there's nothing else in it:
				while(logfile.available()) {
					Serial.write(logfile.read());
					//lcd.clear();lcd.setCursor(0,0);lcd.print(freeRam());
				}
				// close the file:
				logfile.close();
			}
			S_FILE = false;
		}
}
/*
	________________
	|100000m    REC|	distance
	|25C    BPM    |	temp 	BPM
	|80km/h BPM    |	GPSpeed	BPM
	|80km/h	       |	speed	
	|nSAT  00:00:00|	n SAT	crono
	|bat   00:00:00|	battery	time
	----------------
	________________
	|100000m    REC|	distance
	|25C   80   BPM|	temp 	BPM
	|	       |	Graph
	|______________|		heartrate
	|	       |	Graph
	|	       |		speed
	----------------
	________________
	|100000m    REC|	distance
	|25C   80   BPM|	temp 	BPM
	|	       |	Graph
	|_A_ ______A __|		heart
	|   V       V  |	
	|	       |		
	----------------

*/
void LCD_refresh(){
	String dataString;
#ifndef LCD_V2
        temperature = getTemperature();
        lcd.clear();
        lcd.setCursor(13,0); lcd.print(bpm);
        lcd.setCursor(0,1); lcd.print(speedBike);//lcd.print(speed);
      //lcd.setCursor(2,1); lcd.print("km/h");
      
      lcd.setCursor(4,1); lcd.print(temperature);lcd<<t_celsius;
	  lcd.setCursor(8,1);
	  if(CRONO){  
                    if(moment.second() > start_crono_time.second()){
			second = (moment.second() - start_crono_time.second());
                    }else if(moment.second() == start_crono_time.second()){
                        second = 0;
		    }else{
			second = (60 + moment.second() - start_crono_time.second());
	  	    }
                    if(second==0){
                      if(moment.minute() > start_crono_time.minute()){
			minute = moment.minute() - start_crono_time.minute();
		      }else if(moment.minute()== start_crono_time.minute()){
                        minute = 0;
                      }else{
			minute = 60 + moment.minute() - start_crono_time.minute();
		      }
                    }
                    if(minute==0){
                      if(moment.hour() > start_crono_time.hour()){
			hour = moment.hour() - start_crono_time.hour();
                      }else if(moment.hour() == start_crono_time.hour()){
                        hour = 0;
        	      }else{
			hour = 24 + moment.hour() - start_crono_time.hour();
		      }
                    }
		lcd.print(hour);
		lcd.print(t_dotdot[0]);
		lcd.print(minute);
		lcd.print(t_dot[0]);
		lcd.print(second);
		
	  }else{
	        dataString = String(moment.hour());
		dataString += t_dotdot[0];
		dataString += String(moment.minute());
		dataString += t_dot[0];
		dataString += String(moment.second());
		lcd.print(dataString);
	  }		  
	  
	  int sensorValue_battery;//A2
	  sensorValue_battery = analogRead(pin_Battery);
          //lcd.setCursor(8,0);lcd.print(sensorValue_battery);
      if(sensorValue_battery<limit_battery){
        lcd.setCursor(8,0);lcd<<t_lowbattery;
      }
	  lcd.setCursor(0,0);lcd<<t_empty;
      if (distance_float > 10000.0){
		lcd.setCursor(0,0); lcd.print((int)(distance_float/1000));lcd<<t_kilometer;
      }else{
		lcd.setCursor(0,0); lcd.print((int)distance_float);lcd<<t_meter;
      }
#endif
}

#ifdef BUTTON_touch
void button_check(){
	if(digitalRead(pin_green_but)){
		counter_green_but++;
	}else{
		counter_green_but = 0;
	}
	if(digitalRead(pin_red_but)){
		counter_red_but++;
	}else{
		counter_red_but =0;
	}
	if((counter_green_but==255) & (state!=crono_RUN_cardio_REC)){
		retroLight=true;
		state=crono_START_pressed;
		counter_green_but =0;
	}else if(counter_green_but==255){
		retroLight=true;
		counter_green_but =0;
	}
	if((counter_red_but==(65535/2)) & (state==crono_RUN_cardio_REC)){
		state=crono_STOP_RECORDING;
		counter_red_but =0;
	}

}
#else
void button_check(){
	//if the values  are 0 then we are ready to make new test
	if(red_but == 0 & green_but == 0){
		if(red_or_green){
			counter_red_but = 0;
			green_but = 1;
		}else{
			counter_red_but = 0;
			red_but = 1;
		}
	}

	if(red_pushed & state==crono_RUN_cardio_REC){
		//stop recording
		state=crono_STOP_RECORDING;
		red_but = 0;
		//reset counter variable, used to wait unload capacity. two time longer to get it close to zero volt
	for(counter_red_but=counter_red_but*2;counter_red_but>0;counter_red_but--){}
}

if(green_pushed & state!=crono_RUN_cardio_REC){
	//turn on the backlight
	retroLight=true;
	//change state to start crono mode
	state=crono_START_pressed;
	green_but = 0;
	//reset counter variable, used to wait unload capacity. two time longer to get it close to zero volt
for(counter_red_but=counter_red_but*2;counter_red_but>0;counter_red_but--){}
  }else if(green_pushed){
	//turn on the backlight
    retroLight=true;
    green_but = 0;
	//reset counter variable, used to wait unload capacity. two time longer to get it close to zero volt
    for(counter_red_but=counter_red_but*2;counter_red_but>0;counter_red_but--){}
  }

}

void button_irq(){
  counter_red_but++;
//get value
  step_mes = analogRead(pin_Vbutton);  i_step_mes = (int)step_mes;
//test value 
  if(i_step_mes > TRESHOLD){
	//evaluation time constant
    if(counter_red_but > TAU){
	//select original pullup signal
      if(red_or_green){
        green_pushed = true;
      }else{
        red_pushed = true;
      }  
	//time constant is lower than limit = no finger
    }else{
      green_pushed = false;
      red_pushed = false;
    }
	//get treshold we change button test pullup 
    red_or_green = not red_or_green;
  }
}
#endif 

void data_processing(){
  /************************************************************************************/
  //Send the value to IHM in mode ECG
  /*
    ECG  active simple ECG cube send (stand alone ECG)
    BPM  Active cardio mode
      BT_BPM send via Bluetooth the heart-rate
      BT_ECG send via Bluetooth the ECG
  */
  if(new_val){
    if(ECG)Serial.println(sensorValue_QRS);//mode ECG
    if(BT_BPM) Serial.println(int(bpm));//send via Bluetooth data
#ifndef BUTTON_touch
    if(ECGnotFiltred) Serial.println(sensorValue_ECG); //for Bluetooth future module
#endif	
	new_val=false;
  }    
  if(GET_SD_FILES){
	// set up variables using the SD utility library functions:
	Sd2Card card;
	SdVolume volume;
	SdFile root;
	int chipSelect = chip_select;
	card.init(SPI_HALF_SPEED, chipSelect);
	volume.init(card);
	root.openRoot(volume);
	// list all files in the card with date and size
	root.ls(LS_R);	
	GET_SD_FILES = false;		
  }
}
void getFilename(char *filename){
	//int year = now.year(); 
	int month = moment.month(); 
	int day = moment.day(); 
	int hour = moment.hour(); 
	int minute = moment.minute();
	/*filename[0] = '2';
	filename[1] = '0';
	filename[2] = year%10 + '0';
	filename[3] = year%10 + '0';*/
	filename[0] = month/10 + '0';
	filename[1] = month%10 + '0';
	filename[2] = day/10 + '0';
	filename[3] = day%10 + '0';
	filename[4] = hour/10 + '0';
	filename[5] = hour%10 + '0';
	filename[6] = minute/10 + '0';
	filename[7] = minute%10 + '0';
	filename[8] = '.';
	filename[9] = 'C';
	filename[10] = 'S';
	filename[11] = 'V';
	return;
}

void image_refresh(){
	String dataString;
  /************************************************************************************/
  //manage the refresh of LCD display
  if(old_refresh){
        //ECG bpm
        uint8_t bpm_tmp = (250*60/(lastPik));// =15000/30 =15000/210
        if(!CRONO){
          bpm = bpm_tmp;
        }else if((bpm_tmp < (bpm-40))&(bpm_tmp > (bpm+40))){
          bpm = bpm;
        }else if((bpm_tmp < (bpm-20))&(bpm_tmp > (bpm+20))){
          bpm = (bpm + bpm_tmp)/2;
        }else{
          bpm = bpm_tmp;
        }
	moment = RTC.now();
	float ruota_x_bike_loop;
        ruota_x_bike_loop = ((weel_circonference*75)/bikeLoop);
        speedBike = ruota_x_bike_loop*3.6;
        if(speedBike < 3)speedBike = 0;

        switch (state) {
          case crono_RUN:
		LCD_refresh();
                break;
          case crono_START_pressed:
#ifndef LCD_V2
		lcd.clear();
#endif
                //NAME date and hour using RTC
                getFilename(filename);
		//single name with start time!!
		logfile = SD.open(filename, FILE_WRITE);
                if(logfile){
	                logfile<<t_beginoffile;
                        logfile.println(" ");
	                logfile.close();
#ifndef LCD_V2
                        lcd.setCursor(0,0); lcd.print(filename);
#endif
                }
		start_crono_time = moment;
                distance_float = 0;
                second = 0;minute = 0; hour= 0;
                CRONO = true;//start crono
                REC = true;//start recording
                state = crono_RUN_cardio_REC;
				break;
          case crono_RUN_cardio_REC:
                LCD_refresh();
#ifndef LCD_V2
		lcd.setCursor(11,0);lcd<<t_r;
#endif
                break;
          case crono_STOP_RECORDING:
                REC = false; //stop recording
                CRONO = false; //stop recording
                retroLight =true;
#ifndef LCD_V2
		lcd.clear();
                lcd.setCursor(0,0); lcd<<t_filesavedon;
      		lcd.setCursor(0,1); lcd.print(filename);
#endif
		state = 6;
                update_total_km((int)distance_float);
		delay(50);
                          break;
           default:                
                LCD_refresh();
                break; 
        }
        //back-light stay for 3 seconds
        if(retroLight){
          if(retroLight_count<3){
            retroLight_count++;
          }else{
            retroLight_count = 0;
            retroLight =false;
          }
          digitalWrite(pin_BackLight, HIGH);
        }else{
          digitalWrite(pin_BackLight, LOW);
        }
        if(REC & SDbegin){

          dataString = String(int(bpm));
          dataString += t_dotcomma[0]; 
          dataString += String(speedBike);
	  dataString += t_dotcomma[0]; 
	  dataString += String((int)distance_float);
	  dataString += t_dotcomma[0]; 
          dataString += String(temperature);
       	  dataString += t_dotcomma[0]; 
          dataString += String(moment.hour());
       	  dataString += t_dot[0]; 
          dataString += String(moment.minute());       	  
          dataString += t_dot[0]; 
          dataString += String(moment.second());
	  logfile = SD.open(filename, FILE_WRITE);
          if(logfile){
            logfile.println(dataString);                
            logfile.close();
          }
        }          
     	old_refresh = false;
  }
}
void get_command_viaRS232(){
	//get command from serial port (if any)
	if (Serial.available() > 0) {
		if(!R_FILE){
			// read the incoming byte:
			com_in = Serial.read();
			switch (com_in){
				
				case 'E':
				ECG=true;
				break;
				
				case 'G':
				GET_SD_FILES = true;
				break;
				
				case 'B':
				BT_ECG=true;
				break;
				
				case 'Y':
				BT_BPM=true;
				break;
				
				case 'R':
				BT_BPM=false;BT_ECG=false;ECG=false;R_FILE=true;reciving_index=0;
				break;
				
				case 'D':
				SD.remove(filename);
				break;
				
				default:
				BT_BPM=false;BT_ECG=false;ECG=false;R_FILE=false;S_FILE=false;GET_SD_FILES=false;//cardio mode is the default mode
			}
		}else{
			while ((reciving_index < MAX_CHARACTER_FOR_FILE) & (filename[reciving_index-1] != 'V')){
				if(Serial.available() > 0){
				 char inChar = Serial.read(); // Read a character
				 filename[reciving_index] = inChar; // Store it
				 reciving_index++; // Increment where to write next
				}				 
			}	
			filename[reciving_index] = '\0';
			R_FILE=false;
			S_FILE=true;
		}
	}
}

int8_t getTemperature(){
  Wire.requestFrom(TMP101,2);
  byte MSB = Wire.read();
  byte LSB = Wire.read();
 
  int TemperatureSum = ((MSB << 8) | LSB) >> 4;
 
  float celsius = TemperatureSum*0.0625;
  return (int8_t)celsius;
}
 
void SetResolution(){
  Wire.beginTransmission(TMP101);
  Wire.write(B00000001); //addresses the configuration register
  Wire.write(((ResolutionBits-9) << 5)); //writes the resolution bits
  Wire.endTransmission();
 
  Wire.beginTransmission(TMP101); //resets to reading the temperature
  Wire.write((byte)0x00);
  Wire.endTransmission();
}

uint16_t get_total_km(){
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(8);	
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 2);
  uint16_t total = Wire.read();
  total = total * 256;
  total = Wire.read() | total;
  Wire.endTransmission();
return total;
}

void update_total_km(int partial){
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(8);	
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 2);
  uint16_t total = Wire.read();
  total = total * 256;
  total = Wire.read() | total;
  Wire.endTransmission();
  total =  partial + total;
  
  uint8_t total_MSB = total/256;
  uint8_t total_LSB = total - (total_MSB*256);
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(8);
  Wire.write(total_MSB);
  Wire.write(total_LSB);
  Wire.endTransmission();
}

void timers_setup(){
	//set timer0 interrupt at 250Hz
	TCCR0A = 0;// set entire TCCR2A register to 0
	TCCR0B = 0;// same for TCCR2B
	TCNT0  = 0;//initialize counter value to 0
	// set compare match register for 250Hz increments
	OCR0A = 60;// = (16*10^6) / (250*1024) - 1 (must be <61)
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS12 and CS10 bits for 1024 prescaler
	TCCR0B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK0 |= (1 << OCIE0A);

	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <15625)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS12 and CS10 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	//set timer2 interrupt at 75Hz
	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
	// set compare match register for 22hz increments
	OCR2A = 207;// = (16*10^6) / (75*1024) - 1 (must be <207)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS11 bit for 1024 prescaler
	TCCR2B |= (1 << CS20)|(1 << CS21)|(1 << CS22);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);
}

ISR(TIMER0_COMPA_vect){//timer0 interrupt 250Hz 
  //get value from ADC (ECG)
#ifdef BUTTON_touch
	sensorValue_QRS = analogRead(pin_QRS);
#else
	sensorValue_ECG = analogRead(pin_ECG);
#endif
	counterRR++;//500:71   
    //(250*60/(lastPik)); =15000/30 =15000/210  ===> 500:71
#ifndef ECG_not_filtred
        ref_1 = (sensorValue_QRS/2) + (ref_10/2);
        ref_10 = sensorValue_QRS;
#else
        ref_1 = (sensorValue_ECG/2) + (ref_10/2);
        ref_10 = sensorValue_ECG;
#endif
        if((ref_1 > 9)&(counterRR>71)&(counterRR<500)){
		lastPik = counterRR;
		counterRR=0;
	}else if(counterRR>700){//end of RS wave detection
                counterRR=0;
    	}
        

//Serial.println(ref_1);
}

void weel(){
	if(bike_loop > 4){
		bikeLoop = bike_loop;
		distance_float = distance_float + weel_circonference;
		bike_loop = 0;
	}
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz 
  old_refresh = true;
}
  
ISR(TIMER2_COMPA_vect){//timer1 interrupt 75Hz 
	if(bike_loop<255){
          bike_loop++;
        }else{ 
          bikeLoop = bike_loop;
        }
	new_val = true;
#ifndef BUTTON_touch
	button_irq();
#endif
}
/*
int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}*/
/*
\struct pour documenter une structure C.
\union pour documenter une union C.
\enum pour documenter un type énuméré.
\fn pour documenter une fonction.
\var pour documenter une variable / un typedef / un énuméré.
\def pour documenter un #define.
\typedef pour documenter la définition d'un type.
\file pour documenter un fichier.
\namespace pour documenter un namespace.
\package pour documenter un package Java.
\interface pour documenter une interface IDL.
\brief pour donner une description courte.
\class pour documenter une classe.
\param pour documenter un paramètre de fonction/méthode.
\warning pour attirer l'attention.
\author pour donner le nom de l'auteur.
\return pour documenter les valeurs de retour d'une méthode/fonction.
\see pour renvoyer le lecteur vers quelque chose (une fonction, une classe, un fichier...).
\throws pour documenter les exceptions possiblement levées.
\version pour donner le numéro de version.
\since pour faire une note de version (ex : Disponible depuis ...).
\exception pour documenter une exception.
\deprecated pour spécifier qu'une fonction/méthode/variable... n'est plus utilisée.
\li pour faire une puce.
\todo pour faire un To Do (= "à faire")
\fixme pour faire un Fix Me (= "Réparez-moi").
*/
