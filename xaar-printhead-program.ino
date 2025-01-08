/***************************************************
 This is a library for Print-A-Sketch, an open-source handheld printer prototype for sketching circuits and sensors.
 For more information on the project, visit our website.
 Designed and tested to work with Arduino Uno and MEGA2560.
 
 Written by Narjes Pourjafarian, Marion Koelle, Fjolla Mjaku,
			      Paul Strohmeier and Juergen Steimle (Saarland University)

 Xaar128 files are a modified version of the library written by https://github.com/gkyle.
			
 MIT license, all text above must be included in any redistribution
 ****************************************************
 Based on the Print-A-Sketch project, this program is remade by huiyenc, Chia Hui Yen for CMU MSCD Architectural Robotics Lab project.
 ****************************************************/

#include "Arduino.h"
#include "xaar128.h"

#define XAAR_BEGIN SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2))
#define XAAR_END   SPI.endTransaction()

//Bit shifting
typedef unsigned char uchar;
typedef unsigned int uint;
#define ROBOT_SIGNAL_PIN 5
#define ABS(x) ((x)<0?-(x):(x))
#define SIGN(x) ((x)<0?-1:+1)
#define SHIFT_IN_BYTE 0x00 // bit values to be shifted in the array
#define SHIFT_L(ptr,shift) (*(uchar*)(ptr)<<(shift)|*((uchar*)(ptr)+1)>>(8-(shift)))
#define SHIFT_R(ptr,shift) (*(uchar*)(ptr)>>(shift)|*((uchar*)(ptr)-1)<<(8-(shift)))
#define SHIFT(ptr,shift) ((shift)>=0?SHIFT_L(ptr,shift):SHIFT_R(ptr,-(shift)))


Xaar128 xaar128;

float steps = 0.0;
float fails = 0.0;

byte buf1[8]; //stores pixel data for nozzles 0-63
byte buf2[8]; //stores pixel data for nozzles 64-127

//---------------------------- SETUP -----------------------------------
void setup (void) {   
  Serial.begin(115200);
  Serial.println("Restarted");

  //add robot arm pin setup
  pinMode(ROBOT_SIGNAL_PIN, INPUT);
  xaar128.init();

  //1MHz square wave for Xaar CLK
  TCCR1A = _BV(COM1A0);             
  OCR1A = 7;                         
  TCCR1B = _BV(WGM12) | _BV(CS10);   

  pinMode(scroll_button, INPUT_PULLUP);
  pinMode(select_button, INPUT_PULLUP); 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

//------------------------------ LOOP ------------------------------------
void loop (void) {

  if (powerSwitch){
    if (printer_on){
      xaar128.powerDown();
      Serial.println("Power DOWN");
      printer_on = false;
      menu_id = 0;
    }
    else{
      xaar128.powerUp();
      Serial.println("Power UP");
      printer_on = true;
    }
    powerSwitch = false;
  }
  
  // If the printImage flag is set, do load data-print.
  if (printImage) {
      // 1. Loads pixel data for each line of the image into the printhead buffers (buf1 and buf2).
      //
      // 2. The READY/FIRE alternation is based on the printhead's internal timing:
      //    a. Data is only loaded when the READY signal is inactive.
      //    b. Each line of pixel data is printed once the READY signal switches to active,
      //       indicating that the printhead has completed the current line and is ready for the next.
      //
      // 3. repeats this process for each line until the entire image has been printed.
        print_image(); 
  }

  Serial.println("Done loop");
}

//------------------------- PRINT IMAGE --------------------------------
void print_image(){
  //Get data line by line from hardcoded image  
  for(int row_px= 0; row_px<COLS/2; row_px++){
     for (int i=0; i<8; i++) {
       buf2[i] = pgm_read_byte(&PATTERN2[row_px*2][i]);
       buf1[i] = pgm_read_byte(&PATTERN2[row_px*2+1][i]);
     }

      //Wait robot arm signal
      while(!digitalRead(ROBOT_SIGNAL_PIN)){
        delay(1);
       }
       
      //Wait robot arm signal to print
      if(digitalRead(ROBOT_SIGNAL_PIN)){        
        //Load data to printer through SPI
        XAAR_BEGIN;
        digitalWrite(nSS2, LOW);
        digitalWrite(nSS1, HIGH);
        xaar128.loadBuffer64(buf2);
      
        digitalWrite(nSS2, HIGH);
        digitalWrite(nSS1, LOW);
        xaar128.loadBuffer64(buf1);
      
        digitalWrite(nSS1, HIGH);
        XAAR_END;

        //Fire n_fire times
        for (int i = 0; i<n_fire; i++){
          if (!xaar128.fire()) fails++;
          delayMicroseconds(180);
          steps++;
         }
        
        while(digitalRead(ROBOT_SIGNAL_PIN)) {
          delay(1);
        }
      }
  }

  Serial.print("Failure Rate: ");
  Serial.println(fails/steps);
  Serial.println("Finished printing image");
  fails = 0;
  steps = 0;
  printImage = false;
}

//------------------------------ PRINT LINE ---------------------------------
void print_line(byte data[16]){
  int line_length = 5000;
  
  //load data from print line buffer data_[16]
  for(int k = 0; k < line_length; k++){
     for (int i = 0; i < 8; i++) {
       buf2[i] = data[i];
       buf1[i] = data[i+8];
     }
     
     // 等待机器人信号
     while(!digitalRead(ROBOT_SIGNAL_PIN)) {
       // 检查取消打印
       if ((digitalRead(select_button) == LOW) && (digitalRead(scroll_button) == LOW)){
          printLine = false;
          disp_cancelled();
          row_px = COLS/2;
          delay(1000);
          return;
       }
       delay(1);
     }
     
     // 收到信号后执行打印
     if(digitalRead(ROBOT_SIGNAL_PIN)) {
        //Load data to printer through SPI
        XAAR_BEGIN;
        digitalWrite(nSS2, LOW);
        digitalWrite(nSS1, HIGH);
        xaar128.loadBuffer64(buf2);
      
        digitalWrite(nSS2, HIGH);
        digitalWrite(nSS1, LOW);
        xaar128.loadBuffer64(buf1);
      
        digitalWrite(nSS1, HIGH);
        XAAR_END;
        
        //Fire n_fire times
        for (int i = 0; i < n_fire; i++){
          if (!xaar128.fire()) fails++;
          delayMicroseconds(180);
          steps++;
        }

        // 等待信号变为LOW，防止重复打印
        while(digitalRead(ROBOT_SIGNAL_PIN)) {
          delay(1);
        }
     }
  }

  Serial.print("Failure Rate: ");
  Serial.println(fails/steps);
  Serial.println("Finished printing line");
  fails = 0;
  steps = 0;
  printLine = false;
}
