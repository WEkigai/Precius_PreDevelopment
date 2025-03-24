/********************************************************
Precius Precision Cooktop Proof of concept using Arduino Uno R3
Development sponsored by WEkigai B.V. 
Released under GLPv3. See included license file on Github. Libraries used may have different licenses.

Know more: https://wekigai.eu/precius
Main project page https://github.com/WEkigai/Precius 

Developed by:
Gaurav Sharma
ICF Technologies Pvt. ltd.
contact@icftech.in
25-May-2021

*********************************************************/

/* ----------- Simple explaination ------

This program controls a heater using rotary encoder to set the target temperature and hold time (after reaching target).
The other encoder controls the power level of the heater.
Input temperature is read with a thermocouple.
The heater is controlled using a PID controller. 
Power changes of the heater are achieverd by turning a relay on and off over a duty cycle.
A monochrome 128x64 pixel LCD display is used for UI

--------------------------------------*/

// to debug program
#define DEBUG 0

#include <Encoder.h>
#include <max6675.h>
#include <PID_v1.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <OneButton.h>

// encoder 1 pin assignment
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder1Button A1

// encoder 2 pin assignment
#define encoder2PinA 5
#define encoder2PinB 4

#define thermoDO 8
#define thermoCS  9
#define thermoCLK  10
#define RelayPin 14

// Setup a new OneButton on pin A1.
OneButton enc1button(encoder1Button, true);

// initialize graphic lcd ST7920
//U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 19 /* A5 */ , /* data=*/ 17 /* A3 */, /* CS=*/ 18 /* A4 */, /* reset=*/ U8X8_PIN_NONE);
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ 7, /* reset=*/ 6);
//U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ 7);

// intialize thermocouple
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// encoder 2 pins
Encoder PowerControl(encoder2PinA, encoder2PinB);

//Define Variables we'll be connecting to
double TrueSetpoint, Setpoint, Input, Output;
double Diff;
double last_temp;
double temp_slope;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 120, 0, 20, DIRECT);

unsigned int WindowSize = 1000;
unsigned long windowStartTime;
int sensor_check=0;

// encoder 1 vars
volatile unsigned int encoderPos = 0;       // a counter for the dial
unsigned int lastReportedPos = 1;           // change management
static boolean rotating = false;            // debounce management

// encoder 2 vars
int lastPowerDial = -999;
int PowerDial;
int power_counter=0;
char power_arr[3];

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

// to keep time without delay
unsigned long lastDisplay, loosefocus;
unsigned long revolutionTime = 0;
unsigned long now = 0; unsigned long displayPower, screenTime, tempTime;

// time vars
unsigned long loopTime, previousloopTime,previousMillis=0;
unsigned long int previoussecs = 0, currentsecs = 0, currentMillis = 0;
int timer_started_flag = 0;
signed int total_seconds = -1, total_mins = 0;
int get_mins = 0, get_hrs = 0, speed_enc;
unsigned int interval = 1 ;                // updated every 1 second
char get_mins_arr[3], get_hrs_arr[3];
int frameCounter = 0;
boolean PowerDisplayflag = false;
long oldenctime, vel;

// data field vars
int data_field=0;
char data_field_arr[4];
char hr[3]= {'0','0','\0'}, min[3]={'0','0','\0'}, power_level[] = {'0','0','\0'};
char target_temperature[4]={'0', '4', '0', '\0'}, current_temperature[4]={'1', '0', '0', '\0'};

// focus menu counters
signed int focus_counter =0;
boolean page2_flag = false, page3_flag = false, start_heating= false, check_temp = false, focus_menu = true;
unsigned int clicks=0, x_offset=0;

// function decleration
void doEncoder1A();
void doEncoder1B();
void draw();
void Page1();
void Page2();
void Page3();
void error();
void copy_array(char*, char*, int);
void click1();
void doubleclick1();

void setup() {
    //Serial.begin(9600);  // output
    #ifndef DEBUG
        Serial.begin(9600);  // output
    #endif

    u8g2.begin();
    u8g2.enableUTF8Print();

    pinMode(RelayPin, OUTPUT);

    // set pin mode
    pinMode(encoder1PinA, INPUT);
    pinMode(encoder1PinB, INPUT);
    pinMode(encoder1Button, INPUT);

    // turn on pullup resistors
    digitalWrite(encoder1PinA, HIGH);
    digitalWrite(encoder1PinB, HIGH);
    digitalWrite(encoder1Button, HIGH);

    // set event for encoder 1 button
    enc1button.attachClick(click1);
    enc1button.attachDoubleClick(doubleclick1);

    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doEncoder1A, CHANGE);
    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doEncoder1B, CHANGE);

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_ncenB14_tr);	// choose a suitable font
    u8g2.drawStr(18,36,"Welcome");	// write something to the internal memory
    u8g2.sendBuffer();					// transfer internal memory to the display

    delay(2000);

    windowStartTime = millis();

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

// main loop, work is done by interrupt service routines
// this mainly handles graphic LCD display and menu interaction
void loop() {

    // Loop Timer
    loopTime = millis();

    //read encoder 2 for heater power
    PowerDial = PowerControl.read();

    // ready encoder 1 button
    enc1button.tick();

    // PID start
    if(start_heating) myPID.Compute();

    // if max6675 sensor not connected display error
    //while (isnan(Input) || Input <= 0 ){ error(); }

    if (PowerDial != lastPowerDial) {

        if ( PowerDial - lastPowerDial > 0 ) power_counter++;
        else if ( PowerDial - lastPowerDial < 0 ) power_counter--;

        if (power_counter == 100) power_counter = 0;
        if (power_counter < 0) power_counter = 100;

        String power_field_str = String(power_counter);
        power_field_str.toCharArray(power_level,3);

        lastPowerDial = PowerDial;
        lastDisplay = millis();
    }

    if (loopTime  - screenTime > 100){
       if (!page2_flag && !page3_flag) { // display page 1

            Page1();

        } else if (page2_flag && !page3_flag) { //display page 2

            Page2();
            if(digitalRead(encoder1Button) == LOW) {

                clicks=-1;
                page2_flag = false;
                x_offset = 21;

                //timer_started_flag = 1;
                start_heating = true;

                //initialize the variables we're linked to
                Setpoint = atoi((char *)target_temperature);

                check_temp = true;
                focus_counter = 0;
            }

        } else if (page3_flag) {

            Page3();

        }
        screenTime = millis();
    }

    if (loopTime - tempTime > 1000){


        Input = thermocouple.readCelsius();
        uint8_t read_sensor = int(Input);

        //if(timer_started_flag==0) Input++;
        String temp_sensor_str = String(read_sensor);
        temp_sensor_str.toCharArray(current_temperature,4);

        if (atoi((char *)current_temperature) > 300) start_heating = false;

        //tell the PID to range between 0 and the full window size
        //setting the power to 0.6 of the full power
        if(start_heating) {

            float apparant_power = power_counter / 100.0;
            myPID.SetOutputLimits(0, WindowSize*apparant_power);

        }

        //String curr_temp_str = String(curr_temp);
        //curr_temp_str.toCharArray(current_temperature,4);

        tempTime = millis();

    }

    // 1 Hz task (1000 ms)
    if (loopTime - windowStartTime > WindowSize && start_heating) {
        //process1HzTask();
        //Input = thermocouple.readCelsius();
        //Input = 21;
        // stop heating if temperature is neagtive or not rising
        //sensor_check++;
        //if (sensor_check==60 && start_heating){

        //    temp_slope = (Input - last_temp) / sensor_check;

        //    if (temp_slope == 0) start_heating = false;
        //    if (temp_slope > 0) start_heating = true;
        //    if(Input - last_temp < 0) start_heating = false;

        //    sensor_check = 0;
        //}


        //time to shift the Relay Window
        windowStartTime += WindowSize;

        //Serial.print("C = ");
       // Serial.println(temp_sensor_str);
        //last_temp = Input;
        //Input++;
    }

    /************************************************
     turn the output pin on/off based on pid output
    ************************************************/
    if(start_heating){

        if (Output > loopTime - windowStartTime){
            digitalWrite(RelayPin, HIGH);
             //Serial.print("   Relay On");
        }
        else {
            digitalWrite(RelayPin, LOW);
             //Serial.print("   Relay Off");
        }
    }

    // setpoint reached then start heating
    if ((atoi((char *)target_temperature) <= atoi((char *)current_temperature) && check_temp) ){

        timer_started_flag = 1;
        check_temp = false;
    }

    /************************************************
     target T acquired, start the timer
    ************************************************/
    if(timer_started_flag == 1) {

        currentMillis = millis();

        currentsecs = currentMillis / 1000;

        if ((unsigned long)(currentsecs - previoussecs) >= interval) {

            if (total_seconds == -1){            // when total seconds over

                get_mins = get_mins - 1;

                String get_mins_str = String(get_mins);
                get_mins_str.toCharArray(get_mins_arr,3);
                copy_array(get_mins_arr,min,3 );

            }

            total_seconds++;
        }

        if (total_seconds == 59) { total_seconds = -1; }

        if (get_mins == -1 && get_hrs == 0 && total_seconds == 0 ) {  // mins and hrs counter completed

            timer_started_flag = 0;
            start_heating = false;
            get_mins_arr[0]='0';get_mins_arr[1]='0';get_mins_arr[2]='\0';
            copy_array(get_mins_arr,min,3);
            x_offset = 0;
            focus_counter=0;
            clicks=-1;

        } else if(get_mins == -1 && get_hrs != 0){   //mins counter completed and hrs remaining

            get_hrs = get_hrs -1;
            get_mins = 59;
            String get_hrs_str = String(get_hrs);
            get_hrs_str.toCharArray(get_hrs_arr,3);
            copy_array(get_hrs_arr,hr,3 );
            String get_mins_str = String(get_mins);
            get_mins_str.toCharArray(get_mins_arr,3);
            copy_array(get_mins_arr,min,3 );

        }

        previoussecs = currentsecs;

    }

    /************************************************
     encoder rotates and value changes
    ************************************************/
    if (lastReportedPos != encoderPos) { //encoder roatte?

        if ( encoderPos - lastReportedPos == 1 && clicks == 0) focus_counter++;
        else if ( signed(encoderPos - lastReportedPos) == -1 && clicks == 0) focus_counter--;

        if (focus_counter == 2 || focus_counter == -2) focus_counter=0;

        loosefocus = millis();

        // increment depending upon rotation speed of encoder
        vel = (encoderPos-lastReportedPos) * 1000 /(loosefocus-oldenctime);
        if(focus_counter == 0 ){

            if (vel < 100)
                speed_enc = 5;
            else
                speed_enc = 10;
        } else {

            speed_enc = 1;
        }

        #ifndef DEBUG
          Serial.print("focus counter:");
          Serial.print(focus_counter);
        #endif

        if (clicks == 1 || clicks == 2 ){

            data_field  = data_field + (encoderPos - lastReportedPos)*speed_enc;
            String data_field_str = String(data_field);
            data_field_str.toCharArray(data_field_arr,4);

            if (focus_counter == 0 ) copy_array(data_field_arr,target_temperature,4 );
            if ((focus_counter ==1 ||focus_counter == -1) && clicks == 1)  copy_array(data_field_arr,min,3 );
            if ((focus_counter ==1 ||focus_counter == -1) && clicks == 2)  copy_array(data_field_arr,hr,3 );
        }

        #ifndef DEBUG
          Serial.print(" encoderPos:");
          Serial.println(encoderPos, DEC);
        #endif

        lastReportedPos = encoderPos;
        oldenctime = loosefocus;
    }


    rotating = true;  // reset the debouncer
}

void Page1(){ //Paramters Page

    now = millis();

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_helvB10_tf);
    u8g2.setColorIndex(1);

    if (focus_counter ==0 && clicks == 0) { //focus entry 2

        u8g2.drawFrame(0,22,128,20);

    } else if (focus_counter == 0 && clicks == 1) { //select entry 2

        u8g2.drawLine(102,41,125,41);
        u8g2.drawLine(102,42,125,42);

    } else if ((focus_counter ==1 ||focus_counter == -1) && clicks == 0) { // focus entry 3

        u8g2.drawFrame(0,44,128,20);

    } else if ((focus_counter ==1 ||focus_counter == -1) && clicks == 1) { // select entry 3

        u8g2.drawLine(111 - x_offset, 61, 125 - x_offset, 61);
        u8g2.drawLine(111 - x_offset, 62, 125 - x_offset, 62);

    }  else if ((focus_counter ==1 ||focus_counter == -1) && clicks == 2) { //select entry 3.1

        u8g2.drawLine(90 - x_offset, 61, 104 - x_offset, 61);
        u8g2.drawLine(90 - x_offset, 62, 104 - x_offset, 62);

    }

    if (now - loosefocus > 30000) {
        focus_counter=0;
        clicks=-1;
    }

    // display power if time is less than 3 sec otherwise display current temperature
    if( now - lastDisplay < 6000) {

        u8g2.drawStr( 1, 15, "Power Level");
        u8g2.drawStr( 100, 15, power_level);
        u8g2.drawUTF8(116, 15, "%");

    } else {

        u8g2.drawStr( 1, 15, "Current T");
        u8g2.drawUTF8(70, 15, "°");
        u8g2.drawStr( 75, 15, "C");
        u8g2.drawStr( 103, 15, current_temperature);
        //u8g2.drawUTF8(116, 15, "C");

    }

    // set variable withn certain limit and rollover
    if (atoi((char *)target_temperature) < 0) {

        target_temperature[0] = '2';
        target_temperature[1] = '7';
        target_temperature[2] = '0';

        data_field = atoi((char *)target_temperature);  //convert array to integer
    }
    if (atoi((char *)target_temperature) > 270) {

        target_temperature[0] = '0';
        target_temperature[1] = ' ';
        target_temperature[2] = ' ';

        data_field = atoi((char *)target_temperature);  //convert array to integer
    }
    if (atoi((char *)min) < 0) {

        min[0] = '5';
        min[1] = '9';

        data_field = atoi((char *)min);  //convert array to integer
    }
    if (atoi((char *)min) > 59) {

        min[0] = '0';
        min[1] = ' ';

        data_field = atoi((char *)min);  //convert array to integer
    }
    if (atoi((char *)hr) < 0) {

        hr[0] = '2';
        hr[1] = '4';

        data_field = atoi((char *)hr);  //convert array to integer
    }
    if (atoi((char *)hr) > 24) {

        hr[0] = '0';
        hr[1] = ' ';

        data_field = atoi((char *)hr);  //convert array to integer
    }

    u8g2.drawStr(1,38, "Target T");
    u8g2.drawUTF8(60, 38, "°");
    u8g2.drawUTF8(65, 38, "C");
    u8g2.drawStr( 103, 38, target_temperature);
    u8g2.drawStr(1,60, "Set Time");

    // display hours on the display
    if (atoi((char *)hr) >= 10 && atoi((char *)hr) <= 24)
        u8g2.drawStr( 90 - x_offset, 60, hr);
    else if (atoi((char *)hr) > 0 && atoi((char *)hr) < 10) {
        u8g2.drawStr( 90 - x_offset, 60, "0");
        u8g2.drawStr( 98 - x_offset, 60, hr);
    } else if (atoi((char *)hr) < 0  || atoi((char *)hr) == 0) {
        u8g2.drawStr( 90 - x_offset, 60, "0");
        u8g2.drawStr( 98 - x_offset, 60, "0");
    }

    u8g2.drawStr( 105, 59, ":");

    // display minutes on the display
    if (atoi((char *)min) >= 10 && atoi((char *)min) <= 59)
        u8g2.drawStr( 111 - x_offset, 60, min);
    else if (atoi((char *)min) > 0 && atoi((char *)min) < 10) {
        u8g2.drawStr( 111 - x_offset, 60, "0");
        u8g2.drawStr( 119 - x_offset, 60, min);
    } else if (atoi((char *)min) < 0  || atoi((char *)min) == 0) {
        u8g2.drawStr( 111 - x_offset, 60, "0");
        u8g2.drawStr( 119 - x_offset, 60, "0");
    }

    if (start_heating && timer_started_flag==0){
        u8g2.drawStr( 84, 59, ":");
        u8g2.drawStr( 111, 60, "0");
        u8g2.drawStr( 119, 60, "0");
    }

    // display seconds on the display
    if(timer_started_flag == 1){

        u8g2.drawStr( 84, 59, ":");

        char total_secs_arr[3];
        int time_seconds = 59-total_seconds;
        if (time_seconds == 60) time_seconds = 0;
        String total_secs_str = String(time_seconds);
        total_secs_str.toCharArray(total_secs_arr,3);

        if (atoi((char *)total_secs_arr) >= 10 && atoi((char *)total_secs_arr) <= 59)
            u8g2.drawStr( 111 , 60, total_secs_arr);
        else if (atoi((char *)total_secs_arr) > 0 && atoi((char *)total_secs_arr) < 10) {
            u8g2.drawStr( 111, 60, "0");
            u8g2.drawStr( 119, 60, total_secs_arr);
        } else if (atoi((char *)total_secs_arr) < 0  || atoi((char *)total_secs_arr) == 0) {
            u8g2.drawStr( 111, 60, "0");
            u8g2.drawStr( 119, 60, "0");
        }
    }

    u8g2.sendBuffer();

}

void Page2(){ //Start heating?

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setColorIndex(1);
    u8g2.drawRFrame(5, 20, 120, u8g2.getMaxCharHeight() + 6, 4);
    u8g2.drawStr(10, 36, "Start Heating ?");
    u8g2.sendBuffer();

}

void Page3(){ //Pause/Resume

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setColorIndex(1);
    u8g2.drawRFrame(35, 20, 60, u8g2.getMaxCharHeight() + 6, 4);
    u8g2.drawStr(40, 32, "Resume ?");
    u8g2.drawStr(5, 64, "Paused");
    u8g2.sendBuffer();

}

void error(){ //Sensor Error

    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g_font_unifont);
    u8g2.setColorIndex(1);
    u8g2.drawStr(5, 20, "no thermocouple");
    u8g2.drawStr(18, 37, "attached! or");
    u8g2.drawStr(10, 55, "wrong reading");
    u8g2.sendBuffer();

}

void click1(){ // enc1 single click
    encoderPos = 0;
    lastReportedPos =0;
    clicks++;

    #ifndef DEBUG
      Serial.print("clicks:");
      Serial.println(clicks);
    #endif

    if (focus_counter == 0 && clicks <= 2) {  // get temperature data

        data_field = atoi((char *)target_temperature);  //convert array to integer

        #ifndef DEBUG
            Serial.print("target_temperature- data_field:");
            Serial.println(data_field);
        #endif

        if(clicks ==2) clicks=0;

    } else if ( (focus_counter ==1 || focus_counter == -1) && clicks == 1) {  // get min data

        data_field = atoi((char *)min);  //convert array to integer
        #ifndef DEBUG
            Serial.print("target_temperature- min:");
            Serial.println(data_field);
        #endif

    } else if ( (focus_counter ==1 ||focus_counter == -1) && clicks == 2) {  // get hr data

        data_field = atoi((char *)hr);    //convert array to integer
        #ifndef DEBUG
            Serial.print("target_temperature- hr:");
            Serial.println(data_field);
        #endif

    }

    if(clicks == 3 ) {    // start heating

        if(atoi((char *)hr) == 0 && atoi((char *)min) == 0){  // hour and min data field is zero
            focus_counter=0;
            clicks=0;
        } else if (atoi((char *)hr) != 0 || atoi((char *)min) != 0) {
            page2_flag = true;
            total_seconds = -1;
            get_mins = atoi((char *)min);
            get_hrs = atoi((char *)hr);
        }
    }
 }

void doubleclick1(){ // enc1 double click

    if(start_heating && !page3_flag) {

        page3_flag = true;
        start_heating = false;
        digitalWrite(RelayPin, LOW);

    } else if (!start_heating && page3_flag) {

        page3_flag = false;
        start_heating = true;
    }
}

// Interrupt on A changing state
void doEncoder1A() {
    // debounce
    if ( rotating ) delay (1);  // wait a little until the bouncing is done

    // Test transition, did things really change?
    if ( digitalRead(encoder1PinA) != A_set ) { // debounce once more
        A_set = !A_set;

        // adjust counter + if A leads B
        if ( A_set && !B_set )
            encoderPos += 1;

        rotating = false;  // no more debouncing until loop() hits again
    }

}

// Interrupt on B changing state, same as A above
void doEncoder1B() {

  if ( rotating ) delay (1);

  if ( digitalRead(encoder1PinB) != B_set ) {
      B_set = !B_set;
      //  adjust counter - 1 if B leads A
      if ( B_set && !A_set )
        encoderPos -= 1;

      rotating = false;
  }
}

// copy array from source to destination
void copy_array(char* src, char* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
}
