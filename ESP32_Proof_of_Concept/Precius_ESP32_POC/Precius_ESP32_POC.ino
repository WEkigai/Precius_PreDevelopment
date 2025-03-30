/*------------------------------------
Precius Precision Cooktop Proof of concept using ESP32
Developed by WEkigai B.V. [2024-2025]
Released under GLPv3. See included license file on Github. Libraries used may have different licenses.

Know more: https://wekigai.eu/precius
Main project page https://github.com/WEkigai/Precius 

------------------------------------------*/


/* -----Simple explaination -------
The code controls a heater based on the measured temperature from an NTC sensor.
The output of the heater is varied using the output of a PID controller, using on-off relay to vary the power.
Up/Down buttons increase/decrease setpoints of temperatures or total power
Mode button cycles between idle, manual and auto modes
Power mode button toggles the input of up/down buttons to temperature of power variables

Power level is used to compensate for different thermal mass of different cooking pans and minimizing overshoots
----------------------------*/


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <ArduPID.h>
#include <Arduino.h>
#include <Button2.h>

//define all the hardware pinouts

///inputs

// Temperature measurement NTC
#define NTCpin 34

// buttons and other input controls
#define modebuttonpin 13
Button2 modebutton;

#define pow_modebuttonpin 4
Button2 pow_modebutton;

#define upbuttonpin 14
Button2 upbutton(upbuttonpin,true);

#define downbuttonpin 12
Button2 downbutton(downbuttonpin,true);

/// outputs

// Heater output realy (using LED pin for easy debug)
#define RelayPin 2

//buzzer
#define BuzzerPin 18


/// display
//set LCD to pins 21 (SDA) and 22(SCL)
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//time variables
unsigned long int looptime, prev_buzzer_time=0;

// Temperature parameters
double Tset=71.0, Tnow=-1.0;

// PID parameters
double PID_output=0.0;

//power parameters
float power_percent=50.0;

/* NTC parameters*/

float Vref=3.3;     // Reference voltage [V]        
float analog_levels=4095.0; //12 bit resolution of ESP32 analog read
float R_ref=10000;    // Reference Resistor t [ohm] 
float R_0=100000;    // value of rct in T_0 [ohm]
float T_0=298.15;   // use T_0 in Kelvin [K]
float Vout=0.0;    // Vout in A0 
float Rout=0.0;    // Rout in A0
// use the datasheet to get this data.
float beta=3990;    // initial parameters [K]
float TempK=0.0;   // variable output


// define PID
ArduPID myPID;

// Parameters for converting PID output to PWM
unsigned int duty_windowSize = 5000; //total duty cycle of 5 seconds
unsigned long duty_windowStartTime; //start time of each duty cycle window
unsigned int duty_cycle = 0; //number between 0 and duty_windowSize that defines duty cycle

//Parameters to refresh display
unsigned int display_refresh_period = 500; //milliseconds between display updates
unsigned long display_refresh_StartTime; //start time of each refresh cycle

//display refresh flag
static boolean flag_display_refresh=true;

//states, idle is default
static boolean flag_idle=true, flag_manual=false, flag_auto=false, flag_settings=false;

//state for power mode or not
static boolean flag_pow_mode=false;

//buzzer sound flag
static boolean flag_buzzer=false;


/// variables to handle all menus and clicks
int modeclicks=0;

//define state functions
void do_idle();
void do_manual();
void do_auto();
void do_settings();

//function to handle display
void do_display();

//function to handle heater
void do_heater();

//function handle buzzer
void do_buzzer();

// functions to handle input and output
void do_inputcheck();
void do_readtemp();
void modebuttonclick(Button2& btn);
void upbuttonclik(Button2& btn);
void downbuttonclick(Button2& btn);
void pow_modebuttonclicik(Button2& btn);


void setup() {

Serial.begin(115200);

//setup all hardware
modebutton.begin(modebuttonpin,INPUT_PULLUP,true);
upbutton.begin(upbuttonpin,INPUT_PULLUP,true);
downbutton.begin(downbuttonpin,INPUT_PULLUP,true);
pow_modebutton.begin(pow_modebuttonpin, INPUT_PULLUP,true);

modebutton.setClickHandler(modebuttonclick);
upbutton.setClickHandler(upbuttonclik);
downbutton.setClickHandler(downbuttonclick);
pow_modebutton.setClickHandler(pow_modebuttonclicik);

pinMode(RelayPin, OUTPUT);
pinMode(BuzzerPin, OUTPUT);
pinMode(NTCpin, INPUT);

// setup PID duty cycle window start time
duty_windowStartTime = millis();

//setup display refresh period
display_refresh_StartTime = millis();


//setup pid
myPID.begin(&Tnow, &PID_output, &Tset, 100, 1.0, 10.0);

myPID.setOutputLimits(0, duty_windowSize*power_percent*0.01);
myPID.setWindUpLimits(0,1000);
myPID.start();

// start LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Welcome!");
  delay(2000);
  lcd.clear();
}

void loop() {

looptime=millis();

// check input states (buttons and dials)
//do_inputcheck(); //checking buttons in a function did not seem to register button presses. To be debugged in future.
  modebutton.loop();
  upbutton.loop();
  downbutton.loop();
  pow_modebutton.loop();


// read temperatures
do_readtemp();

// state machine
if (flag_idle)do_idle();
if (flag_manual) do_manual();
if (flag_auto) do_auto();
if (flag_settings) do_settings();


//check and update all timers

//check relay window time
if (looptime - duty_windowStartTime > duty_windowSize) {
//time to shift the Relay Window
duty_windowStartTime += duty_windowSize;
 }

 if(looptime-display_refresh_StartTime > display_refresh_period){
  display_refresh_StartTime+=display_refresh_period;
  flag_display_refresh=true;
 }

//update display
do_display();


}

//Functions

void do_idle() {
  duty_cycle=0; //turn off heating
}



void do_manual() { //system in manual control state
duty_cycle = duty_windowSize*power_percent*0.01;
do_heater();
}



void do_auto() { // system in automatic control state
//perform PID calculations
myPID.setOutputLimits(0, duty_windowSize*power_percent*0.01);
myPID.setWindUpLimits(0,2*Tset);//Seeting windup limits based on target temperature. Higher setpoints need higher I term to stabilize temperature due to ambient losses
myPID.compute();
duty_cycle=PID_output;
//Serial.println(PID_output);
//delay(100);
do_heater();
do_buzzer();
}


void do_settings(){}


void do_display() {
  //if it is time to update display update it and set the refresh flag back to false
  if (flag_display_refresh)
  {
 // Serial.println(Tnow); //Using display refresh also for serial output of PID parameters

  myPID.debug(&Serial, "myPID", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);

    //first clear display
    lcd.clear();
    //display different things according to current mode
    if(flag_manual || flag_auto || flag_idle || flag_settings)
      {
        //display state
        lcd.setCursor(0,0);
        if(flag_manual)lcd.print("Manual");
        if(flag_auto)lcd.print("Auto");
        if(flag_idle)lcd.print("Idle");
        if(flag_settings)lcd.print("Settings");
      }
     if(flag_manual || flag_auto || flag_idle)
      {
      //display current temp
        lcd.setCursor(8,0);
        lcd.print("Tset ");
        lcd.setCursor(13,0);
        lcd.printf("%d",int(Tset));
        lcd.setCursor(8,1);
        lcd.print("Tnow ");
        lcd.setCursor(13,1);
        lcd.printf("%d",int(Tnow));
        lcd.setCursor(0,1);
        lcd.printf("%d",int(power_percent));
        if(flag_pow_mode){
          lcd.setCursor(4, 1);
          lcd.print("Change");
        }
      } 
     if(flag_auto)
      {
      //display target temp
      }
    
     if(flag_manual || flag_auto )
      {
      //display power
      }
    
     if(flag_settings )
      {
    //display settings
      }
    
  }
  flag_display_refresh=false; //done with refresh so we do not have to refresh again until it is time
}


void do_inputcheck(){

    // keep watching the push buttons:
  modebutton.loop();
  upbutton.loop();
  downbutton.loop();
  pow_modebutton.loop();
}

void modebuttonclick(Button2& btn) {
  modeclicks++;
  Serial.println(modeclicks);

  if(modeclicks==0){
    flag_idle=true;
    flag_manual=false;
    flag_auto=false;
    flag_settings=false;
  }

    if(modeclicks==1){
    flag_idle=false;
    flag_manual=true;
    flag_auto=false;
    flag_settings=false;
  }

    if(modeclicks==2){
    flag_idle=false;
    flag_manual=false;
    flag_auto=true;
    flag_settings=false;
  }
  
    if(modeclicks==3){
    flag_idle=false;
    flag_manual=false;
    flag_auto=false;
    flag_settings=true;
  }

    if(modeclicks>3){
      modeclicks=0;
    flag_idle=true;
    flag_manual=false;
    flag_auto=false;
    flag_settings=false;
  }

}

void upbuttonclik(Button2& btn){
  Serial.println("upclick");
  if(flag_pow_mode) {
    power_percent++;
      if (power_percent>100)power_percent=100;
  }
  else
  { Tset=Tset+1;
  if (Tset>250)Tset=250;
  }
}

void downbuttonclick(Button2& btn){
  Serial.println("downclick");
  if(flag_pow_mode) {
    power_percent--;
      if (power_percent<0)power_percent=0;
  }
  else
  { Tset=Tset-1;
  if (Tset<10)Tset=10;
  }

}

void pow_modebuttonclicik(Button2& btn){
flag_pow_mode=!flag_pow_mode;
}


void do_readtemp(){

Vout=float(analogReadMilliVolts(NTCpin))/1000.0;
Rout=(Vout*R_ref)/(Vref-Vout);
TempK=1.0/(((log(Rout/R_0))/beta)+(1/(T_0)));

Tnow=TempK-273.15;

}


void do_heater(){
      if (duty_cycle > looptime - duty_windowStartTime){
            digitalWrite(RelayPin, HIGH);
           // Serial.println("   Relay On");
        }
        else {
            digitalWrite(RelayPin, LOW);
            //Serial.println("   Relay Off");
        }
}

void do_buzzer(){
   
}