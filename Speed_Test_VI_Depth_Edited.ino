
/* 
 * This example shows how to control MDDS30 (SmartDriveDuo-30) in PWM mode with Arduino.
 * Set MDDS30 input mode to 0b10110100
 * Note: This example also compatible with MDDS10 and MDDS60
 *
 * This is utilized with a KY-003 Hall sensor, KY-004 Rotary Encoder, HC-SR04 Ultrasonic Sensor, DC Motor, 20 Neodymium Magnets, and Arduino Uno
 *
 *
 * Hardware Connection:
 *   Arduino Uno    MDDS30
 *   GND ---------- GND
 *   4 ------------ IN1
 *   5 ------------ AN1
 *   6 ------------ AN2
 *   7 ------------ IN2
 */

#include <Cytron_SmartDriveDuo.h>
#define IN1 4 // Arduino pin 4 is connected to MDDS30 pin IN1.
#define AN1 5 // Arduino pin 5 is connected to MDDS30 pin AN1.
#define AN2 6 // Arduino pin 6 is connected to MDDS30 pin AN2.
#define IN2 7 // Arduino pin 7 is connected to MDDS30 pin IN2.
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, IN1, IN2, AN1, AN2);

#define SWITCH 3
int Echo=A4;
int Trig=A5;

byte switch0, switch1;

float r= 10.16; // cm
float y;
float dis= 0.00; //cm 
const float pi=3.1415926535897932384626433832795;
float lake;
float delta;
float theta;
float clockwise_delta=-delta;
float clockwise_theta=-theta;

float x;

const float time_buffer= 10; // This should not change.

const byte PulsesPerRevolution = 20;  // Set how many pulses there are on each revolution. Default: 2.


// If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
// last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
// If the period is above this value, the RPM will show as 0.
// The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
// at very low RPM.
// Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
// The unit is in microseconds.
const unsigned long ZeroTimeout = 300000;  // For high response time, a good value would be 100000.
                                           // For reading very low RPM, a good value would be 300000.


// Calibration for smoothing RPM:
const byte numReadings = 2;  // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                             // react slower to changes. 1 = no smoothing. Default: 2.





/////////////
// Variables:
/////////////
volatile unsigned long ZaWarudo;
volatile unsigned long TimeStop;
volatile unsigned long LastTimeWeMeasured;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
volatile unsigned long PeriodAverage = ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long FrequencyRaw;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal;  // Frequency without decimals.
unsigned long RPM;  // Raw RPM without any processing.
unsigned int PulseCounter = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

unsigned long PeriodSum; // Stores the summation of all the periods to do the average.

unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;  // Stores the last time we measure a pulse in that cycle.
                                    // We need a variable with a value that is not going to be affected by the interrupt
                                    // because we are going to do math and functions that are going to mess up if the values
                                    // changes in the middle of the cycle.
unsigned long CurrentMicros = micros();  // Stores the micros in that cycle.
                                         // We need a variable with a value that is not going to be affected by the interrupt
                                         // because we are going to do math and functions that are going to mess up if the values
                                         // changes in the middle of the cycle.
volatile unsigned long top;
volatile unsigned long bottom;
float d; // current destination point (in centimeters)
float dold; //previous destination point (in centimeters)

// We get the RPM by measuring the time between 2 or more pulses so the following will set how many pulses to
// take before calculating the RPM. 1 would be the minimum giving a result every pulse, which would feel very responsive
// even at very low speeds but also is going to be less accurate at higher speeds.
// With a value around 10 you will get a very accurate result at high speeds, but readings at lower speeds are going to be
// farther from eachother making it less "real time" at those speeds.
// There's a function that will set the value depending on the speed so this is done automatically.
unsigned int AmountOfReadings = 1;

unsigned int ZeroDebouncingExtra;  // Stores the extra value added to the ZeroTimeout to debounce it.
                                   // The ZeroTimeout needs debouncing so when the value is close to the threshold it
                                   // doesn't jump from 0 to the value. This extra value changes the threshold a little
                                   // when we show a 0.

// Variables for smoothing tachometer:
unsigned long readings[numReadings];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.

float Height;
float Length;

/////////////////////////////////////////////
/////////////////////////////////////////////
/// Variables that Are Subject to Change////
/////////////////////////////////////////////
/////////////////////////////////////////////

const float mass=0.00; //(Kilograms) This is the mass of the sonde being deployed
const int safezone=1; // (Centimeters) If the sonde and the desired distance is within this safezone, the sonde will stop
const int starting_distance= 100; // (Centimeters) The system will not start until the sensor senses that the floor bottom is at least this distance
const float d_original= 50.00; //This is unchanged, throughout the entire process (in centimeters)
const float threshold= 35.00; // This is unchanged, the minimum distance desired from the sonde and the lake floor (in centimeters) 

const float time_threshold= 4000; // This is 4 seconds since the system will be read in milliseconds. 
//This is the time that the sonde is still until the next desired distance is subscribed and resets the process

///////////////////////////////////////
//////////////////////////////////////
/////////////////////////////////////

float alpha= -0.0001*pow(safezone,4) +0.0054*pow(safezone,3) -0.0552*pow(safezone,2) +0.13477*safezone +1.8959; //This cannot change, this is a damping equation to assign safezone to Length
float psi= -0.0018*pow(mass,5) +0.0241*pow(mass,4) -0.1077*pow(mass,3)+0.1796*pow(mass,2) -0.0697*mass +1.019; //This cannot change, this is a damping equation to assign mass to Length


//Ultrasonic distance measurement Sub function
int getDistance() {
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    return (int)pulseIn(Echo, HIGH) / 58;
}

char inChar;
float speedLeft, speedRight=0;

void Up(){
  digitalWrite(IN1, HIGH);
  Serial.println("Going UP");
}

void Down(){
  digitalWrite(IN1, HIGH);
  Serial.println("Going DOWN");
}

void stop() {
  digitalWrite(IN1, LOW);
  analogWrite(AN1, 0);
  Serial.println("The sonde is stationary");
}




void setup()
{ delay(8000);  // We sometimes take several readings of the period to average. Since we don't have any readings
                // stored we need a high enough value in micros() so if divided is not going to give negative values.
                // The delay allows the micros() to be high enough for the first few cycles.
  Serial.begin(9600);
  digitalWrite(IN1, LOW);
  analogWrite(AN1, 0);  
  pinMode(13, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(AN1, OUTPUT);
  pinMode(SWITCH, INPUT_PULLUP);
  switch0 =digitalRead(SWITCH);
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING);  // Enable interruption pin 2 when going from LOW to HIGH.
  Height=0.00;
  Length=0.00;
  d=d_original; //This sets the initial distance (in centimeters)
  dold=0.00; //cm
  

}


void loop()
{ 
/////////////////////////////////
//// DEFINING THE KILL-SWITCH///
///////////////////////////////
  switch1= digitalRead(SWITCH);
  if ((switch0 == 1) && (switch1 == 0)) {
    Serial.println ("Error in Code! Must Go Back Down");
    speedLeft=0;
    stop();
    Down();
    speedLeft=-(80-80*exp(-0.01*y));
    Length=(100000000000*Length+alpha*psi*r*theta*(PeriodBetweenPulses))/100000000000; //centimeters 
  }
///////////////////////////////
//////////////////////////////
/////////////////////////////

  LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
  CurrentMicros = micros();  // Store the micros() in a variable.
  dis=getDistance(); //centimeters




  float lake=map(dis,0,80,0,579.12);

  // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
  // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
  // LastTimeCycleMeasure I set it as the CurrentMicros.
  // The need of fixing this is that we later use this information to see if pulses stopped.
  if(CurrentMicros < LastTimeCycleMeasure)
  {
    LastTimeCycleMeasure = CurrentMicros;
  }

  // Calculate the frequency:
  FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.
 // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
  if(PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
  {  // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw = 0;  // Set frequency as 0.
    ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }





  FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                          // This is not used to calculate RPM but we remove the decimals just in case
                                          // you want to print it.





  // Calculate the RPM:
  RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                                  // 60 seconds to get minutes.
  //RPM = RPM / 10000;  // Remove the decimals.





  // Smoothing RPM:
  total = total - readings[readIndex];  // Advance to the next position in the array.
  readings[readIndex] = RPM;  // Takes the value that we are going to smooth.
  total = total + readings[readIndex];  // Add the reading to the total.
  readIndex = readIndex + 1;  // Advance to the next position in the array.

  if (readIndex >= numReadings)  // If we're at the end of the array:
  {
    readIndex = 0;  // Reset array index.
  }
  
  // Calculate the average:
  average = total / numReadings;  // The average value it's the smoothed result.

//Calculate Angular Velocity
float theta= pi*average/3;


float y=lake-Length;
float x= d-Length;
//float x= d-Height; //centimeters
  // Print information on the serial monitor:
  // Comment this section if you have a display and you don't need to monitor the values on the serial monitor.
  // This is because disabling this section would make the loop run faster.
//  Serial.print("Period (us): ");
//  Serial.print(PeriodBetweenPulses);
  /*Serial.print("\tReadings: ");
  Serial.print(AmountOfReadings);
  Serial.print("\tFrequency: ");
  Serial.print(FrequencyReal);
  */
 // Serial.print("\tRPM: ");
  //Serial.println(RPM/10000);


// Serial.print("\tZaWarudo");
 //Serial.print(ZaWarudo);
   /* Serial.print("Height (cm):");
    Serial.print("\t");
    Serial.println(Height);  */ 

if ((lake<starting_distance) && (Length<=0)) {  //This section is designated when the boat is leaving the shore or being carried and is still operational
  stop();
  speedLeft=0;
  Serial.print("\tHeight:");
  Serial.println(Length); //centimeters

  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);

  Serial.print("\tDistance of Lake (cm)");
  Serial.println(lake); //centimeters
}

else if ((lake>starting_distance) && (lake-d > threshold) && (x>safezone) && (d>dold) ) { //functions that control the Down features
  Down();
  speedLeft=-(50-50*exp(-0.075*x));
 // Height=(200000000000*Height+r*delta*(PeriodBetweenPulses))/200000000000; //centimeters
  Length=(100000000000*Length+alpha*psi*r*theta*(PeriodBetweenPulses))/100000000000; //centimeters

if ( (speedLeft>-10) &&  (RPM=0) ){
  speedLeft=-25;
}
  Serial.print("\tPower Percentage:");
  Serial.println(speedLeft);

  Serial.print("\tDistance (cm):");
  Serial.println(d);

  Serial.print("\tDiff. of Distance and Sonde (cm):");
  Serial.println(x);
  
  Serial.print("\tRPM: ");
  Serial.println(RPM);

  Serial.print("\tTachometer: ");
  Serial.println(average);
 
  Serial.print("\tPeriod:");
  Serial.println(PeriodBetweenPulses);
    
  Serial.print("\tHeight:");
  Serial.println(Length);    
  
  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);
    
  Serial.print("\tDistance of Lake (cm): ");
  Serial.println(lake);

  //Serial.print("\tTimeStop:");
  //Serial.println(TimeStop);
}

else if ((lake>starting_distance) && (lake-d >threshold) && (x<-safezone) && (dold>d) ) { //functions that controls the Up feature
  Up();
  speedLeft=(50-50*exp(0.075*x));
  clockwise_theta= -theta;
if ( (speedLeft>-10) &&  (RPM=0) ){
  speedLeft=25;
}
  
  Length=(100000000000*Length+alpha*psi*r*clockwise_theta*(PeriodBetweenPulses))/100000000000; //centimeters

  Serial.print("\tPower Percentage:");
  Serial.println(speedLeft);

  Serial.print("\tDistance (cm):");
  Serial.println(d);

  Serial.print("\tDiff. of Distance and Sonde (cm):");
  Serial.println(x);
  
  Serial.print("\tRPM: ");
  Serial.println(RPM);

  Serial.print("\tTachometer: ");
  Serial.println(average);
    
  Serial.print("\tPeriod:");
  Serial.println(PeriodBetweenPulses);
  
  Serial.print("\tHeight:");
  Serial.println(Length);    

  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);
  
  Serial.print("\tDistance of Lake (cm): ");
  Serial.println(lake);

  //Serial.print("\tTimeStop:");
  //Serial.println(TimeStop);
  
}

else if ( (x<10) && (x>safezone) && (dold>d)  ) {
  Down();
  speedLeft=-25;
  Length=(100000000000*Length+alpha*psi*r*clockwise_theta*(PeriodBetweenPulses))/100000000000; //centimeters

  Serial.print("\tPower Percentage:");
  Serial.println(speedLeft);

  Serial.print("\tDistance (cm):");
  Serial.println(d);

  Serial.print("\tDiff. of Distance and Sonde (cm):");
  Serial.println(x);
  
  Serial.print("\tRPM: ");
  Serial.println(RPM);

  Serial.print("\tTachometer: ");
  Serial.println(average);
    
  Serial.print("\tPeriod:");
  Serial.println(PeriodBetweenPulses);
  
  Serial.print("\tHeight:");
  Serial.println(Length);    

  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);
  
  Serial.print("\tDistance of Lake (cm): ");
  Serial.println(lake);
}

else if ( (x>-10) && (x<-safezone) && (d>dold) ) {
  Up();
  speedLeft=25;
  clockwise_theta= -theta;
  Length=(100000000000*Length+alpha*psi*r*clockwise_theta*(PeriodBetweenPulses))/100000000000; //centimeters

  Serial.print("\tPower Percentage:");
  Serial.println(speedLeft);

  Serial.print("\tDistance (cm):");
  Serial.println(d);

  Serial.print("\tDiff. of Distance and Sonde (cm):");
  Serial.println(x);
  
  Serial.print("\tRPM: ");
  Serial.println(RPM);

  Serial.print("\tTachometer: ");
  Serial.println(average);
    
  Serial.print("\tPeriod:");
  Serial.println(PeriodBetweenPulses);
  
  Serial.print("\tHeight:");
  Serial.println(Length);    

  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);
  
  Serial.print("\tDistance of Lake (cm): ");
  Serial.println(lake);
}

else if ( (abs(x) <safezone) && (TimeStop < time_threshold -100)) { //tells the sonde to stop if within the safezone
  Length=d; //this is for simplification and to not break out of this loop
  speedLeft=0;
  Serial.print("\tTime Remaining Until Next Move");
  Serial.println((time_threshold + time_buffer-TimeStop)/1000);
  TimeStop= millis()-ZaWarudo;
}

else if ( (TimeStop>=time_threshold-100)&&(TimeStop<time_threshold + time_buffer)&&(lake-d> (threshold+d_original)) && (dold <d) &&(abs(x)<safezone) ) { //ensures system continues to go down
dold=d;
d+=d_original;
TimeStop=0;

}

else if ( (TimeStop>=time_threshold-100)&&(TimeStop<time_threshold + time_buffer)&&(lake-d<(threshold+d_original))&&(abs(x)<safezone) ){ //restarts system if it is close to the bottom
  dold=d;
  d-=d_original;
  TimeStop=0;
}

else if ( (TimeStop>=time_threshold-100)&&(TimeStop<time_threshold + time_buffer)&&(lake>starting_distance)&&(dold >d)&&(d !=d_original)&&(abs(x)<safezone) ){ //system continues to go up
  dold=d;
  d-=d_original;
  TimeStop=0;
}

else if ( (TimeStop>=time_threshold-100)&&(TimeStop<time_threshold + time_buffer)&&(lake>starting_distance)&&(d=d_original)&&(dold >d) &&(abs(x)<safezone) ){ // restart if d_original is hit
dold=d;
d+=d_original;
TimeStop=0;
}



else if (Length <0) { //in the case that the sonde is above water
  delay(3000);
  Down();
  speedLeft=-(50-50*exp(-0.075*x));
  Length=(100000000000*Length+alpha*psi*r*theta*(PeriodBetweenPulses))/100000000000; //centimeters
  Serial.print("\tHeight:");
  Serial.println(Length);  
}

else if ( (Length> 10) && (lake<starting_distance) ) { //in the case the boat is approaching the shore and the sonde is still in the water
  d=0;
  Up();
  speedLeft=(50-50*exp(0.075*x));
  float clockwise_theta=-theta;
  Length=(100000000000*Length+alpha*psi*r*clockwise_theta*(PeriodBetweenPulses))/100000000000; //centimeters
  Serial.print("\tPower Percentage:");
  Serial.println(speedLeft);

  Serial.print("\tDistance (cm):");
  Serial.println(d);

  Serial.print("\tDiff. of Distance and Sonde (cm):");
  Serial.println(x);
  
  Serial.print("\tRPM: ");
  Serial.println(RPM);

  Serial.print("\tTachometer: ");
  Serial.println(average);
 
  Serial.print("Period");
  Serial.println(PeriodBetweenPulses);
    
  Serial.print("\tHeight:");
  Serial.println(Length);    

  Serial.print("\tDistance of Sensor (cm):");
  Serial.println(dis);
  
  Serial.print("\tDistance of Lake (cm): ");
  Serial.println(lake);

  if ( x > -10) {
    stop();
    speedLeft=0;
  }
}

    smartDriveDuo30.control(speedLeft, speedRight);
  //} */
}

void Pulse_Event()  // The interrupt runs this to calculate the period between pulses:
{

  PeriodBetweenPulses = micros() - LastTimeWeMeasured;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.

  LastTimeWeMeasured = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

  ZaWarudo= millis();

  if(PulseCounter >= AmountOfReadings)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage = PeriodSum / AmountOfReadings;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.


    // Change the amount of readings depending on the period between pulses.
    // To be very responsive, ideally we should read every pulse. The problem is that at higher speeds the period gets
    // too low decreasing the accuracy. To get more accurate readings at higher speeds we should get multiple pulses and
    // average the period, but if we do that at lower speeds then we would have readings too far apart (laggy or sluggish).
    // To have both advantages at different speeds, we will change the amount of readings depending on the period between pulses.
    // Remap period to the amount of readings:
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings = RemapedAmountOfReadings;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter++;  // Increase the counter for amount of readings by 1.
    PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
  }


}
