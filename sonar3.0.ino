// ---------------------------------------------------------
// This example code was used to successfully communicate
// with 15 ultrasonic sensors. You can adjust the number of
// sensors in your project by changing sonarNum and the
// number of NewPing objects in the "sonar" array. You also
// need to change the pins for each sensor for the NewPing
// objects. Each sensor is pinged at 33ms intervals. So, one
// cycle of all sensors takes 495ms (33 * 15 = 495ms). The
// results are sent to the "oneSensorCycle" function which
// currently just displays the distance data. Your project
// would normally process the sensor results in this
// function (for example, decide if a robot needs to turn
// and call the turn function). Keep in mind this example is
// event-driven. Your complete sketch needs to be written so
// there's no "delay" commands and the loop() cycles at
// faster than a 33ms rate. If other processes take longer
// than 33ms, you'll need to increase pingInt so it
// doesn't get behind.
//EDITED: Jacob Kolb 9/9/2016
//NewPing(triggerPin,EchoPin,maxDist)
//trigger pin is digital
//echo pin is PWM
// ---------------------------------------------------------
#include <NewPing.h>
const int analogOutPin = 10;
int sensorValue = 0;        
int outputValue = 0;
//int buzzer=0;
unsigned int maxFq = 6000;//65535;
unsigned int minFq = 300;//31; 

const int sonarNum = 1; // Number or sensors.
int maxDist = 400;// Max distance in cm.
int pingInt = 33; // Milliseconds between pings.

int trig1=12;
int echo1=11;
int trig2=9;
int echo2=6;

int avg=0;
 
unsigned long pingTimer[sonarNum]; // When each pings.
unsigned int cm[sonarNum]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.
 
NewPing sonar[sonarNum] = { // Sensor object array.
  NewPing(trig1,echo1, maxDist),
//  NewPing(trig2,echo2, maxDist),
//  NewPing(45, 20, maxDist),
//  NewPing(21, 22, maxDist),
//  NewPing(23, 24, maxDist),
//  NewPing(25, 26, maxDist),
//  NewPing(27, 28, maxDist),
//  NewPing(29, 30, maxDist),
//  NewPing(31, 32, maxDist),
//  NewPing(34, 33, maxDist),
//  NewPing(35, 36, maxDist),
//  NewPing(37, 38, maxDist),
//  NewPing(39, 40, maxDist),
//  NewPing(50, 51, maxDist),
//  NewPing(52, 53, maxDist)
};
 
void setup() {
  //Serial.begin(57600);
  pingTimer[0] = millis() + 33; // First ping start in ms.
  for (uint8_t i = 1; i < sonarNum; i++)
    pingTimer[i] = pingTimer[i - 1] + pingInt;
    
}
 
void loop() {
  for (uint8_t i = 0; i < sonarNum; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += pingInt * sonarNum;
      if (i == 0 && currentSensor == sonarNum - 1)
        oneSensorCycle(); // Do something with results.
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
  // The rest of your code would go here.
}
 
void echoCheck() { // If ping echo, set distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
void oneSensorCycle() { // Do something with the results.
//  for (uint8_t i = 0; i < sonarNum; i++) {
////    Serial.print(i);
////    Serial.print("=");
////    Serial.print(cm[i]);
////    Serial.println("cm ");
//    avg=(cm[i]+avg);
//  }
  Serial.print("Average Distance "); 
  outputValue = cm[1];   
  Serial.println(outputValue);
  outputValue=map(outputValue,0,400,minFq,maxFq);
  outputValue = (-1)*(outputValue-maxFq);
  tone(analogOutPin, outputValue);
  Serial.println(outputValue);
  avg=0;
  
}

