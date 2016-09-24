#include <NewPing.h>
 
#define TRIGGER_PIN  5
#define ECHO_PIN     6
#define MAX_DISTANCE 500

const int pinOut = 13;
unsigned int maxFq = 1500;//65535
unsigned int minFq = 350; 
int dist = 0;
int fqNow;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
void setup() {
  Serial.begin(115200);
}
 
void loop()
{
  int uS = sonar.ping();
  int cm = abs(uS/ US_ROUNDTRIP_CM);
  if (cm >= MAX_DISTANCE || cm==0)
  {
    Serial.print("Ping: ");
    Serial.print(cm);
    Serial.println("cm");
    dist=MAX_DISTANCE;
    fqNow=map(dist,1,MAX_DISTANCE,minFq,maxFq);
    fqNow= (-1)*(fqNow-maxFq);
    tone(pinOut,fqNow);
    Serial.print("Frequency is ");
    Serial.print(fqNow);
    Serial.println("Hz");
  }
  else{
  Serial.print("Ping: ");
  Serial.print(cm);
  Serial.println("cm");
  dist=cm;
  fqNow=map(dist,1,MAX_DISTANCE,minFq,maxFq);
  fqNow= (-1)*(fqNow-maxFq);
  tone(pinOut,fqNow);
  Serial.print("Frequency is ");
  Serial.print(fqNow);
  Serial.println("Hz");
  }
}
