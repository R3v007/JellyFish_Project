//sol pins are where the PWM value is sent and the direc pins control the direction
//solenoid 1
const int sol1 = 3;
const int direc1= 4;

//solenoid 2
const int sol2= 11;
const int direc2= 12;

//solenoid 3
const int sol3=5;
const int direc3 = 8;

//solenoid 4
const int sol4 = 6;
const int direc4=7;

void setup(){
  for(int i=3;i<9;i++){
    pinMode(i,OUTPUT);
  } 

  for(int i=11;i<13;i++){
  pinMode(i,OUTPUT);
  }
}

void loop(){
  for(int i=0; i<255; i=i+10){
    digitalWrite(direc4, LOW);
    analogWrite(sol4, i);
    delay(1000);
  }
  delay(2000);
}