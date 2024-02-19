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

//-----------------------------------------
//MOTION FUNCTIONS
//-----------------------------------------

//UPWARD FUNCTIONS
void fin1_up(int val){
  //for fin1 up => direction = LOW
  digitalWrite(direc1, LOW);
  analogWrite(sol1, val);
  //delay(duration);
  //NOTE FOR NOW THE OPPOSITE DIRECTION ONE HAS NOT BEEN IMPLEMENTED. WE SHALL SEE IF IT IS NEEDED
}

void fin2_up(int val){
  //for fin2 up => direction = HIGH
  digitalWrite(direc2, HIGH);
  analogWrite(sol2, val);
  //delay(duration);
}

void fin3_up(int val){
  //for fin3 up => direction = HIGH
  digitalWrite(direc3, HIGH);
  analogWrite(sol3, val);
  //delay(duration);
}

void fin4_up(int val){
  //for fin2 up => direction = LOW
  digitalWrite(direc4, LOW);
  analogWrite(sol4, val);
  //delay(duration);
}

//DOWNWARD FUNCTIONS
void fin1_down(int val){
  //for fin1 up => direction = HIGH
  digitalWrite(direc1, HIGH);
  analogWrite(sol1, val);
  //delay(duration);
  //NOTE FOR NOW THE OPPOSITE DIRECTION ONE HAS NOT BEEN IMPLEMENTED. WE SHALL SEE IF IT IS NEEDED
}

void fin2_down(int val){
  //for fin2 up => direction = LOW
  digitalWrite(direc2, LOW);
  analogWrite(sol2, val);
  //delay(duration);
}

void fin3_down(int val){
  //for fin3 up => direction = LOW
  digitalWrite(direc3, LOW);
  analogWrite(sol3, val);
  //delay(duration);
}

void fin4_down(int val){
  //for fin2 up => direction = HIGH
  digitalWrite(direc4, HIGH);
  analogWrite(sol4, val);
  //delay(duration);
}

//------------------------------------------
//CONTROL CODE
//------------------------------------------

void setup(){
  for(int i=3;i<9;i++){
    pinMode(i,OUTPUT);
  } 

  for(int i=11;i<13;i++){
  pinMode(i,OUTPUT);
  }
}

void loop(){
  float utr=0.5;
  int tp=1000;
  fin1_up(255);
  //fin2_up(255);
  //fin3_up(255);
  //fin4_up(255);
  delay(utr*tp);
  fin1_down(255);
  //fin2_down(255);
  //fin3_down(255);
  //fin4_down(255);
  delay((1-utr)*tp);
}