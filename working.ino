
struct info {
  double otr;
  int tp;
};

double otr=0.5;
double freq=1.0;
double time_s=1.0;
double time_ms=100.0;

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

//--------------------------------------
//MOVEMENT FUNCTIONS
//--------------------------------------

//ZERO STATES
void fin1_zero(){
  digitalWrite(direc1, LOW);
  analogWrite(sol1, 0);
}

void fin2_zero(){
  digitalWrite(direc2, LOW);
  analogWrite(sol2, 0);
}

void fin3_zero(){
  digitalWrite(direc3, LOW);
  analogWrite(sol3, 0);
}

void fin4_zero(){
  digitalWrite(direc4, LOW);
  analogWrite(sol4, 0);
}

//UPWARD FUNCTIONS
void fin1_up(){
  digitalWrite(direc1, HIGH);
  analogWrite(sol1, 230);
}

void fin2_up(){
  digitalWrite(direc2, HIGH);
  analogWrite(sol2, 230);
}

void fin3_up(){
  digitalWrite(direc3, HIGH);
  analogWrite(sol3, 230);
}

void fin4_up(){
  digitalWrite(direc4, HIGH);
  analogWrite(sol4, 230);
  //delay(duration);
}

//DOWNWARD FUNCTIONS
void fin1_down(){
  digitalWrite(direc1, LOW);
  analogWrite(sol1, 210);
}

void fin2_down(){
  digitalWrite(direc2, LOW);
  analogWrite(sol2, 210);
}

void fin3_down(){
  digitalWrite(direc3, LOW);
  analogWrite(sol3, 210);
}

void fin4_down(){
  digitalWrite(direc4, LOW);
  analogWrite(sol4, 210);
}

void setup() {
  for(int i=3;i<9;i++){
    pinMode(i,OUTPUT);
  } 
  for(int i=11;i<13;i++){
  pinMode(i,OUTPUT);
  }
  Serial.begin(115200);
}



void loop(){
  //int time=500;  // if there is data comming
  if(Serial.available() >0){
    String inp_1 = Serial.readStringUntil(';'); 
    otr=inp_1.toDouble();
    String inp_2=Serial.readStringUntil('\n');
    freq=inp_2.toDouble();}
    time_s=1/freq;
    time_ms=1000*time_s;
    Serial.println(time_ms);
    Serial.println(otr);
    switch(int(otr*10)){
      case 0:
        Serial.println("Zero State");
        fin1_zero();
        fin3_zero();
        fin2_zero();
        fin4_zero();
        break;
      default:
        Serial.println("moving");
        fin1_up();
        fin2_up();
        fin3_up();
        fin4_up();
        delay(otr*time_ms);
        fin1_zero();
        fin3_zero();
        fin2_zero();
        fin4_zero();

        fin1_down();
        fin2_down();
        fin3_down();
        fin4_down();
        delay((1-otr)*time_ms);
        fin1_zero();
        fin2_zero();
        fin3_zero();
        fin4_zero();
        break;
      }
}
