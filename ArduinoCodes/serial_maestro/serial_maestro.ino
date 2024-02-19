///NOTES ABOUT THE CODE:
//This is the code that implements the on_time_ratio with serial input

struct info{
  int fin_number;
  int otr;
  int period;
};

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
//CONSTITUENT FUNCTIONS
//--------------------------------------

//UPWARD FUNCTIONS
void fin1_up(){
  //for fin1 up => direction = LOW
  digitalWrite(direc1, LOW);
  analogWrite(sol1, 255);
  Serial.println("set fin1 to high");
}

void fin2_up(){
  //for fin2 up => direction = HIGH
  digitalWrite(direc2, HIGH);
  analogWrite(sol2, 255);
}

void fin3_up(){
  //for fin3 up => direction = HIGH
  digitalWrite(direc3, HIGH);
  analogWrite(sol3, 255);
}

void fin4_up(){
  //for fin2 up => direction = LOW
  digitalWrite(direc4, LOW);
  analogWrite(sol4, 255);
  //delay(duration);
}

//DOWNWARD FUNCTIONS
void fin1_down(){
  //for fin1 up => direction = HIGH
  digitalWrite(direc1, HIGH);
  analogWrite(sol1, 255);
  Serial.println("set fin1 to low");
}

void fin2_down(){
  //for fin2 up => direction = LOW
  digitalWrite(direc2, LOW);
  analogWrite(sol2, 255);
}

void fin3_down(){
  //for fin3 up => direction = LOW
  digitalWrite(direc3, LOW);
  analogWrite(sol3, 255);
}

void fin4_down(){
  //for fin2 up => direction = HIGH
  digitalWrite(direc4, HIGH);
  analogWrite(sol4, 255);
}

//------------------------------------------
//SERIAL INPUT MANAGEMENT CODE
//------------------------------------------

info splitter(String inp, char delim) {
  String first, second, third;
  Serial.println(inp);
  int len = inp.length();
  int pos[2];
  info t1;
  int mark=0;
  //Serial.println(inp);
  for (int i = 0; i < len; i++) {
    if (inp[i] == delim) {
      pos[mark] = i; //finding position of delimiter
      mark++;
    }
  }
  for (int i = 0; i < len; i++) {
    if (i < pos[0]) {
      first = first + inp[i]; //getting the first half of the message
    } else if (i == pos[0]) {
      continue;
    } else if (i>pos[0] && i<pos[1]){
      second = second + inp[i]; //getting the second half
    } else if (i==pos[1]){
      continue;
    } else {
      third=third+inp[i]; //getting the time period
    }
  }

  Serial.println(first);
  Serial.println(second);
  Serial.println(third);
  t1.fin_number = first.toInt();
  t1.otr = second.toDouble();
  t1.period=third.toInt();
  
  return t1;
}

//------------------------------------------
//CONTROL CODE
//------------------------------------------

bool test_func(info data){
  Serial.println("Reached test function");
  delay(data.period);
  return true;
}

bool rerouter(info message){
  Serial.println("In re-router");
  bool status=false;
  if(message.fin_number==1){
    Serial.println("Activating fin1");
    fin1_up();
    delay(message.otr*message.period);
    fin1_down();
    delay((1-message.otr)*message.period);
    Serial.println("Done with fin1");
    status=true;
  }
  if(message.fin_number==2){
    Serial.println("Activating fin2");
    fin2_up();
    delay(message.otr*message.period);
    fin2_down();
    delay((1-message.otr)*message.period);
    status=true;
  }
  if(message.fin_number==3){
    Serial.println("Activating fin3");
    fin3_up();
    delay(message.otr*message.period);
    fin3_down();
    delay((1-message.otr)*message.period);
    status=true;
  }
  if(message.fin_number==4){
    Serial.println("Activating fin4");
    fin4_up();
    delay(message.otr*message.period);
    fin4_down();
    delay((1-message.otr)*message.period);
    status=true;
  }
  Serial.println(status);
  return status;
}

void setup(){
  for(int i=3;i<9;i++){
    pinMode(i,OUTPUT);
  } 

  for(int i=11;i<13;i++){
  pinMode(i,OUTPUT);
  }
  Serial.begin(115200);
}

void loop(){
  //fin1(0.1);
  info data;
  bool status=false;
  if (Serial.available() > 0)  // if there is data comming
  {
    String inp = Serial.readStringUntil('\n');  // read string until meet newline character
    Serial.println(inp);
    data = splitter(inp, ';');
    status=rerouter(data);
    Serial.println("returned from rerouter");
    Serial.println(status);
  }
}