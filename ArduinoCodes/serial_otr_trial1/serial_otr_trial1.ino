///NOTES ABOUT THE CODE:
//This is the code that implements the serial input

struct info{ //struct that you can modify to use the data you want
  double otr[4];
  double tp[4];
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
//MOVEMENT FUNCTIONS
//--------------------------------------

//UPWARD FUNCTIONS
void fin1_up(){
  //for fin1 up => direction = LOW
  digitalWrite(direc1, LOW);
  analogWrite(sol1, 255);
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
  String f1otr, f1tp, f2otr, f2tp, f3otr, f3tp, f4otr, f4tp;
  Serial.println(inp);
  int len = inp.length();
  int pos[7]; //to add more message elements, increase the index here
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
      f1otr = f1otr + inp[i]; //getting the first half of the message
    } else if (i == pos[0]) {
      continue;
    } else if (i>pos[0] && i<pos[1]){
      f1tp = f1tp + inp[i]; //getting the second half
    } else if (i==pos[1]){
      continue;
    } else if (i>pos[1] && i<pos[2]){
      f2otr = f2otr + inp[i]; //getting the second half
    } else if (i==pos[2]){
      continue;
    } else if (i>pos[2] && i<pos[3]){
      f2tp = f2tp + inp[i]; //getting the second half
    } else if (i==pos[3]){
      continue;
    } else if (i>pos[3] && i<pos[4]){
      f3otr = f3otr + inp[i]; //getting the second half
    } else if (i==pos[4]){
      continue;
    }else if (i>pos[4] && i<pos[5]){
      f3tp = f3tp + inp[i]; //getting the second half
    } else if (i==pos[5]){
      continue;
    }else if (i>pos[5] && i<pos[6]){
      f4otr = f4otr + inp[i]; //getting the second half
    } else if (i==pos[6]){
      continue;
    }else {
      f4tp=f4tp+inp[i]; //getting the time period
    } //for more message elements add more if conditions. 
  }
  info t1;

  t1.otr[0]=f1otr.toDouble();
  t1.otr[1]=f2otr.toDouble();
  t1.otr[2]=f3otr.toDouble();
  t1.otr[3]=f4otr.toDouble();
  
  t1.tp[0]=f1tp.toDouble();
  t1.tp[1]=f2tp.toDouble();
  t1.tp[2]=f3tp.toDouble();
  t1.tp[3]=f4tp.toDouble();
  Serial.println("Time Periods:"+f1tp+";"+f2tp+";"+f3tp+";"+f4tp);
  Serial.println("On-time Ratios:"+f1otr+";"+f2otr+";"+f3otr+";"+f4otr);
  return t1;
}

//------------------------------------------
//CONTROL CODE
//------------------------------------------

//just a test function 
bool test_func(info data){
  Serial.println("Reached test function");
  return true;
}

bool rerouter(info message){ //if statements to move to the correct fin
  Serial.println("In re-router");
  Serial.println("Activating fin1");
  fin1_up();
  delay(message.otr[0]*message.tp[0]);
  fin1_down();
  delay((1-message.otr[0])*message.tp[0]);
  
  Serial.println("Activating fin2");
  fin2_up();
  delay(message.otr[1]*message.tp[1]);
  fin2_down();
  delay((1-message.otr[1])*message.tp[1]);

  Serial.println("Activating fin3");
  fin2_up();
  delay(message.otr[2]*message.tp[2]);
  fin2_down();
  delay((1-message.otr[2])*message.tp[2]);


  Serial.println("Activating fin4");
  fin2_up();
  delay(message.otr[3]*message.tp[3]);
  fin2_down();
  delay((1-message.otr[3])*message.tp[3]);
  return true;
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
    Serial.println(inp); //checking if the string is received
    data = splitter(inp, ';'); //sending for splitting
    status=rerouter(data); //routing
    Serial.println("returned from rerouter"); //check print statement. can be removed
    Serial.println(status); //again checking
  }
}