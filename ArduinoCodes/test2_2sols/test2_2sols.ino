//int led_pin = D9;

const int pwmpin = D9; //Define LED Pin
const int freq = 5000;   //Set PWM signal frequency
const int pwmChannel = 0;   //There are 16 channels from 0 to 15, set to PWM channel 0
const int resolution = 8;    //Set the duty cycle resolution of the signal, from 1 to 16 bits. Select 8-bit resolution here, ranging from 0 to 255

struct info{
  uint8_t pname1;
  int val1;
  uint8_t pname2;
  int val2;
};

static info splitter(String inp, char delim){
  String first, second, third, fourth;
  int len=inp.length();
  int pos[3]={-1, -1, -1};
  info t1;
  int pos_p=0;
  //Serial.println(inp);
  for(int i=0; i<len; i++){
    if(inp[i]==delim){
      pos[pos_p] = i;
      pos_p++;
    }
  }
  for(int i=0; i<len; i++){
    if(i<pos[0]){
      first=first+inp[i];
    }
    else if(i==pos[0]){continue;}
    else if(i>pos[0]&&i<pos[1]){
      second=second+inp[i];
    }
    else if(i==pos[1]){continue;}
    else if(i>pos[1]&&i<pos[2]){
      third=third+inp[i];
    }
    else if(i==pos[2]){continue;}
    else {
      fourth=fourth+inp[i];
    }
    
  }
  //Serial.println(first);
  //Serial.println(second);
  t1.pname1=first.toInt();
  t1.val1=second.toInt();
  t1.pname2=third.toInt();
  t1.val2=fourth.toInt();
  return t1;
}

void setup() {
  Serial.begin(9600);
  //pinMode(led_pin, OUTPUT); 
  ledcSetup(pwmChannel,freq,resolution);
  ledcAttachPin(pwmpin,pwmChannel);    //Set the pin for outputting PWM signals and the channel for generating PWM signals                               // set the digital pin as output:
}
void loop() {
  //Serial.println("Starting read, enter value between 0 - 255");
  if(Serial.available()>0)                                   // if there is data comming
  {
    String inp = Serial.readStringUntil('\n');         // read string until meet newline character
    info data = splitter(inp, ';');
    Serial.print(inp);
    ledcWrite(data.pname1, data.val1);
    ledcWrite(data.pname2, data.val2);
    delay(50);
  }
}   