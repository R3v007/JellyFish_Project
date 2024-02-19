int led_pin = D9;
//const int pwmpin = D9; //Define LED Pin
const int pin1 = D3;
const int pin2 = D6;
const int pin3 = D11;
const int pin4 = D12;
const int freq = 5000;  //Set PWM signal frequency
const int chan1 = 0;
const int chan2 = 1;
const int chan3 = 2;
const int chan4 = 3;
const int resolution = 8;  //Set the duty cycle resolution of the signal, from 1 to 16 bits. Select 8-bit resolution here, ranging from 0 to 255

struct info {
  uint8_t cname;
  int val;
};

static info splitter(String inp, char delim) {
  String first, second;
  int len = inp.length();
  int pos = -1;
  info t1;
  //Serial.println(inp);
  for (int i = 0; i < len; i++) {
    if (inp[i] == delim) {
      pos = i;
    }
  }
  for (int i = 0; i < len; i++) {
    if (i < pos) {
      first = first + inp[i];
    } else if (i == pos) {
      continue;
    } else {
      second = second + inp[i];
    }
  }
  Serial.println(first);
  Serial.println(second);
  t1.cname = first.toInt();
  t1.val = second.toInt();
  return t1;
}

void setup() {
  Serial.begin(9600);
  //pinMode(led_pin, OUTPUT);

  ledcSetup(chan1, freq, resolution);
  ledcAttachPin(pin1, chan1);  //Set the pin for outputting PWM signals and the channel for generating PWM signals
  ledcSetup(chan2, freq, resolution);
  ledcAttachPin(pin2, chan2);
  ledcAttachPin(led_pin, chan1);
  ledcSetup(chan3, freq, resolution);
  ledcAttachPin(pin3, chan3);
  ledcSetup(chan4, freq, resolution);
  ledcAttachPin(pin4, chan4);
}

void loop() {
  //Serial.println("Starting read, enter value between 0 - 255");
  if (Serial.available() > 0)  // if there is data comming
  {
    String inp = Serial.readStringUntil('\n');  // read string until meet newline character
    info data = splitter(inp, ';');
    Serial.print(inp);
    ledcWrite(data.cname, data.val);
    delay(50);
  }
}