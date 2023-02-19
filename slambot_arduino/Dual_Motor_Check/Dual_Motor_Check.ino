#define ENC1A 2
#define ENC1B 3
#define ENC2A 18
#define ENC2B 19

int count_pulses_enc1=0;
int count_pulses_enc2=0;
void setup() {
  Serial.begin(9600);
  pinMode(ENC1A,INPUT);
  pinMode(ENC1B,INPUT);
  pinMode(ENC2A,INPUT);
  pinMode(ENC2B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1A),motor1_encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A),motor2_encoder,RISING);
  
  // put your setup code here, to run once:

}

void loop() {
  
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.println("Encoder Count");
  Serial.println(count_pulses_enc1);
  Serial.println(count_pulses_enc2);
  Serial.println(".........................");

}
void motor1_encoder(){
  int b=digitalRead(ENC1B);

  if (b>0){
    count_pulses_enc1++;
  }
  else {
     count_pulses_enc1--;
  }
}

void motor2_encoder(){
  int b=digitalRead(ENC2B);

  if (b>0){
    count_pulses_enc2++;
  }
  else {
     count_pulses_enc2--;
  }
}
