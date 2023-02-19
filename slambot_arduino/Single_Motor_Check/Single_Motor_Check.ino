#define ENC1A 2
#define ENC1B 3

int count_pulses=0;
void setup() {
  Serial.begin(9600);
  pinMode(ENC1A,INPUT);
  pinMode(ENC1B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1A),motor1_encoder,RISING);
  
  // put your setup code here, to run once:

}

void loop() {
  
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.println(count_pulses);

}
void motor1_encoder(){
  int b=digitalRead(ENC1B);

  if (b>0){
    count_pulses++;
  }
  else {
     count_pulses--;
  }
}
