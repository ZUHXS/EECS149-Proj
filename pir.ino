const int Pin =  6;
char control = 0;
float start = 0;
float tem = 0;
int mod = 0;
void setup() {
  pinMode(Pin, OUTPUT); 
  digitalWrite(Pin, LOW);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    control=Serial.read();
  }
  if (control == 'a'){
    mod = 1;
  }
  if ((control == 'b')&&( mod == 1)){
    mod = 2;
  } 
  tem=millis();
  if(tem>5000){
    if(mod == 1){
      digitalWrite(Pin, HIGH);
    }
    else if(mod == 2){
      digitalWrite(Pin, LOW);
      int n=0;
      int sum = 0;
        while (n<10){
          int sensorValue = analogRead(A0);
          //Serial.println(mod);
          sum+=sensorValue;
          n++;
          delay(100);
        }
        if (sum>6000){
        digitalWrite(Pin, HIGH);
        mod=3;
        start=millis();
        }
      }
    else if (mod == 3){
      tem=millis();
      if((tem-start) > 300000){
        int n=0;
        int sum = 0;
        while (n<10){
          int sensorValue = analogRead(A0);
          //Serial.println(mod);
          sum+=sensorValue;
          n++;
          delay(100);
        }
        if (sum<5000){
          digitalWrite(Pin, LOW);
          mod=2;
        }
      }
      digitalWrite(Pin, HIGH);
    }
    else{
      digitalWrite(Pin, LOW);
    }
    delay(100);  
  }
}
