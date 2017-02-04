
int m1en = 3;
int m2en = 4;
int m3en = 5;
int m4en = 6;

int m1i1 = 7;
int m1i2 = 8;
int m2i1 = 9;
int m2i2 = 10;
int m3i1 = 11;
int m3i2 = 12;
int m4i1 = 13;
int m4i2 = 14;

int vel = 100;
int restdelay = 500;

void setup(){
  pinMode(m1en, OUTPUT);
  pinMode(m1i1, OUTPUT);
  pinMode(m1i2, OUTPUT);
  
  pinMode(m2en, OUTPUT);
  pinMode(m2i1, OUTPUT);
  pinMode(m2i2, OUTPUT);
  
  pinMode(m3en, OUTPUT);
  pinMode(m3i1, OUTPUT);
  pinMode(m3i2, OUTPUT);
  
  pinMode(m4en, OUTPUT);
  pinMode(m4i1, OUTPUT);
  pinMode(m4i2, OUTPUT);


  digitalWrite(m1i1, LOW);
  digitalWrite(m1i2, LOW);
  analogWrite(m1en, 0);
  
  digitalWrite(m2i1, LOW);
  digitalWrite(m2i2, LOW);
  analogWrite(m2en, 0);
  
  digitalWrite(m3i1, LOW);
  digitalWrite(m3i2, LOW);
  analogWrite(m3en, 0);
  
  digitalWrite(m4i1, LOW);
  digitalWrite(m4i2, LOW);
  analogWrite(m4en, 0);

 
}

void loop(){
  analogWrite(m1en, 0);
  analogWrite(m2en, 0);
  analogWrite(m3en, 0);
  analogWrite(m4en, 0);

  delay(restdelay);

  digitalWrite(m3i1, HIGH);
  digitalWrite(m3i2, LOW);
  digitalWrite(m4i1, LOW);
  digitalWrite(m4i2, HIGH);

  analogWrite(m4en, vel);
  analogWrite(m3en, vel);

  delay(restdelay);

  analogWrite(m1en, 0);
  analogWrite(m2en, 0);
  analogWrite(m3en, 0);
  analogWrite(m4en, 0);

  delay(restdelay);
  
  digitalWrite(m4i1, HIGH);
  digitalWrite(m4i2, LOW);
  digitalWrite(m1i1, LOW);
  digitalWrite(m1i2, HIGH);

  analogWrite(m1en, vel);
  analogWrite(m4en, vel);

  delay(restdelay);

  analogWrite(m1en, 0);
  analogWrite(m2en, 0);
  analogWrite(m3en, 0);
  analogWrite(m4en, 0);

  delay(restdelay);
  
  digitalWrite(m1i1, HIGH);
  digitalWrite(m1i2, LOW);
  digitalWrite(m3i1, LOW);
  digitalWrite(m3i2, HIGH);

  analogWrite(m1en, vel);
  analogWrite(m3en, vel);

  delay(restdelay);

}  
