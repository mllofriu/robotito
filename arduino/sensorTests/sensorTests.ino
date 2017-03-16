int ds1 = A9;
int ds2 = A10;
int ds3 = A11;
int ds4 = A12;
int ds5 = A13;
int ds6 = A14;
int ds7 = A15;
int ds8 = A16;
int ds9 = A17;
int ds10 = A18;
int ds11 = A19;
int ds12 = A20;

int NUM_SENSORS = 12;

// Distance sensors starting from front going right
int sensors[] = {ds12, ds11, ds10, ds9, ds8, ds7,
                 ds1, ds2, ds3, ds4, ds5, ds6};
                 


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensors[i], INPUT);

  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(analogRead(sensors[i]));
    Serial.print("\t");
  }
  Serial.println();
}
