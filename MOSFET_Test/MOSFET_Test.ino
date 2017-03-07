int MOSpin1 = 7;
int MOSpin2 = 8;
int MOSpin3 = 9;
int MOSpin4 = 10;
int MOSpin5 = 11;
int MOSpin6 = 12;
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MOSpin1, OUTPUT);
  pinMode(MOSpin2, OUTPUT);
  pinMode(MOSpin3, OUTPUT);
  pinMode(MOSpin4, OUTPUT);
  pinMode(MOSpin5, OUTPUT);
  pinMode(MOSpin6, OUTPUT);
}

void loop() {
  digitalWrite(7, HIGH);
  Serial.println("hi");
  delay(5);
  digitalWrite(7, LOW);
  delay(5);
}
