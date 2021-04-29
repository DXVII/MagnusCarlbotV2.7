#define joyX A14
#define joyY A15
#define SW 52 

const int ledPin = LED_BUILTIN;
int xValue = 0, yValue = 0, SW_state = 0;
int mapX = 0, mapY = 0;



void setup() {
  Serial.begin(9600);
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
}
 
void loop() {
  // put your main code here, to run repeatedly:
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);
  SW_state = digitalRead(SW);
//  mapX = map(xValue, -10, 10, -512, 512);
//  mapY = map(yValue, -10, 10, -512, 512);

  if(SW_state){
    
    digitalWrite(ledPin, HIGH);
  }
  else{
    digitalWrite(ledPin, LOW);
  }
  
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(" | Y: ");
  Serial.print(yValue);
  Serial.print(" | Button: ");
  Serial.println(SW_state);

  delay(1000);
  
}
