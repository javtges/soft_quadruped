void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
//    Serial.println(Serial.available());
    
    String params = Serial.readString();
    Serial.println("Read a new set of params");
    Serial.println(params);
  }
}
