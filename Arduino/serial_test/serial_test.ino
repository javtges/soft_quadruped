
void setup() {
 Serial.begin(115200);
 Serial.setTimeout(1);
}
void loop() {
 while (!Serial.available());
 String x = Serial.readString();
 Serial.print(x);
}
