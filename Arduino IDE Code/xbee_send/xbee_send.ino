void setup() {
  Serial1.begin(9600);      // Match XBee baud rate
}

void loop() {
  Serial1.println("Hello from Teensy!");
  delay(1000);
}