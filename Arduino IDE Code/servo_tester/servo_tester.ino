#include <Servo.h>


Servo s1; 

void setup() {
  s1.attach(1);
}

void loop() {
for (int pos = 0; pos <= 180; pos++) {
    s1.write(pos);

}

delay(500);

for (int pos = 0; pos <= 180; pos++) {
    s1.write(180 - pos);

    delay(15);
}

}
