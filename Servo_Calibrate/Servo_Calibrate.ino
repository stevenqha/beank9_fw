#include <Servo.h>

Servo servo1;

#define MIN_SIG 500
#define MAX_SIG 2500
#define SERVO_PIN1 12
#define BUF_LENGTH 10

char buffer[BUF_LENGTH];
int numBytes = 0;
int servo_sig = 1500;

int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

void setup() {
    servo1.attach(SERVO_PIN1, MIN_SIG, MAX_SIG);
    Serial.begin(115200);
    while(!Serial) {}
}

void loop() {
    numBytes = Serial.available();
    if(readline(Serial.read(), buffer, BUF_LENGTH) > 0) {
        servo_sig = atoi(buffer);
        Serial.println(servo_sig);
    }

    servo1.writeMicroseconds(servo_sig);
}
