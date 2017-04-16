// Plotclock
// cc - by Johannes Heberlein 2014; modifications by ME102B Project Group 2017

#define CALIBRATION      // uncomment to enable calibration mode

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
// Note: robot-right is left, robot-left is right in code, where robot-left means facing from robot's writing perspective.
#define SERVOFAKTORLEFT 610 // 610 describes the range of angles swept by left motor. smaller means smaller range. SERVOFAKTORLEFT and -RIGHT should be about same.
#define SERVOFAKTORRIGHT 630 //630

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
// determine the origin from which servos rotate +90 deg.
#define SERVOLEFTNULL 2200 //  bigger means doesn't go as far outside (to the right of the desk)
#define SERVORIGHTNULL 1000 // sets the horizontal line (goes down below writing surface when bigger)

#define SERVOPINLIFT  11 // lift
#define SERVOPINLEFT  12 // left
#define SERVOPINRIGHT 13 // right

// lift positions of lifting servo
#define LIFT0 940  // on drawing surface ((higher number is pen closer to surface, lower is more raised) writing plane angle, 0 is writing surface lifted up)
#define LIFT1 825  // between numbers (lift on space???)
#define LIFT2 1005  // going towards eraser (lift on erase???)

// speed of lifting arm, higher is slower
#define LIFTSPEED 3000 // 1500

// length of arms
#define L1 35
#define L2 57.1
#define L3 13.2
// shorter servo arm is (L2 - L3).

// origin points of left and right servo
#define O1X 21
#define O1Y -25
#define O2X 48
#define O2Y -25

#define PARKX 77
#define PARKY 47
#define ERASEMAXX 60

#include <TimeLib.h> // see http://playground.arduino.cc/Code/time 
#include <Servo.h>

int servoLift = 1500;

Servo servo1; // lift
Servo servo2; // left
Servo servo3; // right

volatile double lastX = 75;
volatile double lastY = PARKY;

int last_min = 0;

//#include <Math.h>

void setup()
{
  // Set current time only the first to values, hh,mm are needed
  setTime(15, 35, 0, 0, 0, 0);
  drawTo(PARKX, PARKY);
  lift(0);
  servo1.attach(SERVOPINLIFT);  //  lifting servo
  servo2.attach(SERVOPINLEFT);  //  left servo
  servo3.attach(SERVOPINRIGHT);  //  right servo
  delay(1000);
}

void loop()
{

#ifdef CALIBRATION
  // Servohorns will have 90° between movements, parallel to x and y axis
  drawTo(-3, 29.2);
  delay(500);
  drawTo(74.1, 28);
  delay(500);

#else


  int i = 0;
  if (last_min != minute()) {

    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

    lift(0);

    hour();
    while ((i + 1) * 10 <= hour())
    {
      i++;
    }
    // erase 2x
    number(3, 3, 111, 1);
    number(3, 3, 111, 1);
    // hour1
    number(5, 25, i, 0.9);
    // hour2
    number(19, 25, (hour() - i * 10), 0.9);
    // colon
    number(28, 25, 11, 0.9);

    i = 0;
    while ((i + 1) * 10 <= minute())
    {
      i++;
    }
    // minute1
    number(34, 25, i, 0.9);
    // minute2
    number(48, 25, (minute() - i * 10), 0.9);
    lift(2);
    drawTo(PARKX, PARKY);
    lift(1);
    last_min = minute();

    servo1.detach();
    servo2.detach();
    servo3.detach();
  }

#endif

}

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

    case 0:
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(0);
      bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
      lift(1);
      break;
    case 1:

      drawTo(bx + 3 * scale, by + 15 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(1);
      break;
    case 2:
      drawTo(bx + 2 * scale, by + 12 * scale);
      lift(0);
      bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
      drawTo(bx + 1 * scale, by + 0 * scale);
      drawTo(bx + 12 * scale, by + 0 * scale);
      lift(1);
      break;
    case 3:
      drawTo(bx + 2 * scale, by + 17 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
      bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
      lift(1);
      break;
    case 4:
      drawTo(bx + 10 * scale, by + 0 * scale);
      lift(0);
      drawTo(bx + 10 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 6 * scale);
      drawTo(bx + 12 * scale, by + 6 * scale);
      lift(1);
      break;
    case 5:
      drawTo(bx + 2 * scale, by + 5 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
      drawTo(bx + 5 * scale, by + 20 * scale);
      drawTo(bx + 12 * scale, by + 20 * scale);
      lift(1);
      break;
    case 6:
      drawTo(bx + 2 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
      drawTo(bx + 11 * scale, by + 20 * scale);
      lift(1);
      break;
    case 7:
      drawTo(bx + 2 * scale, by + 20 * scale);
      lift(0);
      drawTo(bx + 12 * scale, by + 20 * scale);
      drawTo(bx + 2 * scale, by + 0);
      lift(1);
      break;
    case 8:
      drawTo(bx + 5 * scale, by + 10 * scale);
      lift(0);
      bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
      lift(1);
      break;

    case 9:
      drawTo(bx + 9 * scale, by + 11 * scale);
      lift(0);
      bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
      drawTo(bx + 5 * scale, by + 0);
      lift(1);
      break;

    case 111:

      lift(0);
      drawTo(70, 46);
      drawTo(ERASEMAXX, 43);

      drawTo(ERASEMAXX, 49);
      drawTo(5, 49);
      drawTo(5, 45);
      drawTo(ERASEMAXX, 45);
      drawTo(ERASEMAXX, 40);

      drawTo(5, 40);
      drawTo(5, 35);
      drawTo(ERASEMAXX, 35);
      drawTo(ERASEMAXX, 30);

      drawTo(5, 30);
      drawTo(5, 25);
      drawTo(ERASEMAXX, 25);
      drawTo(ERASEMAXX, 20);

      drawTo(5, 20);
      drawTo(60, 44);

      drawTo(PARKX, PARKY);
      lift(2);

      break;

    case 11:
      drawTo(bx + 5 * scale, by + 15 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      drawTo(bx + 5 * scale, by + 5 * scale);
      lift(0);
      bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
      lift(1);
      break;

  }
}



void lift(char lift) {
  switch (lift) {
    // room to optimize  !

    case 0: //850

      if (servoLift >= LIFT0) {
        while (servoLift >= LIFT0)
        {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT0) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);

        }

      }

      break;

    case 1: //150

      if (servoLift >= LIFT1) {
        while (servoLift >= LIFT1) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);

        }
      }
      else {
        while (servoLift <= LIFT1) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }

      }

      break;

    case 2:

      if (servoLift >= LIFT2) {
        while (servoLift >= LIFT2) {
          servoLift--;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      else {
        while (servoLift <= LIFT2) {
          servoLift++;
          servo1.writeMicroseconds(servoLift);
          delayMicroseconds(LIFTSPEED);
        }
      }
      break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
           radius * sin(start + count) + by);
    count += inkr;
  }
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  // path length in mm * 4 = 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));  // Approx arc length in mm is sqrt(dx * dx + dy * dy). Number of servo steps is 4 * (# mm). c gives us number of servo steps to new point.

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) { // for each servo step in arc length
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));
  }
  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty)
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint (servo-left joint angle)
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar length (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); //
  a1 = atan2(dy, dx); // (angle on surface)
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // calculate joint arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));

}


