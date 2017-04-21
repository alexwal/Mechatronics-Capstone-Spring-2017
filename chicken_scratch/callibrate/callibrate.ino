/*  Mechatronics ME102B
    Simulation of joint angle path
    required to move from one point
    to another.
  
    Assuming servos lined up on horizontal axis,
    and theta drawn counterclockwise and in radians.
    [L], [R]    : servo coordinates.
    [q]         : end effector.
    [A], [B]    : unactuated joints coordinates.
    t1, t2      : joint angles.

                        V [q = (x, y)]
                       / \ 
                      /   \ 
                     /     \ 
                    /       \ 
                   /         \ 
                  /           \ 
                 /             \ 
            [A] /               \ [B]
                \               /
                 \             /
                  \           /
                   \ t1      / t2
[Origin] -------- [L] ----- [R]               
(0, 0)                                         ^ y
|----------------->| = a                       |
|--------------------------->| = b              --> x

    End effector space boundaries:

          [C2]                    [C3]
           ...                    ...
           ...          V  [q]    ...
           ...         / \        ...
           ...                    ...
           ...       ... ...      ...
           ...                    ...
          [C1]      [L]   [R]     [C4]
*/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Servo.h>

#define MAX_SIZE 100
#define pi 3.1415926536

#define SERVO_PIN_LIFT  11
#define SERVO_PIN_LEFT  9
#define SERVO_PIN_RIGHT 13

// Fixed robot parameters. (units?)
double a = 1; // See diagram.
double b = 4; // See diagram. 
double l1 = 2; // | [A] - [L] |
double l2 = 3; // | [q] - [A] |
double l3 = 3; // | [q] - [B] |
double l4 = 2; // | [B] - [R] |
double midpoint; //= a + (b-a) / 2.0;
//assert l2 + l3 > midpoint, 'impossible dimensions: arms disconnected!'

// Corners of end effector space. See diagram.
double C1[2];
double C2[2];
double C3[2];
double C4[2];

// Initialize servos.
Servo left_servo;
Servo right_servo;
Servo lift_servo;

// // // // // // // // // // // // // // // // // // // // // // // // 
// // // // // // // // // // // // // // // // // // // // // // // // 

// Use callibrate.ino to set the zero and 180 degree range.
// At 0 degrees, both servos point right horizontal, and at 180 degrees, servos rotate ccw to left horizontal.
// Callibrate by ONLY attaching first robot arm link to servos. (not whole tool)

double left_servo_0 = 710;
double left_servo_180 = 2400;
double right_servo_0 = 600;
double right_servo_180 = 2320;

// // // // // // // // // // // // // // // // // // // // // // // // 
// // // // // // // // // // // // // // // // // // // // // // // // 

// Store current angle location and (estimate of) tool position.
double x_cur;
double y_cur;
double t1_cur;
double t2_cur;

// Path tuning parameters (see move(...))
int path_index = 0;
double xdes_path[] = {2.500, 2.837, 3.679, 4.858, 5, -1}; // this needs work
double ydes_path[] = {4.598, 4.382, 3.844, 3.090, 3, -1};
double step_size = 0.2; // initial value of alpha in move(...)
double decay = 0.001; // step size decay
double tolerance = 0.1;

// We update these arrays as we move the robot.
double J[2][2]; // this is the Jacobian
double t1_joint_path[MAX_SIZE];
double t2_joint_path[MAX_SIZE];
double x_path[MAX_SIZE];
double y_path[MAX_SIZE]; 

void setup() {
  // Setup "desk" boundaries.
  double extra = 1;
  C1[0] = a - l1;
  C1[1] = 0; 
  C2[0] = a - l1;
  C2[1] = l1 + l2 + extra;
  C3[0] = b + l4;
  C3[1] = l3 + l4 + extra;
  C4[0] = b + l4;
  C4[1] = 0;

  // Set initial conditions of end effector and joint angles.
  midpoint = a + (b-a) / 2.0;
  x_cur = midpoint;
  y_cur = l1 + sqrt(l2 * l2 - (midpoint - a) * (midpoint - a));
  t1_cur = pi/2;
  t2_cur = pi/2; // add description.

  // Attach servos to pins.
  left_servo.attach(SERVO_PIN_LEFT);
  right_servo.attach(SERVO_PIN_RIGHT);
  lift_servo.attach(SERVO_PIN_LIFT);

//  double t1_scaled = t1_cur * 1000 / (2 * pi) + 1000;
//  double t2_scaled = t2_cur * 1000 / (2 * pi) + 1000;

  // // // // // // // // // // // // // // // // // // // // // // // // 
  // // // // // // // // // // // // // // // // // // // // // // // // 

  // TUNE zeros and 180s here.
//  left_servo.writeMicroseconds(left_servo_0); // RECORD RANGES ABOVE!
//  right_servo.writeMicroseconds(right_servo_0); // RECORD RANGES ABOVE!
//  delay(3000);

  // NOTE:  map(value, fromLow, fromHigh, toLow, toHigh);
  // NOTE2: max value of input angle is pi = 180 deg. That explains the from range in map below!
  float deg = 90;
  float left_us   =   map(deg * (pi / 180.0),   0,  pi,  left_servo_0,   left_servo_180);
  float right_us  =   map(deg * (pi / 180.0),   0,  pi,  right_servo_0,  right_servo_180);
  Serial.begin(9600);
  Serial.println(deg);
  Serial.println(left_us);
  Serial.println(right_us);

  
  left_servo.writeMicroseconds(left_us); // RECORD RANGES ABOVE!
  right_servo.writeMicroseconds(right_us); // RECORD RANGES ABOVE!

  // // // // // // // // // // // // // // // // // // // // // // // // 
  // // // // // // // // // // // // // // // // // // // // // // // // 
}

void loop() {

}
