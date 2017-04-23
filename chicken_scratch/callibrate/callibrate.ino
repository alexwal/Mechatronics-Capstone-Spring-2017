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

// Fixed robot parameters. (units?) mm
double a = 0; // See diagram.
double b = 47; // See diagram. 
double l1 = 36; // | [A] - [L] |
double l2 = 45; // | [q] - [A] |
double l3 = 45; // | [q] - [B] |
double l4 = 36; // | [B] - [R] |
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
// IMPORTANT: Callibrate by ONLY attaching first robot arm link to servos. (not whole tool)

double left_servo_0 = 700;
double left_servo_180 = 2440;
double right_servo_0 = 650;
double right_servo_180 = 2050;

double lift_servo_write = 1450; //between 1450-1500 depending on pen length
double lift_servo_pause = 700;


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

double left_rad_to_joint_angle(double rad) {
   return left_servo_0 + rad * (left_servo_180 - left_servo_0) / pi;
}

double right_rad_to_joint_angle(double rad) {
      return right_servo_0 + rad * (right_servo_180 - right_servo_0) / pi;
}

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

  double rad = pi / 2;
  double left_us   =   left_rad_to_joint_angle(rad);
  double right_us  =   right_rad_to_joint_angle(rad);
  
  Serial.begin(9600);
  Serial.println(rad);
  Serial.println("left_us");
  Serial.println(left_us);
  Serial.println("right_us");
  Serial.println(right_us);

  left_servo.writeMicroseconds(left_us); // RECORD RANGES ABOVE!
  right_servo.writeMicroseconds(right_us); // RECORD RANGES ABOVE!
  delay(2000);

  // // // // // // // // // // // // // // // // // // // // // // // // 
  // // // // // // // // // // // // // // // // // // // // // // // //

// Callibrate left and right servos.

//  while(1) {
//    // Go to 0
//    //left_servo.writeMicroseconds(left_servo_0); // RECORD RANGES ABOVE!
//    right_servo.writeMicroseconds(right_servo_0); // RECORD RANGES ABOVE!
//    delay(2000);
//
//    // Go to 'deg'
//    //left_servo.writeMicroseconds(left_us); // RECORD RANGES ABOVE!
//    right_servo.writeMicroseconds(right_us); // RECORD RANGES ABOVE!
//    delay(2000);
//
//    // Got to 180
//    //left_servo.writeMicroseconds(left_servo_180); // RECORD RANGES ABOVE!
//    right_servo.writeMicroseconds(right_servo_180); // RECORD RANGES ABOVE!
//    delay(2000);
//  }

// Callibrate lift servo.

  while(1) {
    lift_servo.writeMicroseconds(lift_servo_write); // RECORD RANGES ABOVE!
    delay(2000);

    lift_servo.writeMicroseconds(lift_servo_pause); // RECORD RANGES ABOVE!
    delay(2000);
  }

  // // // // // // // // // // // // // // // // // // // // // // // // 
  // // // // // // // // // // // // // // // // // // // // // // // // 
   
}

void loop() {

}
