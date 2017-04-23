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

#define MAX_SIZE 50
#define pi 3.1415926536

#define X_HOME 23.50 // pen x starting point (MAYBE don't want hard coded)
#define Y_HOME 74.38 // pen y starting point (ditto)

#define SERVO_PIN_LIFT  11
#define SERVO_PIN_LEFT  9
#define SERVO_PIN_RIGHT 13

// Fixed robot parameters. (units: millimeters)
float a = 0; // See diagram.
float b = 47; // See diagram. 
float l1 = 36; // | [A] - [L] |
float l2 = 45; // | [q] - [A] |
float l3 = 45; // | [q] - [B] |
float l4 = 36; // | [B] - [R] |
float midpoint; //= a + (b-a) / 2.0;
//assert l2 + l3 > midpoint, 'impossible dimensions: arms disconnected!'

// Corners of end effector space. See diagram.
float C1[2];
float C2[2];
float C3[2];
float C4[2];

// Initialize servos.
Servo left_servo;
Servo right_servo;
Servo lift_servo;

// // // // // // // // // // // // // // // // // // // // // // // // 
// // // // // // // // // // // // // // // // // // // // // // // // 

// Use callibrate.ino to set the zero and 180 degree range.
// At 0 degrees, both servos point right horizontal, and at 180 degrees, servos rotate ccw to left horizontal.
// IMPORTANT: Callibrate by ONLY attaching first robot arm link to servos. (not whole tool)

float left_servo_0 = 700;
float left_servo_180 = 2440;
float right_servo_0 = 650;
float right_servo_180 = 2050;
float lift_servo_write = 1450; //between 1450-1500 depending on pen length
float lift_servo_pause = 700;

// // // // // // // // // // // // // // // // // // // // // // // // 
// // // // // // // // // // // // // // // // // // // // // // // // 

// Store current angle location and (estimate of) tool position.
float x_cur;
float y_cur;
float t1_cur;
float t2_cur;

// Path tuning parameters (see move(...))          Move links to as close as 90 degrees as possible.
int path_index = 0;
float xdes_path[] = {X_HOME, 40, X_HOME, -1}; //          Paths must start out at:   23.5 (midpoint), 74.38 (corresponding y)
float ydes_path[] = {Y_HOME, Y_HOME, Y_HOME, -1};                               
float step_size = 0.2; // initial value of alpha in move(...)
float decay = 0.001; // step size decay
float tolerance = 0.1;

// We update these arrays as we move the robot.
float J[2][2]; // this is the Jacobian
float t1_joint_path[MAX_SIZE];
float t2_joint_path[MAX_SIZE];
float x_path[MAX_SIZE];
float y_path[MAX_SIZE]; 

float left_rad_to_joint_angle(float rad) {
   return left_servo_0 + rad * (left_servo_180 - left_servo_0) / pi;
}

float right_rad_to_joint_angle(float rad) {
      return right_servo_0 + rad * (right_servo_180 - right_servo_0) / pi;
} 

void setup() {
  // Start serial monitor.
  Serial.begin(9600);

  // Setup "desk" boundaries.
  float extra = 1;
  C1[0] = a - l1;
  C1[1] = 0; 
  C2[0] = a - l1;
  C2[1] = l1 + l2 + extra;
  C3[0] = b + l4;
  C3[1] = l3 + l4 + extra;
  C4[0] = b + l4;
  C4[1] = 0;

  // Set initial conditions of end effector and joint angles.
  // Servos start out at 90 degrees.
  midpoint = a + (b-a) / 2.0;
  x_cur = midpoint;
  y_cur = l1 + sqrt(l2 * l2 - (midpoint - a) * (midpoint - a));
  t1_cur = pi/2;
  t2_cur = pi/2;

  // Attach servos to pins.
  lift_servo.attach(SERVO_PIN_LIFT);
  left_servo.attach(SERVO_PIN_LEFT);
  right_servo.attach(SERVO_PIN_RIGHT);

  lift_servo.writeMicroseconds(lift_servo_write);
  float t1_us    =   left_rad_to_joint_angle(t1_cur);
  float t2_us    =   right_rad_to_joint_angle(t2_cur);
  left_servo.writeMicroseconds(t1_us);
  right_servo.writeMicroseconds(t2_us);

  delay(1000);
}

// Used in finding: theta' = J * q', the required nudge of angles to get
// tool to move towards qdes = (xdes,ydes).
void compute_Jacobian(float x, float y, float t1, float t2) {
  float J_00 = (-a + x - l1 * cos(t1)) / (l1 * (y * cos(t1) + a * sin(t1) - x * sin(t1)));
  float J_01 = (y - l1 * sin(t1)) / (l1 * (y * cos(t1) + a * sin(t1) - x * sin(t1)));
  float J_10 = (-b + x - l4 * cos(t2)) / (l4 * (y * cos(t2) + b * sin(t2) - x * sin(t2)));
  float J_11 = (y - l4 * sin(t2)) / (l4 * (y * cos(t2) + b * sin(t2) - x * sin(t2)));
  J[0][0] = J_00;
  J[0][1] = J_01;
  J[1][0] = J_10;
  J[1][1] = J_11;
}

// These two functions do theta' = J*q'.
// First row of Jacobian dot [q' = (dxdt, dydt)] is first entry of theta'.
float compute_dt1dt(float dxdt, float dydt) {
  return J[0][0] * dxdt + J[0][1] * dydt;
}

// Second row of Jacobian dot [q' = (dxdt, dydt)] is second entry of theta'.
float compute_dt2dt(float dxdt, float dydt) {
  return J[1][0] * dxdt + J[1][1] * dydt;
}

float mag(float v1, float v2) {
  // Returns the L2 norm of vector v = [v1, v2].
  return sqrt(v1 * v1 + v2 * v2);
}

//void return_pen_to_home() {
//  
//}

// Moves and updates the current robot configuration over some time until
// end effector reaches the desired point: q = (xdes, ydes).
void move(float xdes, float ydes) {
  float alpha = step_size;
  int i = 0;
  t1_joint_path[i] = -1;
  t2_joint_path[i] = -1;
  while (mag(xdes - x_cur, ydes - y_cur) > tolerance && i < MAX_SIZE) {
      Serial.println("[MOVE] Current position @ iteration " + String(i) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
      
      // The Jacobian is the exact approximation of our dynamics at the current state.
      // We use it to approximate the correct movement of joint angles that will 
      // produce a desired movement of the end effector (towards q = (xdes, ydes)).
      compute_Jacobian(x_cur, y_cur, t1_cur, t2_cur);
      
      // Determine in what direction the end effector should move to get to desired position.
      float dxdt = xdes - x_cur;
      float dydt = ydes - y_cur;

      // Scale q' so that step size is always alpha.
      float norm = mag(dxdt, dydt);
      if (norm > 0.01) {
          dxdt = dxdt/norm;
          dydt = dydt/norm;
      }
      
      // Compute theta' using Jacobian.
      float dt1dt = compute_dt1dt(dxdt, dydt); // first row of J times q'
      float dt2dt = compute_dt2dt(dxdt, dydt); // secnd row of J times q'
                                                // = theta'
      // Update current position.
      t1_cur = t1_cur + alpha * dt1dt;
      t2_cur = t2_cur + alpha * dt2dt;

      // // // // // // // // // // // // // // //
      // \\ // \\ // \\ // \\ // \\ // \\ // \\ //
      // // // // // // // // // // // // // // //
      
      // Move servos to current t1, t2, first, converting t1, t2 to `us`, domain of writeMicroseconds.
      // NOTE: servo.writeMicroseconds(uS): 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.
      float t1_us    =   left_rad_to_joint_angle(t1_cur);
      float t2_us    =   right_rad_to_joint_angle(t2_cur);

      left_servo.writeMicroseconds(t1_us);
      right_servo.writeMicroseconds(t2_us);

      // // // // // // // // // // // // // // //
      // \\ // \\ // \\ // \\ // \\ // \\ // \\ //
      // // // // // // // // // // // // // // //

      t1_joint_path[i] = t1_cur; // use path to debug.
      t2_joint_path[i] = t2_cur;

      // Update current tool position estimate.
      x_cur = x_cur + alpha * dxdt;
      y_cur = y_cur + alpha * dydt;
      
      x_path[i] = x_cur; // use path to debug.
      y_path[i] = y_cur;
      
      // Increment iterations.
      i += 1;

      if(i % 20 == 0) { // decay step size over time as we're drawing one line.
        alpha = alpha * (1 - decay);
      }
    }

  // Done moving servos along joint path. 
//  return_pen_to_home();
  t1_joint_path[i] = -1;
  t2_joint_path[i] = -1;
  x_path[i] = -1;
  y_path[i] = -1;
}

void loop2() {}

void loop() {
  // Comptued joint_path, which will be the input to servo (it's a sequence of joint angles) 
  // Now, we will have a loop which incrementally updates the servo positions so that
  // the end effector ends up at (xdes, ydes).

  char buffer[100];


  // Loop through desired end effector path and set servo angles accordingly and at small enough steps.
  float x_next;
  float y_next;
  if (xdes_path[path_index] != -1 && ydes_path[path_index] != -1) { // not done drawing path
    Serial.println("[LOOP] Current position @ path_index " + String(path_index) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
    // Grab the next point from desired path.
    x_next = xdes_path[path_index];
    y_next = ydes_path[path_index];
    
    // Compute the necessary servo angles steps needed to move to that point. 
    move(x_next, y_next);
  
    // Prepare to move to the next point. 
    path_index++;

    // Exit if done drawing desired path.
    if (xdes_path[path_index] == -1 || ydes_path[path_index] == -1) {
      Serial.println("[LOOP] Final position @ path_index " + String(path_index) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
//      left_servo.detach();
//      right_servo.detach();
//      exit(0);
    }
  }
}

