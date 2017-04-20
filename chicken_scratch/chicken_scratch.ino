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
#define SERVO_PIN_LEFT  12
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
  left_servo.attach(SERVO_PIN_LIFT);
  right_servo.attach(SERVO_PIN_LEFT);
  lift_servo.attach(SERVO_PIN_RIGHT);
}

// Used in finding: theta' = J * q', the required nudge of angles to get
// tool to move towards qdes = (xdes,ydes).
void compute_Jacobian(double x, double y, double t1, double t2) {
  double J_00 = (-a + x - l1 * cos(t1)) / (l1 * (y * cos(t1) + a * sin(t1) - x * sin(t1)));
  double J_01 = (y - l1 * sin(t1)) / (l1 * (y * cos(t1) + a * sin(t1) - x * sin(t1)));
  double J_10 = (-b + x - l4 * cos(t2)) / (l4 * (y * cos(t2) + b * sin(t2) - x * sin(t2)));
  double J_11 = (y - l4 * sin(t2)) / (l4 * (y * cos(t2) + b * sin(t2) - x * sin(t2)));
  J[0][0] = J_00;
  J[0][1] = J_01;
  J[1][0] = J_10;
  J[1][1] = J_11;
}

// These two functions do theta' = J*q'.
// First row of Jacobian dot [q' = (dxdt, dydt)] is first entry of theta'.
double compute_dt1dt(double dxdt, double dydt) {
  return J[0][0] * dxdt + J[0][1] * dydt;
}

// Second row of Jacobian dot [q' = (dxdt, dydt)] is second entry of theta'.
double compute_dt2dt(double dxdt, double dydt) {
  return J[1][0] * dxdt + J[1][1] * dydt;
}

double mag(double v1, double v2) {
  // Returns the L2 norm of vector v = [v1, v2].
  return sqrt(v1 * v1 + v2 * v2);
}

// Moves and updates the current robot configuration over some time until
// end effector reaches the desired point: q = (xdes, ydes).
void move(double xdes, double ydes) {
  double alpha = step_size;
  int i = 0;
  t1_joint_path[i] = -1;
  t2_joint_path[i] = -1;
  while (mag(xdes - x_cur, ydes - y_cur) > tolerance && i < MAX_SIZE) {

      // The Jacobian is the exact approximation of our dynamics at the current state.
      // We use it to approximate the correct movement of joint angles that will 
      // produce a desired movement of the end effector (towards q = (xdes, ydes)).
      compute_Jacobian(x_cur, y_cur, t1_cur, t2_cur);
      
      // Determine in what direction the end effector should move to get to desired position.
      double dxdt = xdes - x_cur;
      double dydt = ydes - y_cur;

      // Scale q' so that step size is always alpha.
      double norm = mag(dxdt, dydt);
      if (norm > 0.01) {
          dxdt = dxdt/norm;
          dydt = dydt/norm;
      }

      // Compute theta' using Jacobian.
      double dt1dt = compute_dt1dt(dxdt, dydt); // first row of J times q'
      double dt2dt = compute_dt2dt(dxdt, dydt); // secnd row of J times q'
                                                // = theta'
      // Update current position.
      t1_cur = t1_cur + alpha * dt1dt;
      t2_cur = t2_cur + alpha * dt2dt;

      // // // // // // // // // // // // // // //
      // \\ // \\ // \\ // \\ // \\ // \\ // \\ //
      // // // // // // // // // // // // // // //
      
      // Move servos to current t1, t2.
      // servo.writeMicroseconds(uS): 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.

      // Scale t1, t2 to domain of writeMicroseconds.
      double t1_scaled = t1_cur * 1000 / (2 * pi) + 1000;
      double t2_scaled = t2_cur * 1000 / (2 * pi) + 1000;
      left_servo.writeMicroseconds(t1_scaled);
      right_servo.writeMicroseconds(t2_scaled);

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
  t1_joint_path[i] = -1;
  t2_joint_path[i] = -1;
  x_path[i] = -1;
  y_path[i] = -1;
}

void loop() {  
  // Comptued joint_path, which will be the input to servo (it's a sequence of joint angles) 
  // Now, we will have a loop which incrementally updates the servo positions so that
  // the end effector ends up at (xdes, ydes).

  // Loop through desired end effector path and set servo angles accordingly and at small enough steps.
  double x_next;
  double y_next;
  if (xdes_path[path_index] != -1 && ydes_path[path_index] != -1) { // not done drawing path
    printf("Current position @ path_index %d: %f, %f, %f, %f\n", path_index, x_cur, y_cur, t1_cur, t2_cur);
    
    // Grab the next point from desired path.
    x_next = xdes_path[path_index];
    y_next = ydes_path[path_index];
    
    // Compute the necessary servo angles steps needed to move to that point. 
    move(x_next, y_next);
  
    // Prepare to move to the next point. 
    path_index++;

    // Exit if done drawing desired path.
    if (xdes_path[path_index] == -1 || ydes_path[path_index] == -1) {
      printf("Final position @ path_index %d: %f, %f, %f, %f\n", path_index, x_cur, y_cur, t1_cur, t2_cur);
      lift_servo.detach();
      left_servo.detach();
      right_servo.detach();
      exit(0);
    }
  }
}

int main() {
  setup();
  while(1)
    loop();
}
