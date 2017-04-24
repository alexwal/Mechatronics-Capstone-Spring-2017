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
#include <math.h>
#include <Servo.h>

//#define MAX_SIZE 50
#define MAX_MOVES 100
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
float right_servo_180 = 2150;
float lift_servo_write = 1450; //between 1450-1500 depending on pen length
float lift_servo_pause = 700;

// // // // // // // // // // // // // // // // // // // // // // // //
// // // // // // // // // // // // // // // // // // // // // // // //

// Store current angle location and (estimate of) tool position.
float x_cur;
float y_cur;
float t1_cur;
float t2_cur;

// Path tuning parameters (see move(...))         Move links to as close as 90 degrees as possible.
int path_index = 0;

// Create path(s).
// Paths must start out at:   23.5 (midpoint), 74.38 (corresponding y).
//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//
//|\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//|

// -15 is xmin, 65 is xmax.
// 20 is ymin, y_home is ymax.

float xdes_path[] = {X_HOME, 36.5, 35.8637347118, 34.0172209269, 31.1412082798, 27.5172209269, 23.5, 19.4827790731, 15.8587917202, 12.9827790731, 11.1362652882, 10.5, 11.1362652882, 12.9827790731, 15.8587917202, 19.4827790731, 23.5, 27.5172209269, 31.1412082798, 34.0172209269, 35.8637347118, 36.5, X_HOME, -1};
float ydes_path[] = {Y_HOME, 59.38, 63.3972209269, 67.0212082798, 69.8972209269, 71.7437347118, 72.38, 71.7437347118, 69.8972209269, 67.0212082798, 63.3972209269, 59.38, 55.3627790731, 51.7387917202, 48.8627790731, 47.0162652882, 46.38, 47.0162652882, 48.8627790731, 51.7387917202, 55.3627790731, 59.38, Y_HOME, -1};

//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//
//|\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//||\\||//|

float step_size = 0.8; // initial value of alpha in move(...)
float decay = 0.001; // step size decay
float tolerance = 0.4;

// We update these arrays as we move the robot.
float J[2][2]; // this is the Jacobian

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
  midpoint = a + (b - a) / 2.0;
  x_cur = midpoint;
  y_cur = l1 + sqrt(l2 * l2 - (midpoint - a) * (midpoint - a));
  t1_cur = pi / 2;
  t2_cur = pi / 2;

  // Attach servos to pins.
  lift_servo.attach(SERVO_PIN_LIFT);
  left_servo.attach(SERVO_PIN_LEFT);
  right_servo.attach(SERVO_PIN_RIGHT);

  lift_servo.writeMicroseconds(lift_servo_write);
  float t1_us    =   left_rad_to_joint_angle(t1_cur);
  float t2_us    =   right_rad_to_joint_angle(t2_cur);
  left_servo.writeMicroseconds(t1_us);
  right_servo.writeMicroseconds(t2_us);

  timeLoop(millis(), 1000);
}

void timeLoop (long int startMillis, long int interval) { // the delay function
  // this loops until milliseconds has passed since the function began
  while (millis() - startMillis < interval) {}
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
  while (mag(xdes - x_cur, ydes - y_cur) > tolerance && i < MAX_MOVES) {
    Serial.println("[MOVE] Current position @ iteration " + String(i) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
    timeLoop(millis(), 50);

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
      dxdt = dxdt / norm;
      dydt = dydt / norm;
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

    // Update current tool position estimate.
    x_cur = x_cur + alpha * dxdt;
    y_cur = y_cur + alpha * dydt;

    // Increment iterations.
    i += 1;

    if (i % 20 == 0) { // decay step size over time as we're drawing one line.
      alpha = alpha * (1 - decay);
    }
  }

  // This means that the number of moves to go between two points on our path
  // was too high for our comfort. This path will have to be modified so that
  // points are closer, or MAX_MOVES increased.
  //  if (i >= MAX_MOVES) { // maybe increment path index in here if successful (after loop)
  //    Serial.println("MAX_MOVES EXCEEDED!!! Exiting...");
  //    Serial.flush();
  //    exit(0);
  //  }

  // Done moving servos along joint path.
  // return_pen_to_home();
}


//Draw on the fly without saving paths in arrays

//////////////////////////////////////////////////////
void pen_down() {
  // Pen down.
  lift_servo.writeMicroseconds(lift_servo_write);
}

void pen_up() {
  // Pen up.
  lift_servo.writeMicroseconds(lift_servo_pause);
}

// Draw a CCW arc originating from x, y with radius r, origin x0, y0.
void arc(float x0, float y0, float x, float y, float theta){
        pen_up();
        //move to x, y --> the poit from which the arc will begin
        move(x, y);
        pen_down();
        //initalize vars
        float n = 20.0; //number of delta theta steps to take to draw arc
        float del_theta = theta/n;
        float x_cur = x;
        float y_cur = y;
        // this loop moves the arc along for n iterations
        int i = 0;
        while (i < n) {
          //shift x1, y1 so that it is with respect to (0,0) origin
          float x_shifted = x_cur - x0;
          float y_shifted = y_cur = y0;

          //rotate the the shifted point del_theta radians
          float x_rot = x_shifted * cos(del_theta) - y_shifted * sin(del_theta);
          float y_rot = x_shifted * sin(del_theta) + y_shifted * cos(del_theta);

          //shift point back to original origin (x0, y0)
          float x_rot_shifted = x_rot + x0;
          float y_rot_shifted = y_rot + y0;

          //actually do the move to the intermediate point
          move(x_rot_shifted, y_rot_shifted);

          x_cur = x_rot_shifted;
          y_cur = y_rot_shifted;
          i++; 
        }
        pen_up();
}

void line(float x0, float y0, float x1, float y1){
        // Draw a line.
        move(x0, y0);
        pen_down();
        move(x1, y1);
        pen_up();
}

//void write_a() {
//}
//etc...etc..


//////////////////////////////////////////////////////
void loop() {
  
  float x0 = 30.5;
  float y0 = 60.38;
//  move(x0, y0);
  float x = 30.5;
  float y = 60.38;
  float theta = pi/2;
  arc(x0, y0, x, y, theta);
  
  }

void loop2() {
  // Comptued joint_path, which will be the input to servo (it's a sequence of joint angles)
  // Now, we will have a loop which incrementally updates the servo positions so that
  // the end effector ends up at (xdes, ydes).

  // Loop through desired end effector path and set servo angles accordingly and at small enough steps.
  float x_next;
  float y_next;
  if (xdes_path[path_index] != -1 && ydes_path[path_index] != -1) { // not done drawing path
    Serial.println("[LOOP] Current position @ path_index " + String(path_index) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
    timeLoop(millis(), 100);

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
      timeLoop(millis(), 1000);
      Serial.flush();
      timeLoop(millis(), 3000);
      left_servo.detach();
      right_servo.detach();
      exit(0);
    }
  }
}

