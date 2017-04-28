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

int motor1Pin = 5;
int motor2Pin = 6;
int onSpeed = 150;
int offSpeed = 0;
boolean notWriting = true;

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

float left_servo_0 = 600;
float left_servo_180 = 2440;
float right_servo_0 = 580;
float right_servo_180 = 2300;
float lift_servo_write = 1450; //between 1450-1500 depending on pen length
float lift_servo_pause = 1350; //was 700

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
float decay = 0.0; // step size decay
float tolerance = 0.4;

//used to determine the bounds of rect inside which letters are drawn
double x_top_left = X_HOME - 15 + 7.5 + 2.5; //18.5
double y_top_left = Y_HOME-25 + 20 + 6; //74.38 

double x_top_right = X_HOME + 15 - 7.5 - 2.5; //28.5
double y_top_right = y_top_left; //74.38

double x_bottom_left = x_top_left; //18.5
double y_bottom_left = Y_HOME-25 - 0 - 4; //44.38

double x_bottom_right = x_top_right; //28.5
double y_bottom_right = y_bottom_left; //44.38 

// We update these arrays as we move the robot.
float J[2][2]; // this is the Jacobian

float left_rad_to_joint_angle(float rad) {
  return left_servo_0 + rad * (left_servo_180 - left_servo_0) / pi;
}

float right_rad_to_joint_angle(float rad) {
  return right_servo_0 + rad * (right_servo_180 - right_servo_0) / pi;
}

void setup() {
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  
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

void moveMotors() {
   // Motors turned on, move paper distance of ____ with on time of _____.
   analogWrite(motor1Pin, offSpeed);
   analogWrite(motor2Pin, onSpeed);
   delay(250);
   // Turn off motors after character space stopped.
   analogWrite(motor1Pin, offSpeed);
   analogWrite(motor2Pin, offSpeed);
   delay(2000);
   
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
//    Serial.println("[MOVE] Current position @ iteration " + String(i) + ", x_cur: " + String(x_cur, 2) + ", y_cur: " + String(y_cur, 2) + ", t1_cur: " + String(t1_cur, 2) + ", t2_cur: " + String(t2_cur, 2));
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
        //pen_up();
        //move to x, y --> the point from which the arc will begin
        move(x, y);
//        pen_down();
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
          float y_shifted = y_cur - y0;

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
        //pen_up();
}

void line(float x0, float y0, float x1, float y1){
        // Draw a line.
        move(x0, y0);
//        pen_down();
        move(x1, y1);
//        pen_up();
}

void return_to_home() {
//  pen_up();
  move(X_HOME, Y_HOME); // gets us pretty close to original position so that we can explicitly set servo angle.
  double rad = pi / 2;
  double left_us   =   left_rad_to_joint_angle(rad);
  double right_us  =   right_rad_to_joint_angle(rad);
  left_servo.writeMicroseconds(left_us); // RECORD RANGES ABOVE!
  right_servo.writeMicroseconds(right_us); // RECORD RANGES ABOVE!
  x_cur = X_HOME;
  y_cur = Y_HOME;
//  pen_down();
}

//draw_a() {
//  // RANGE is x = [], y in []
//  // circle
//  x0, y0 = a, b;
//  r = n;
//  
//  // line
//  start = (x0, y0) + (r, r);
//  end = (x0, y0) + (r, -r);
//
//  // translate by bottom left point of WORKING REGION!!!
//  // so that we're back in robot coordinates.
//  x0, y0 += bot left
//  start += bot left
//  end += bot left
//
//  line(start, end);
//  arc(x0, y0, x0+r, y0, 360);
//}

//drawing a circle in middle third
void func_1() {
  float x0 = (x_bottom_left + x_bottom_right)/2.0;
  float y0 = (y_bottom_left + y_top_left)/ 2.0;
  float x = x_bottom_right;
  float y = y0;
  float theta = 2*pi;
  
//  pen_up();
  move(x, y);
  pen_down();

  arc(x0, y0, x, y, theta);
}

//draw  verticle line in middle third on right edge
void func_2() {
    float x0 = x_bottom_right;
    float y0 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);
    float x = x_bottom_right;
    float y = ((2.0*y_bottom_right)/3.0) + (y_top_right/3.0);
    
//    pen_up();
    move(x0,y0);
    pen_down();

    line(x0, y0, x, y);
}

//draw verticle line on left edge along top and middle third
void func_3() {
  float x0 = x_top_left;
  float y0 = y_top_left;
  float x = x_bottom_left;
  float y = ((2.0*y_bottom_left)/3.0) + (y_top_left/3.0);

  move(x0, y0);
  pen_down();
  line(x0, y0, x, y);
}

//for the letter c
void func_4(){
  float x0 = (x_bottom_left + x_bottom_right)/2.0;
  float y0 = (y_bottom_left + y_top_left)/ 2.0;

  float x = ((x_bottom_left + x_bottom_right)/2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  float y = ((y_bottom_left + y_top_left)/ 2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  
  move(x, y);
  pen_down();
  float theta = 1.5*pi;

  arc(x0, y0, x, y, theta);
}

//draw verticle line on right edge along top and middle third
void func_5(){
  float x0 = x_top_right;
  float y0 = y_top_right;

  move(x0, y0);
  pen_down();

  float x = x_bottom_right;
  float y = ((2.0*y_bottom_right)/3.0) + (y_top_right/3.0);

  line(x0, y0, x, y); 
}

//makes a 180 arc in the top half of the middle third
void func_6() {
  float x0 = (x_bottom_left + x_bottom_right)/2.0;
  float y0 = (y_bottom_left + y_top_left)/ 2.0;

  float x = x_bottom_left;
  float y = y0;
  move(x, y);
  pen_down();
  float theta = -pi;
  
  arc(x0, y0, x, y, theta);
  
}

//bottom half of 'e' CHANGED to first e to c
void func_7() {
  float x0 = x_bottom_left;
  float y0 = (y_bottom_left + y_top_left)/2.0;

//  float x0 = ((x_bottom_left + x_bottom_right)/2.0) - ((sqrt(2)/12.0) * (y_top_right - y_bottom_right)); //bigger loop e
//  float y0 = ((y_bottom_left + y_top_left)/ 2.0) - ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));

  move(x0, y0);
  pen_down();

  float x = ((x_bottom_left + x_bottom_right)/2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  float y = ((y_bottom_left + y_top_left)/ 2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  
  line(x0, y0, x, y); 
  
//  float x0 = x_bottom_right;
//  float y0 = (y_bottom_left + y_top_left)/ 2.0;
//
//  pen_up();
//  move(x0, y0);
//  pen_down();
//
//  float x1 = x_bottom_left;
//  float y1 = (y_bottom_left + y_top_left)/2.0;
//  line(x0, y0, x1, y1);
//
//  arc(x0, y0, x1, y1, pi);
//
//  float x2 = (x_bottom_left + x_bottom_right)/2.0;
//  float y2 = ((2.0*y_bottom_left)/3.0) + (y_top_left/3.0);
//
//  float x3 = x_bottom_right;
//  float y3 = ((2.0*y_bottom_right)/3.0) + (y_top_right/3.0);  
}

//draw 'f'
void func_8() {
  float x0 = x_bottom_right;
  float y0 = (((5.0 * y_top_left) + y_bottom_left)/6.0);

  float x = x_top_right;
  float y = y_top_right;
//  pen_up();
  move(x, y);
  pen_down();
  float theta = pi/2;
  
  arc(x0, y0, x, y, theta);

  float x1 = (x_bottom_right + x_bottom_left)/2.0;
  float y1 = ((5.0 * y_top_left)/6.0) + y_bottom_left/6.0; 

  float x2 = (x_bottom_left + x_bottom_right)/2.0;
  float y2 = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0;

  line(x1,y1, x2, y2);
}

//write 'j'
void func_9() {
  float x0 = x_bottom_right;
  float y0 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

//  pen_up();
  move (x0, y0);
  pen_down();
  float x1 = x_bottom_right;
  float y1 = (y_top_left/6.0) + ((5.0 * y_bottom_left)/6.0);
  
  line(x0, y0, x1, y1);
  
  float x_orig = (x_bottom_left + x_bottom_right)/2.0;
  float y_orig = (y_top_left/6.0) + ((5.0 * y_bottom_left)/6.0);

  arc(x_orig, y_orig, x1, y1, -pi);
}


//verticle line in the middle of the third third //DEBUG THIS
void func_10() {
  float x0 = (x_bottom_left + x_bottom_right)/2.0;
  float y0 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

//  pen_up();
  move(x0, y0);
  pen_down();

  float x = (x_bottom_left + x_bottom_right)/2.0;
  float y = ((2.0*y_bottom_left)/3.0) + (y_top_left/3.0);
  line(x0, y0, x,y);
  
  pen_up();
  float x2 = (x_bottom_left + x_bottom_right)/2.0;
  float y2 = (y_bottom_left/6.0) + ((5.0 * y_top_left)/6.0);
  float y3 = y2 * 1.05;
  move(x2, y3);
  pen_down();
  line(x2, y3, x2, y2);
}

void func_11() {
  float x = x_bottom_right;
  float y = (y_bottom_left/6.0) + ((5.0 * y_top_left)/6.0);
  float y2 = y * 1.05;
  move(x, y2);
  pen_down();
  line(x, y2, x, y);
}

void func_12() {
  float x0 = x_bottom_right;
  float y0 = (y_bottom_left + y_top_left)/2.0;
//  pen_up();
  move(x0,y0);
  pen_down();
  float x = x_bottom_right;
  float y = ((2.0*y_bottom_right)/3.0) + (y_top_right/3.0);
  
  line(x0, y0, x, y); 
}

//draw a diagonal in the middle third
void func_13() {
  float x0 = x_bottom_right;
  float y0 = ((5.0 * y_top_left)/6.0) + y_bottom_left/6.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_left;
  float y = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0;

  line(x0,y0, x, y);
}

void func_14() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_right;
  float y = ((2.0 * y_bottom_right)/3.0) + y_top_right/3.0; 

  line(x0,y0, x, y);

}

void func_15() {
  float x0 = (x_bottom_left + x_bottom_right)/2;
  float y0 = y_top_left; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = (x_bottom_left + x_bottom_right)/2;
  float y = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0; 

  line(x0,y0, x, y); 
}

void func_16() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_left;
  float y = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0; 

  line(x0,y0, x, y);  
}

void func_17() {
  float o1_x = (3.0 * x_bottom_left + x_bottom_right)/4.0;
  float o1_y = (3.0 * y_top_left + y_bottom_left)/4.0;

  float o2_x = (3.0* x_bottom_right + x_bottom_left)/4.0;

  float x1 = x_bottom_left;
  float x2 = (x_bottom_right + x_bottom_left)/2;
  float x3 = x_bottom_right;

//  pen_up();
  move(x1,o1_y);
  pen_down();

  arc(o1_x, o1_y, x1, o1_y, -pi);
  line(x2, o1_y, x2, ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0);
  arc(o2_x, o1_y, x2, o1_y, -pi);
  line(x3, o1_y, x3, ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0);
}

void func_18() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_left;
  float y = y_bottom_left;

  line(x0,y0, x,y);
}

void func_19 () {
  float x0 = x_bottom_right;
  float y0 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

//  pen_up();
  move (x0, y0);
  pen_down();
  float x = x_bottom_right;
  float y = y_bottom_right;

  line(x0,y0, x,y);
}

void func_20() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_right;
  float y = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

  line(x0,y0, x,y);
}

void func_21() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0;
  
//  pen_up();
  move (x0, y0);
  pen_down();
  float x1 = x_bottom_left;
  float y1 = (y_bottom_left + y_top_left)/2.0;
  line(x0, y0, x1, y1);
  
  float x_orig = (x_bottom_left + x_bottom_right)/2.0;
  float y_orig = (y_bottom_left + y_top_left)/ 2.0;
  
  arc(x_orig, y_orig, x1, y1, pi);
}

void func_22() {
  float x_orig1 = (x_bottom_left + x_bottom_right)/2.0;
  float y_orig1 = (y_bottom_left + y_top_left)/ 2.0;

  float x_orig2 = (x_bottom_left + x_bottom_right)/2.0;
  float y_orig2 = ((7.0*y_top_right)/12.0) + ((5.0*y_bottom_right)/12.0);

  float x_orig3 = (x_bottom_left + x_bottom_right)/2.0;
  float y_orig3 = ((5.0*y_top_right)/12.0) + ((7.0*y_bottom_right)/12.0);

  float x1 = ((x_bottom_left + x_bottom_right)/2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  float y1 = ((y_bottom_left + y_top_left)/ 2.0) + ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));

  float x2 = (x_bottom_left + x_bottom_right)/2.0;
  float y2 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

  float x3 = ((x_bottom_left + x_bottom_right)/2.0) - ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));
  float y3 = ((y_bottom_left + y_top_left)/ 2.0) - ((sqrt(2)/12.0) * (y_top_right - y_bottom_right));

  float x4 = (x_bottom_left + x_bottom_right)/2;
  float y4 = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0;
  
//  pen_up();
  move(x1, y1);
  pen_down();
  float theta1 = pi/4;
  float theta2 = pi;

  arc(x_orig1, y_orig1, x1, y1, theta1);
  arc(x_orig2, y_orig2, x2, y2, theta2);
//  pen_up();
  
//  move(x3, y3);
//  pen_down();

  arc(x_orig3, y_orig3, x_orig1, y_orig1, -theta2);
  arc(x_orig1, y_orig1, x4, y4, -theta1);
}

void func_23() {
  float x0 = x_bottom_right;
  float y0 = (y_bottom_left + y_top_left)/2.0;
  float x = x_bottom_right;
  float y = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

//  pen_up();
//  move (x0, y0);
//  pen_down();
  line(x0,y0, x,y);
}

void func_25() {
 float o_x = x_bottom_left;
 float o_y =  ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

 float u_x = ((3.0 * x_bottom_left)/4.0) + (x_bottom_right/4.0);
 float u_y = ((2.0 * y_bottom_left)/3.0) + (y_top_left/3.0);

 float v_x = (x_bottom_left + 3 * x_bottom_right)/4.0;
 float v_y = ((2.0 * y_bottom_left)/3.0) + (y_top_left/3.0);
 

 float p_x = (x_bottom_left + x_bottom_right)/2;
 float p_y = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

 float c_x = x_bottom_right;
 float c_y = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

// pen_up();
 move(o_x, o_y);
 pen_down();

 line(o_x, o_y, u_x, u_y);
 line(u_x, u_y, p_x, p_y);
 line(p_x, p_y, v_x, v_y);
 line(v_x, v_y, c_x, c_y);
 
}
void func_24() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_top_right)/3.0) + y_bottom_right/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = (x_bottom_left + x_bottom_right)/2.0;
  float y = ((2.0*y_bottom_left)/3.0) + (y_top_left/3.0);
  line(x0, y0, x,y);

  float x1 = x_bottom_right;
  float y1 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0); 
  line(x, y, x1, y1);
  
}

void func_26() {
  float x0 = x_bottom_right;
  float y0 = ((2.0*y_top_right)/3.0) + (y_bottom_right/3.0);

//  pen_up();
  move (x0, y0);
  pen_down();
  float x = x_bottom_left;
  float y = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0;

  line(x0,y0, x,y);

}

void func_27() {
  float x0 = x_bottom_left;
  float y0 = ((2.0 * y_bottom_left)/3.0) + y_top_left/3.0; 
//  pen_up();
  move(x0,y0);
  pen_down();

  float x = x_bottom_right;
  float y = ((2.0 * y_bottom_right)/3.0) + y_top_right/3.0; 
  line(x0,y0, x,y);
}

void func_28() {
  float x0 = (x_bottom_left + x_bottom_right)/2.0;
  float y0 = (y_bottom_left + y_top_left)/ 2.0;

  float x = x_bottom_left;
  float y = y0;
//  pen_up();
//  move(x, y);
//  pen_down();
  float theta = 2*pi;

  arc(x0, y0, x, y, -theta);
}

//box ratio y/x : 3/1 //FOR DEBUGGING
void box() {

  line(x_top_left, y_top_left, x_top_right, y_top_right);
  line(x_top_right, y_top_right, x_bottom_right, y_bottom_right);
  line(x_bottom_right, y_bottom_right, x_bottom_left, y_bottom_left);
  line(x_bottom_left, y_bottom_left, x_top_left, y_top_left);
 }

void draw_a() {
  func_1();
  func_2();
  pen_up();    
}

void draw_b(){
  func_3();
  func_28();
  pen_up();
}

void draw_c() {
  func_4();
  pen_up();
}

void draw_d(){
  func_1();
  func_5();  
  pen_up();
}

void draw_e() {
//  func_6();
//  func_7();  
    func_7();
    func_4();
    pen_up();
}

void draw_f() {
  func_8();
  pen_up();
  func_20();
  pen_up();
}

void draw_g() {
  func_1();
  func_9();
  pen_up();
}

void draw_h() {
  func_3();
  func_6();
  func_12();
  pen_up();  
}

void draw_i() {
  func_10();
  pen_up();
}

void draw_j(){
  func_9();
  pen_up(); 
  func_11();
  pen_up();
}

void draw_k(){
  func_3();
  pen_up();
  func_13();
  func_14(); 
  pen_up(); 
}

void draw_l(){
  func_15();
  pen_up();
}

void draw_m() {
  func_16();
  func_17();
  pen_up();
}

void draw_n() {
  func_16();
  func_6();
  func_12();
  pen_up();
}

void draw_o() { 
  func_1();  
  pen_up();
}

void draw_p() {
  func_18();
  func_28();
  pen_up();
}
void draw_q(){
  func_1();
  func_19();
  pen_up();  
}

void draw_r() {
  func_16();
  func_6();
  pen_up();  
}

void draw_s() {
  func_22();
  pen_up();
}

void draw_t() {
  func_15();
  pen_up();
  func_20();
  pen_up();
}

void draw_u() {
  func_21();
  func_23();
  func_12();
  pen_up();  
}

void draw_v() {
  func_24();
  pen_up();    
}

void draw_w() {
  func_25();
  pen_up();  
}

void draw_x() {
  func_14();
  pen_up();  
  func_26();
  pen_up();  
}
void draw_y() {
  func_21();
  func_9();
  pen_up();      
}

void draw_z() {
  func_20();
  func_26();
  func_27();
  pen_up();    
}


//////////////////////////////////////////////////////
void loop() {
    pen_up(); //Always start with pen_up--this is the default state
    
    if (Serial.available() > 0) { //Wait for serial input to do anything.
      String inputWord = Serial.readString();
      inputWord.toLowerCase();
      Serial.println("Writing: " + inputWord + ".");
    
      for (int i = 0; i < inputWord.length(); i++) {
        switch (inputWord[i]) {
          case 'a':
            draw_a();
            break;
          case 'b':
            draw_b();
            break;
          case 'c':
            draw_c();
            break;
          case 'd':
            draw_d();
            break;
          case 'e':
            draw_e();
            break;
          case 'f':
            draw_f();
            break;
          case 'g':
            draw_g();
            break;
          case 'h':
            draw_h();
            break;
          case 'i':
            draw_i();
            break;
          case 'j':
            draw_j();
            break;
          case 'k':
            draw_k();
            break;
          case 'l':
            draw_l();
            break;
          case 'm':
            draw_m();
            break;
          case 'n':
            draw_n();
            break;
          case 'o':
            draw_o();
            break;
          case 'p':
            draw_p();
            break;
          case 'q':
            draw_q();
            break;
          case 'r':
            draw_r();
            break;
          case 's':
            draw_s();
            break;
          case 't':
            draw_t();
            break;
          case 'u':
            draw_u();
            break;
          case 'v':
            draw_v();
            break;
          case 'w':
            draw_w();
            break;
          case 'x':
            draw_x();
            break;
          case 'y':
            draw_y();
            break;
          case 'z':
            draw_z();
            break;
          case ' ':
            break;
        }
        timeLoop(millis(), 1000);
        moveMotors();
      }
    
//  arc(X_HOME, Y_HOME-15, X_HOME + 5, Y_HOME-15, 2.0*pi);

//      draw_h();
//      timeLoop(millis(), 1000);
//      moveMotors();
//      draw_i();
//      timeLoop(millis(), 1000);
//      moveMotors();
//      draw_j();
//      timeLoop(millis(), 1000);
//      moveMotors();    
//      draw_k();
//      timeLoop(millis(), 1000);
//      moveMotors();
    }

    
//    exit(0);
    

//  box();
//  return_to_home();
//  timeLoop(millis(),3000);
  }

void loop2() {
  // Computed joint_path, which will be the input to servo (it's a sequence of joint angles)
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

