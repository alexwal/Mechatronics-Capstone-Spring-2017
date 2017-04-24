void pen_down() {
  // Pen down.
  lift_servo.writeMicroseconds(lift_servo_write);
}

void pen_up() {
  // Pen up.
  lift_servo.writeMicroseconds(lift_servo_pause);
}

// Draw a CCW arc originating from x, y with radius r, origin x0, y0.
void arc(float x0, float y0, float x, float y, float theta):
        //move to x, y --> the poit from which the arc will begin
        move(x, y)
        pen_down();
        //initalize vars
        n = 20; //number of delta theta steps to take to draw arc
        double del_theta = theta/n;
        double x_cur = x;
        double y_cur = y;
        // this loop moves the arc along for n iterations
        int i = 0;
        while (i < n) {
          //shift x1, y1 so that it is with respect to (0,0) origin
          double x_shifted = x_cur - x0;
          double y_shifted = y_cur = y0;

          //rotate the the shifted point del_theta radians
          double x_rot = x_shifted * cos(del_theta) - y_shifted * sin(del_theta)
          double y_rot = x_shifted * sin(del_theta) + y_shifted * cos(del_theta)

          //shift point back to original origin (x0, y0)
          double x_rot_shifted = x_rot + x0
          double y_rot_shifted = y_rot + y0

          //actually do the move to the intermediate point
          move(x_rot_shifted, y_rot_shifted);

          x_cur = x_rot_shifted;
          y_cur = y_rot_shifted; 
        }
        
        
        pen_up();
}

void line(x0, y0, x1, y1):
        // Draw a line.
        move(x0, y0);
        pen_down();
        move(x1, y1);
        pen_up();
}
