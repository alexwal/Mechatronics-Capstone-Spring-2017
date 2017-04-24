void pen_down() {
  // Pen down.
  lift_servo.writeMicroseconds(lift_servo_write);
}

void pen_up() {
  // Pen up.
  lift_servo.writeMicroseconds(lift_servo_pause);
}

void arc(x0, y0, x, y, r, theta):
        // Draw a CCW arc originating from x, y with radius r, origin x0, y0.
        move(x0 + r, y0); // move to 0 degrees. (! not theta0)
        pen_down();

        int STEPS = 20;
        // 1. translate x, y to about 0,0
        
        theta_prime = atan2(x-x0, y-y0);
        // 2. Get arc step end position.
        // X. rotate back by theta_prime.

        delta_theta = theta/STEPS;
        
        for (int i = 0; i < STEPS; i++) {
          // get next position corr to shift of delta theta
          x_next = next_x_position(x_cur, y_cur);
          y_next = next_y_position(x_cur, y_cur);
          // move to that next position.
          move(x_next, y_next);
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
