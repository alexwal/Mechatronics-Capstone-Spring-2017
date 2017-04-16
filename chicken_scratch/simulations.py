# Mechatronics 102B
# Simulation of joint angle path
# required to move from one point
# to another.
from __future__ import division
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
c, s = np.cos, np.sin
acos, asin = np.arccos, np.arcsin
pinv = np.linalg.pinv
pi = np.pi
mag = np.linalg.norm # L2 norm

'''
    Assuming servos lined up on horizontal axis,
    and theta drawn counterclockwise and in radians.

    [L], [R]    : servo  coordinates.
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
'''

# Fixed robot parameters. (units?)
a = 1 # See diagram.
b = 4 # See diagram. 
l1 = 2 # | [A] - [L] |
l2 = 3 # | [q] - [A] |
l3 = 3 # | [q] - [B] |
l4 = 2 # | [B] - [R] |

midpoint = a + (b-a) / 2
assert l2 + l3 > midpoint, 'impossible dimensions: arms disconnected!'

# Corners of end effector space. See diagram.
extra = 1
C1 = (a - l1, 0) # function of lis?
C2 = (a - l1, l1 + l2 + extra)
C3 = (b + l4, l3 + l4 + extra)
C4 = (b + l4, 0)

# Jacobians.
def compute_Jq(x, y, t1, t2):
  # At:
  # End effector position:  q = (x, y),
  # Joint angles         :  ti = theta_i.
  Jq = np.array([
                  [-l1 * c(t1) + x - a, y - l1 * s(t1)],
                  [-l4 * c(t2) + x - b, y - l4 * s(t2),]
                ]) * 2
  return Jq

def compute_Jtheta(x, y, t1, t2):
  # At:
  # End effector position:  q = (x, y),
  # Joint angles         :  ti = theta_i.
  Jtheta = np.array([
                      [l1 * s(t1) * (x - a) - l1 * c(t1) * y, 0],
                      [0, l4 * s(t2) * (x - b) - l4 * c(t2) * y]
                    ]) * 2
  return Jtheta

def compute_Jacobian(x, y, t1, t2):
  # At:
  # End effector position:  q = (x, y),
  # Joint angles         :  ti = theta_i.
  # Main idea:
  #     Jq * q' = -Jtheta * theta'.
  #     => theta' = J * q' = -pseudoinv(Jtheta) * Jq * q'.
  #         ^joint velocity                            ^end effector velocity
  Jq = compute_Jq(x, y, t1, t2)
  Jtheta = compute_Jtheta(x, y, t1, t2)
  J = -np.dot(pinv(Jtheta), Jq)
  return J

def move(qdes, q0, theta0, tolerance=0.1, alpha=0.2, decay=1e-3):
  '''
  Fill in. 
  alpha = step size.
  '''
  q_cur = np.array(q0)
  theta_cur = np.array(theta0)
  qdes = np.array(qdes)
  joint_path = list() # { theta0, theta1, theta2, theta3, ... } Sequence of servo angles.
  q_path = list() # { q0, q1, q2, q3, ... } Resulting end effector path.
  joint_path.append(theta0) 
  q_path.append(q0) 
  iterations = 0
  while mag(qdes - q_cur) > tolerance and iterations < 100:
    if iterations % 20 == 0:
      print 'Iteration ' + str(iterations) + ' error:', mag(qdes - q_cur)
    x, y = q_cur[0], q_cur[1]
    t1, t2 = theta_cur[0], theta_cur[1]
    J = compute_Jacobian(x, y, t1, t2)
    dqdt = qdes - q_cur # NOTE: dqdt gets smaller as time goes on, maybe want to keep constant step?
                        # i.e., make mag(dqdt) always equal 1? --> scale.
    if mag(dqdt) > 0.01: # avoid numerical issues in denom, results in constant step size.
      dqdt = dqdt/mag(dqdt)
    dthetadt = J.dot(dqdt)
    theta_cur = theta_cur + alpha * dthetadt

    # Move servos to theta_cur 
    q_cur = q_cur + alpha * dqdt # best guess of current pos (without using FK eqn).

    joint_path.append(theta_cur)
    q_path.append(q_cur)

    iterations += 1

    # decay step size over time
    if iterations % 10 == 0:
      alpha = alpha * (1 - decay)

  print 'Final iteration (' + str(iterations) + ') error:', mag(qdes - q_cur)
  print iterations
  return joint_path, q_path

def drawArrow(A, B, width=0.01, ax=plt):
      ax.arrow(A[0], A[1], B[0] - A[0], B[1] - A[1],
               head_width=width, length_includes_head=True, zorder=10)

if __name__ == '__main__':

  # Example.
  # Start q0 at point where servo arms at 90 deg and
  # end eff pointing away from servos. (is it even (always) possible?)
  x0 = midpoint 
  y0 = l1 + np.sqrt(l2**2 - (midpoint - a)**2)
  t1_0, t2_0 = pi/2, pi/2 

  # Preview Jacobian at initial configuration.
  J = compute_Jacobian(x0, y0, t1_0, t2_0)
  print J

  # Format q0, theta0, and qdes.
  theta0 = (t1_0, t2_0)
  q0 = (x0, y0) # depends on theta0!
  qdes = (5, 3)

  # Given the initial conditions and desired point,
  # compute the required joint path and resulting
  # end effector path.
  joint_path, q_path = move(qdes, q0, theta0)

  # Break down paths into path for each of { left servo, right servo, x coordinate, y coordinate }
  # This makes it easier to plot the paths.
  t1s = [t[0] for t in joint_path]
  t2s = [t[1] for t in joint_path]
  xs = [q[0] for q in q_path]
  ys = [q[1] for q in q_path]

  #############
  # Plotting. #
  #############

  # Make two subplots
  f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)

  # Configure subplot I: End effector path
  ax1.set_aspect('equal')
  ax1.set_title('End effector path')
  ax1.set_xlabel('x')
  ax1.set_ylabel('y')

  # Draw shapes for "desk", servo positions, links.
  workspace_rect = plt.Rectangle(C1, C3[0] - C1[0], C3[1] - C1[1], fc='lightgreen', zorder=1)
  circle1 = plt.Circle((a, 0), 0.1, fc='red', zorder=2) # left servo
  circle2 = plt.Circle((b, 0), 0.1, fc='red', zorder=2) # right servo
  ax1.add_patch(circle1)
  ax1.add_patch(circle2)
  ax1.add_patch(workspace_rect)

  # Draw lines for links.
  L_ = (a, 0)
  A_ = (a, l1)
  B_ = (b, l4)
  R_ = (b, 0)
  Q_ = (x0, y0)
  ax1.plot([L_[0], A_[0]], [L_[1], A_[1]], 'k-', linewidth=2) # [x0, x1] , [y0, y1]
  ax1.plot([Q_[0], A_[0]], [Q_[1], A_[1]], 'k-', linewidth=2)
  ax1.plot([Q_[0], B_[0]], [Q_[1], B_[1]], 'k-', linewidth=2)
  ax1.plot([R_[0], B_[0]], [R_[1], B_[1]], 'k-', linewidth=2)

  ax1.scatter(xs, ys)
  # ax1.plot(xs, ys) # shows connected lines, no arrows
  for i in range(len(q_path) - 1):
    q0 = q_path[i]
    q1 = q_path[i + 1]
    drawArrow(q0, q1, width=0.1, ax=ax1)
  
  # Configure subplot II: Joint path.
  ax2.set_aspect('equal')
  ax2.set_title('Joint path')
  ax2.set_xlabel('theta_1')
  ax2.set_ylabel('theta_2')
  ax2.scatter(t1s, t2s)
  for i in range(len(joint_path) - 1):
    t0 = joint_path[i]
    t1 = joint_path[i + 1]
    drawArrow(t0, t1, width=0.05, ax=ax2)

  # Show plot.
  plt.show()

