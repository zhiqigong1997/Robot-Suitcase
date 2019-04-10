import math
import rospy
import Twist


def euclidean_distance(xd, yd):
    return math.sqrt(math.pow(xd, 2) + math.pow(yd, 2))


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class PID:

    def __init__(self, kp=0.5, kd=0.0, ki=0.0, dt=0.1):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self.err_acc = 0
        self.err_previous = 0.1

    def compute_pid(self, err):
        # Integral"
        self.err_acc = self.err_acc + self.dt * (err + self.err_previous) / 2
        # Derivative
        err_deriv = (err - self.err_previous) / self.dt
        # PID
        pid = self.kp * err + self.kd * err_deriv + self.ki * self.err_acc
        # Update Error
        self.err_previous = err
        return pid


class Current:
    def __init__(self):
        # Robot Current Condition
        self.x = 0
        self.y = 0
        self.theta = 0


class Controller:

    def __init__(self):
        # Subscriber
        self.velocity_publisher = rospy.Publisher("/Velocity", Twist, queue_size=1)
        self.goal_subscriber = rospy.Subscriber("/Goal", Goal, self.move_to_goal)
        # self data
        self.hz = 20
        self.dt = (1.0 / self.hz)
        self.L = 20  # Wheel Base: How far are two wheels
        self.R = 20  # Wheel Radius

        self.current = Current()

        # define pid for rho, alpha, beta
        self.pid_rho = PID(kp=0.5, dt=self.dt)
        self.pid_alpha = PID(kp=1, dt=self.dt)
        self.pid_beta = PID(kp=0.5, dt=self.dt)

    def update_self_position(self):
        # pos = msg.pose.pose.position
        # ori = msg.pose.pose.orientation
        self.current.x = 0
        self.current.y = 0
        self.current.theta = 0

    def move_to_goal(self, goal):
        threshold = 1
        goal_x = goal.position.x
        goal_y = goal.position.y
        goal_theta = goal.theta

        # compute distance
        rho = euclidean_distance((goal_x - self.current.x), (goal_y - self.current.y))

        vel_msg = Twist()

        while rho >= threshold:
            # errors
            self.update_self_position()

            # rho,alpha,beta
            rho = euclidean_distance((goal_x - self.current.x), (goal_y - self.current.x))
            alpha = angle_diff(math.atan2((goal_y - self.current.x), (goal_x - self.current.x)), self.current.theta)
            beta = angle_diff(angle_diff(goal_theta, self.current.theta), alpha)

            # compute pid
            v = self.pid_rho.compute_pid(rho)
            w = self.pid_alpha.compute_pid(alpha) - self.pid_beta.compute_pid(beta)

            # publish vr and vl
            vel_msg.vr = (2 * v + w * self.L) / (2 * self.R)
            vel_msg.vl = (2 * v - w * self.L) / (2 * self.R)
            self.velocity_publisher.publish(vel_msg)

            # debug info
            print("Right wheel Velocity%a\n", vel_msg.vr)
            print("Left wheel Velocity%a\n", vel_msg.vl)
            print("_________________\n")

        # Halt the robot
        vel_msg.vr = 0
        vel_msg.vl = 0
        self.velocity_publisher.publish(vel_msg)

        # debug info
        print("robot stop!\n")
        print("Right wheel Velocity%a\n", vel_msg.vr)
        print("Left wheel Velocity%a\n", vel_msg.vl)
        print("_________________\n")
