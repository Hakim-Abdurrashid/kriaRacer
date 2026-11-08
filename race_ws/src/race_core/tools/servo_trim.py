#!/usr/bin/env python3
"""Steer the servo by hand from the joystick to find straight ahead and the two steering stops.

Run with only vesc_driver_node and joy_node up (nothing else may publish /commands/servo/position):
  python3 servo_trim.py
Buttons: B +0.01 (right), X -0.01 (left), hold to repeat; Y back to 0.50; A marks the current value.
Every change and mark is printed; servo value -> vesc.yaml steering_angle_to_servo_offset / servo_min / servo_max.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

A, B, X, Y = 0, 1, 2, 3
STEP, REPEAT_DELAY, REPEAT_PERIOD = 0.01, 0.4, 0.1


class ServoTrim(Node):
    def __init__(self):
        super().__init__('servo_trim')
        self.value = 0.50
        self.prev = [0] * 4
        self.held_since = {}
        self.last_repeat = {}
        self.pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.05, self.tick)
        print(f'servo {self.value:.2f}  (B right, X left, Y centre 0.50, A mark)', flush=True)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def move(self, delta):
        self.value = min(1.0, max(0.0, round(self.value + delta, 3)))
        print(f'servo {self.value:.2f}', flush=True)

    def on_joy(self, msg):
        b = list(msg.buttons[:4]) + [0] * (4 - len(msg.buttons[:4]))
        t = self.now()
        for i, d in ((B, STEP), (X, -STEP)):
            if b[i] and not self.prev[i]:
                self.move(d)
                self.held_since[i] = self.last_repeat[i] = t
            elif not b[i]:
                self.held_since.pop(i, None)
        if b[Y] and not self.prev[Y]:
            self.value = 0.50
            print('servo 0.50 (centre)', flush=True)
        if b[A] and not self.prev[A]:
            print(f'MARK {self.value:.2f}', flush=True)
        self.prev = b

    def tick(self):
        t = self.now()
        for i, d in ((B, STEP), (X, -STEP)):
            if i in self.held_since and t - self.held_since[i] > REPEAT_DELAY \
                    and t - self.last_repeat[i] > REPEAT_PERIOD:
                self.move(d)
                self.last_repeat[i] = t
        self.pub.publish(Float64(data=self.value))


def main():
    rclpy.init()
    rclpy.spin(ServoTrim())


if __name__ == '__main__':
    main()
