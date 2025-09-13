
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class OdometryVisualizer(Node):
    def __init__(self):
        super().__init__('odometry_visualizer')
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odometry_callback,
            10)
        self.x_data = []
        self.y_data = []
        self.max_path_length = 100

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.x_data, self.y_data, 'r-')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Odometry Path')
        self.ax.grid(True)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        
        self.timer = self.create_timer(0.1, self.update_plot)

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)

        if len(self.x_data) > self.max_path_length:
            self.x_data.pop(0)
            self.y_data.pop(0)

    def update_plot(self):
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    odometry_visualizer = OdometryVisualizer()
    try:
        rclpy.spin(odometry_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_visualizer.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
