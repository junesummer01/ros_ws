
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

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.x_data, self.y_data, 'r-')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Odometry Path')
        self.ax.grid(True)
        
        self.timer = self.create_timer(0.1, self.update_plot)

    def odometry_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)

    def update_plot(self):
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)
        self.ax.relim()
        self.ax.autoscale_view()
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
