import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import subprocess
import shlex
import signal
import psutil

class CommandHandlerNode(Node):
    def __init__(self):
        super().__init__('command_handler_node')

        self.subscription = self.create_subscription(
            String,
            'node_control_topic',
            self.command_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.processes = {}  

    def send_zero_velocity(self):
        zero_cmd = Twist()
        zero_cmd.linear.x = 0.0
        zero_cmd.linear.y = 0.0
        zero_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(zero_cmd)
        self.get_logger().info('Sent zero velocity command')

    def command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f'Received command: {command}')

        if command == 'clear_all':
            self.clear_all_processes()
        elif command.startswith('ros2 launch') or command.startswith('ros2 run'):
            self.start_process(command)
        elif command.startswith('stop '):
            stop_command = command.replace('stop ', '', 1).strip()
            self.stop_process(stop_command)
        else:
            self.get_logger().warn('Invalid command received.')

    def clear_all_processes(self):
        try:
            self.get_logger().info('Clearing all running processes...')
            commands = list(self.processes.keys()) 
            
            for command in commands:
                self.send_zero_velocity() 
                self.stop_process(command)
            
            self.get_logger().info('All processes cleared successfully!')
        except Exception as e:
            self.get_logger().error(f'Error while clearing processes: {e}')

    def start_process(self, command):
        try:
            if command in self.processes:
                self.get_logger().warn(f'Process with command "{command}" is already running.')
                return
            args = shlex.split(command)
            process = subprocess.Popen(args, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))
            self.processes[command] = process
            self.get_logger().info(f'Started process: {command}')
        except Exception as e:
            self.get_logger().error(f'Failed to start process: {e}')

    def stop_process(self, command):
        if command in self.processes:
            try:
                process = self.processes[command]
                parent = psutil.Process(process.pid)
                self.send_zero_velocity() 

                for child in parent.children(recursive=True):
                    child.terminate()

                parent.terminate()
                gone, alive = psutil.wait_procs(parent.children(), timeout=3)

                if alive:
                    for p in alive:
                        p.kill()
                if parent.is_running():
                    parent.kill()

                del self.processes[command]
                self.get_logger().info(f'Process "{command}" stopped successfully.')
            except (psutil.NoSuchProcess, psutil.AccessDenied, Exception) as e:
                self.get_logger().error(f'Failed to stop process: {e}')

                if command in self.processes:
                    del self.processes[command]
        else:
            self.get_logger().warn(f'No process found for command: {command}')

    def stop_all_processes(self):
        commands = list(self.processes.keys())  
        for command in commands:
            self.stop_process(command)

def main(args=None):
    rclpy.init(args=args)
    node = CommandHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.stop_all_processes()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()