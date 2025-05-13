import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float32
import pygame
import sys

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')

        # Create individual publishers
        self.publisher_x = self.create_publisher(Float32, 'ControlMode/control/x', 10)
        self.publisher_y = self.create_publisher(Float32, 'ControlMode/control/y', 10)
        self.publisher_z = self.create_publisher(Float32, 'ControlMode/control/z', 10)
        self.publisher_r = self.create_publisher(Float32, 'ControlMode/control/r', 10)

        try:
            pygame.init()
            self.screen = pygame.display.set_mode((400, 300))
            pygame.display.set_caption("Keyboard Control")
        except pygame.error as e:
            self.get_logger().error(f"Failed to initialize pygame: {str(e)}")
            rclpy.shutdown()
            sys.exit(1)

        self.timer = self.create_timer(0.1, self.publish_key_events)
        self.pressed_keys = {
            "w": 0.0, "a": 0.0, "s": 0.0, "d": 0.0,  # x, y control
            "q": 0.0, "e": 0.0,  # z control
            "g": 0.0, "h": 0.0   # rotation control
        }

        self.get_logger().info("Keyboard Publisher Node initialized")

    def publish_key_events(self):
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.cleanup()
                    return

                elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    value = 1.0 if event.type == pygame.KEYDOWN else 0.0
                    key_mapping = {
                        pygame.K_w: ("w", value),
                        pygame.K_a: ("a", -value),
                        pygame.K_s: ("s", -value),
                        pygame.K_d: ("d", value),
                        pygame.K_q: ("q", value), 
                        pygame.K_e: ("e", -value),
                        pygame.K_g: ("g", -value),
                        pygame.K_h: ("h", value)
                    }

                    if event.key in key_mapping:
                        key, val = key_mapping[event.key]
                        self.pressed_keys[key] = val
                        self.get_logger().info(f"{key.upper()} key, value: {val}")

            # Publish values based on pressed keys
            self.publish_value(self.publisher_x, self.pressed_keys["w"] + self.pressed_keys["s"])
            self.publish_value(self.publisher_y, self.pressed_keys["a"] + self.pressed_keys["d"])
            self.publish_value(self.publisher_z, self.pressed_keys["q"] + self.pressed_keys["e"])
            self.publish_value(self.publisher_r, self.pressed_keys["g"] + self.pressed_keys["h"])

            # Update pygame display
            self.screen.fill((200, 200, 200))  # Grey background
            pygame.display.flip()

        except Exception as e:
            self.get_logger().error(f"Error in publish_key_events: {str(e)}")

    def publish_value(self, publisher, value):
        try:
            msg = Float32()
            msg.data = float(value)
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing value: {str(e)}")

    def cleanup(self):
        """Cleanup function for proper shutdown"""
        self.get_logger().info("Shutting down keyboard publisher...")
        pygame.quit()
        rclpy.shutdown()

    def __del__(self):
        """Destructor to ensure pygame is properly quit"""
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = KeyboardPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        if 'node' in locals():
            node.cleanup()
            node.destroy_node()

if __name__ == '__main__':
    main()