#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, UInt8MultiArray
from geometry_msgs.msg import Vector3
import serial
import json
from datetime import datetime

class PuzzleBoardNode(Node):
    def __init__(self):
        super().__init__('puzzle_board_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0') # Default port, specify port in arguments '--port'
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('debug', True)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial_port = None
        
        # Publishers
        self.joystick_pub = self.create_publisher(Vector3, '/puzzle_board/joystick', 10)
        self.buttons_pub = self.create_publisher(Int32MultiArray, '/puzzle_board/buttons', 10)
        self.leds_pub = self.create_publisher(UInt8MultiArray, '/puzzle_board/leds', 10)
        self.display_pub = self.create_publisher(String, '/puzzle_board/display', 10)
        self.puzzle_state_pub = self.create_publisher(String, '/puzzle_board/state', 10)
        self.puzzle_generation_pub = self.create_publisher(String, '/puzzle/generation', 10)
        
        # Create timer to read from Arduino
        self.timer = self.create_timer(0.05, self.read_arduino)  # 20Hz
        
        # Store current state
        self.current_state = {
            'joystick_x': 0.0,
            'joystick_y': 0.0,
            'buttons': [0] * 12,  # 4x3 button matrix
            'leds': [0] * 64,     # 8x8 LED matrix
            'display_text': '',
            'puzzle_solved': False,
            'timestamp': datetime.now().isoformat()
        }
        
        self.get_logger().info("Puzzle Board Node initialized and waiting for data...")
    
    def read_arduino(self):
        """Read data from Arduino serial port"""
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if line:
                    self.parse_and_publish(line)
        
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
    
    def parse_and_publish(self, data):
        """Parse Arduino data and publish to appropriate topics"""
        try:
            # Expected format from Arduino (can be adjusted):
            # J:512,768,1|B:1,0,0,0,0,0,0,0,0,0,0,0|L:11110000,11110000,...|T:HELLO|P:0
            # J = Joystick (VRx, VRy, SW)
            # B = Buttons (12 values)
            # L = LEDs (8x8 matrix as binary strings)
            # T = Display text
            # P = Puzzle solved flag
            
            if self.debug:
                self.get_logger().info(f"Raw data: {data}")
            
            parts = data.split('|')
            
            for part in parts:
                if part.startswith('J:'):
                    self.handle_joystick(part[2:])
                elif part.startswith('B:'):
                    self.handle_buttons(part[2:])
                elif part.startswith('L:'):
                    self.handle_leds(part[2:])
                elif part.startswith('T:'):
                    self.handle_display(part[2:])
                elif part.startswith('P:'):
                    self.handle_puzzle_state(part[2:])
                elif part.startswith('CODE:'):
                    self.handle_puzzle_generation(data)  # Pass the entire data string to be handled
            
            # Publish combined state
            self.publish_puzzle_state()
        
        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    def publish_puzzle_generation(self, puzzle_data):
        """Publish the puzzle generation state once to the topic"""
        try:
            msg = String()
            msg.data = json.dumps(puzzle_data)
            
            self.puzzle_generation_pub.publish(msg)
            self.get_logger().info(f"Puzzle generation published: {puzzle_data}")
        
        except Exception as e:
            self.get_logger().error(f"Puzzle generation publish error: {e}")

    def handle_puzzle_generation(self, data):
        """Parse and handle puzzle generation data from Arduino"""
        try:
            # Expected format: "CODE:123456|MAZE:row0,row1,...|MAZE_END:x,y"
            # Example: "CODE:123456|MAZE:11110000,00001111,...|MAZE_END:7,7"
            
            puzzle_gen = {}
            
            # Split by | to get different parts
            parts = data.split('|')
            
            for part in parts:
                if part.startswith('CODE:'):
                    puzzle_gen['code'] = part[5:]  # Extract code value
                
                elif part.startswith('MAZE:'):
                    maze_str = part[5:]  # Extract maze data
                    maze_rows = maze_str.split(',')
                    
                    maze_matrix = []
                    for row in maze_rows[:8]:  # Limit to 8 rows
                        row_values = []
                        for bit in row[:8]:  # Limit to 8 bits per row
                            row_values.append(int(bit))
                        maze_matrix.append(row_values)
                    
                    puzzle_gen['maze'] = maze_matrix
                
                elif part.startswith('MAZE_END:'):
                    coords = part[9:].split(',')  # Extract x,y coordinates
                    if len(coords) == 2:
                        puzzle_gen['maze_end_x'] = int(coords[0])
                        puzzle_gen['maze_end_y'] = int(coords[1])
            
            # Add timestamp
            puzzle_gen['timestamp'] = datetime.now().isoformat()
            
            # Publish puzzle generation
            self.publish_puzzle_generation(puzzle_gen)
            
            # Print visual maze for debugging
            if self.debug and 'maze' in puzzle_gen:
                self.print_maze(puzzle_gen['maze'], puzzle_gen.get('maze_end_x', -1), puzzle_gen.get('maze_end_y', -1))
                self.get_logger().info(f"Puzzle Code: {puzzle_gen.get('code', 'N/A')}")
                self.get_logger().info(f"Maze End Position: ({puzzle_gen.get('maze_end_x', -1)}, {puzzle_gen.get('maze_end_y', -1)})")
        
        except Exception as e:
            self.get_logger().error(f"Puzzle generation parse error: {e}")

    def print_maze(self, maze, end_x=-1, end_y=-1):
        """Print maze in visual 8x8 format"""
        try:
            output = "\n╔════════════════╗\n║  Maze Pattern  ║\n╠════════════════╣\n"
            
            for row_idx, row in enumerate(maze):
                output += "║"
                for col_idx, cell in enumerate(row):
                    if row_idx == end_y and col_idx == end_x:
                        # Mark the end position
                        output += " ✓"
                    elif cell == 1:
                        output += " █"
                    else:
                        output += " ·"
                output += " ║\n"
            
            output += "╚════════════════╝"
            self.get_logger().info(output)
        
        except Exception as e:
            self.get_logger().error(f"Maze display error: {e}")
    
    def handle_joystick(self, data):
        """Parse and publish joystick data"""
        try:
            # Expected: "512,768,1" (VRx, VRy, SW)
            values = data.split(',')
            if len(values) >= 2:
                # Normalize from 0-1023 to -1.0 to 1.0
                vrx = (int(values[0]) - 512) / 512.0
                vry = (int(values[1]) - 512) / 512.0
                
                msg = Vector3()
                msg.x = float(vrx)
                msg.y = float(vry)
                
                self.joystick_pub.publish(msg)
                self.current_state['joystick_x'] = vrx
                self.current_state['joystick_y'] = vry
                
                if self.debug:
                    self.get_logger().info(f"Joystick - X: {vrx:.2f}, Y: {vry:.2f}")
        
        except Exception as e:
            self.get_logger().error(f"Joystick parse error: {e}")
    
    def handle_buttons(self, data):
        """Parse and publish button data"""
        try:
            # Expected: "1,0,0,0,0,0,0,0,0,0,0,0" (12 button states)
            button_values = [int(x) for x in data.split(',')]
            
            if len(button_values) >= 12:
                button_values = button_values[:12]  # Limit to 12 buttons
                
                msg = Int32MultiArray()
                msg.data = button_values
                
                self.buttons_pub.publish(msg)
                self.current_state['buttons'] = button_values
                
                if self.debug and any(button_values):  # Only log if button pressed
                    pressed_buttons = [i for i, v in enumerate(button_values) if v == 1]
                    self.get_logger().info(f"Buttons pressed: {pressed_buttons}")
        
        except Exception as e:
            self.get_logger().error(f"Button parse error: {e}")

    def handle_leds(self, data):
        """Parse and publish LED matrix data"""
        try:
            # Expected: "11110000,11110000,..." (8 rows of 8 bits each)
            rows = data.split(',')
            
            if len(rows) >= 8:
                led_values = []
                matrix = []  # Store as 8x8 for visual display
                
                for row in rows[:8]:
                    row_values = []
                    for bit in row[:8]:
                        value = int(bit)
                        led_values.append(value)
                        row_values.append(value)
                    matrix.append(row_values)
                
                msg = UInt8MultiArray()
                msg.data = led_values[:64]  # Limit to 64 LEDs
                
                self.leds_pub.publish(msg)
                self.current_state['leds'] = led_values
                
                if self.debug:
                    self.get_logger().info(f"LED Matrix updated: {len(led_values)} LEDs")
                    self.print_led_matrix(matrix)
        
        except Exception as e:
            self.get_logger().error(f"LED parse error: {e}")
    
    def print_led_matrix(self, matrix):
        """Print LED matrix in visual 8x8 format"""
        try:
            output = "\n╔════════════════╗\n║ LED Matrix 8x8 ║\n╠════════════════╣\n"
            
            for row_idx, row in enumerate(matrix):
                output += "║"
                for col_idx, led in enumerate(row):
                    # Use █ for ON (1) and · for OFF (0)
                    output += " █" if led == 1 else " ·"
                output += " ║\n"
            
            output += "╚════════════════╝"
            self.get_logger().info(output)
        
        except Exception as e:
            self.get_logger().error(f"LED display error: {e}")


    def handle_display(self, data):
        """Parse and publish alphanumeric display data"""
        try:
            msg = String()
            msg.data = data.strip()
            
            self.display_pub.publish(msg)
            self.current_state['display_text'] = data
            
            if self.debug:
                self.get_logger().info(f"Display: {data}")
        
        except Exception as e:
            self.get_logger().error(f"Display parse error: {e}")
    
    def handle_puzzle_state(self, data):
        """Parse puzzle solved state"""
        try:
            puzzle_solved = int(data) == 1
            self.current_state['puzzle_solved'] = puzzle_solved
            
            if puzzle_solved:
                self.get_logger().info("🎉 PUZZLE SOLVED! 🎉")
            
            if self.debug:
                self.get_logger().info(f"Puzzle solved: {puzzle_solved}")
        
        except Exception as e:
            self.get_logger().error(f"Puzzle state parse error: {e}")
    
    def publish_puzzle_state(self):
        """Publish the complete puzzle state as JSON"""
        try:
            self.current_state['timestamp'] = datetime.now().isoformat()
            
            msg = String()
            msg.data = json.dumps(self.current_state)
            
            self.puzzle_state_pub.publish(msg)
        
        except Exception as e:
            self.get_logger().error(f"State publish error: {e}")
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    puzzle_board_node = PuzzleBoardNode()
    
    try:
        rclpy.spin(puzzle_board_node)
    except KeyboardInterrupt:
        puzzle_board_node.get_logger().info("Shutting down Puzzle Board Node...")
    finally:
        puzzle_board_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()