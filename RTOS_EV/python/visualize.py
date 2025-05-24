import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


#!/usr/bin/env python3

import time
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

MAX_LIST_LEN = 500
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

class SerialDataPlotter:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, window_size=50):
        """
        Initialize the serial data plotter.
        
        :param port: Serial port to read from (default: '/dev/ttyUSB0')
        :param baudrate: Baud rate for serial communication (default: 9600)
        :param window_size: Number of data points to display (default: 500)
        """
        # Open serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

        # Initialize data storage
        self.window_size = window_size
        self.data = np.zeros(window_size)
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line, = self.ax.plot(self.data)
        
        # Configure plot appearance
        self.ax.set_title('Live PID Graph')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Speed')
        self.ax.set_ylim(0, 5)  # Adjust based on expected data range
        self.ax.axhline(y=3, color='r', linestyle='--', label='y = 3.0')

        
        # Create animation
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)

    def read_serial_data(self):
        """
        Read a single data point from serial port.
        
        :return: Integer value from serial port or None if error
        """
        try:
            # Read a line from serial and convert to integer
            line = self.ser.readline().decode('utf-8').strip()
            return int(line)/100 if line else None
        except (ValueError, serial.SerialException):
            return None

    def update_plot(self, frame):
        """
        Update the plot with new serial data.
        
        :param frame: Animation frame (automatically provided by FuncAnimation)
        """
        # Shift data left and add new point
        self.data = np.roll(self.data, -1)
        
        # Read new data point
        new_data = self.read_serial_data()
        
        # Update last element with new data if available
        if new_data is not None:
            self.data[-1] = new_data
        
        # Update plot
        self.line.set_ydata(self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        
        return self.line,

    def run(self):
        """
        Start the live plotting.
        """
        plt.tight_layout()
        plt.show()

    def __del__(self):
        """
        Cleanup method to close serial connection.
        """
        if hasattr(self, 'ser'):
            self.ser.close()

# Example usage
if __name__ == '__main__':
    try:
        # You may need to adjust the port and baudrate
        plotter = SerialDataPlotter(port='/dev/ttyACM0', baudrate=115200)
        plotter.run()
    except Exception as e:
        print(f"Error: {e}")