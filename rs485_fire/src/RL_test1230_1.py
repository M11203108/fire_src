from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct
import keyboard

def float_to_hex(f):
    packed = struct.pack('!f', f)  # Convert float to 4-byte IEEE 754
    return packed.hex()

def registers_to_float(high, low):
    packed = struct.pack('!HH', high, low)  # Convert two 16-bit registers to float
    return struct.unpack('!f', packed)[0]

class MotorController:
    def __init__(self, port, unit, resolution=3600 * 4):
        self.client = ModbusClient(
            method='rtu',
            port=port,
            baudrate=9600,
            parity='N',
            stopbits=2,
            bytesize=8,
            timeout=3
        )
        self.unit = unit
        self.resolution = resolution

    def connect(self):
        if self.client.connect():
            print("Connected")
        else:
            raise ConnectionError("Cannot connect to Modbus device")

    def close(self):
        self.client.close()
        print("Connection closed")

    def enable_motor(self):
        self.client.write_register(0x004F, 0x0001, unit=self.unit)
        print("Motor enabled")

    def disable_motor(self):
        self.client.write_register(0x004F, 0x0000, unit=self.unit)
        print("Motor disabled")

    def set_rpm(self, rpm):
        ieee_rpm = float_to_hex(rpm)
        self.client.write_register(0x0053, int('0x' + ieee_rpm[0:4], 16), unit=self.unit)
        self.client.write_register(0x0054, int('0x' + ieee_rpm[4:8], 16), unit=self.unit)
        print(f"Set RPM to {rpm}")

    def set_direction(self, direction):
        self.client.write_register(0x0050, direction, unit=self.unit)
        print("Set direction")

    def set_circle(self, circle):
        ieee_circle = float_to_hex(circle)
        self.client.write_register(0x0055, int('0x' + ieee_circle[0:4], 16), unit=self.unit)
        self.client.write_register(0x0056, int('0x' + ieee_circle[4:8], 16), unit=self.unit)
        print(f"Set circle to {circle}")

    def read_encoder(self):
        position_high = self.client.read_holding_registers(0x0007, 1, unit=self.unit).registers[0]
        position_low = self.client.read_holding_registers(0x0008, 1, unit=self.unit).registers[0]
        encoder_value = (position_high << 16) | position_low
        if encoder_value & 0x80000000:  # Handle two's complement for negative values
            encoder_value = -(0xFFFFFFFF - encoder_value + 1)
        current_circle = encoder_value / self.resolution
        return current_circle

    def move_to_position(self, target_circle, rpm):
        self.enable_motor()
        self.set_rpm(rpm)
        self.set_circle(target_circle)

        while True:
            current_circle = self.read_encoder()
            print(f"Current position: {current_circle:.4f} circles")

            if abs(current_circle - target_circle) < 0.01:
                print("Target reached")
                break

            time.sleep(1)

        self.disable_motor()

# Control logic for real-time keyboard input
def control_motor_with_keyboard():
    left_right_motor = MotorController(port='/dev/LRttyUSB', unit=0x00)
    up_down_motor = MotorController(port='/dev/updottyUSB', unit=0x01)

    try:
        left_right_motor.connect()
        up_down_motor.connect()

        current_lr_angle = 0  # Current angle for left-right motor
        current_ud_angle = 0  # Current angle for up-down motor
        step_angle = 1        # Step angle for each key press

        while True:
            if keyboard.is_pressed('j'):  # Move left
                if current_lr_angle > -90:
                    current_lr_angle -= step_angle
                    print(f"Moving left to {current_lr_angle} degrees")
                    left_right_motor.move_to_position(current_lr_angle / 360, rpm=5)

            elif keyboard.is_pressed('l'):  # Move right
                if current_lr_angle < 90:
                    current_lr_angle += step_angle
                    print(f"Moving right to {current_lr_angle} degrees")
                    left_right_motor.move_to_position(current_lr_angle / 360, rpm=5)

            elif keyboard.is_pressed('i'):  # Move up
                if current_ud_angle < 90:
                    current_ud_angle += step_angle
                    print(f"Moving up to {current_ud_angle} degrees")
                    up_down_motor.move_to_position(current_ud_angle / 360, rpm=5)

            elif keyboard.is_pressed(','):  # Move down
                if current_ud_angle > 0:
                    current_ud_angle -= step_angle
                    print(f"Moving down to {current_ud_angle} degrees")
                    up_down_motor.move_to_position(current_ud_angle / 360, rpm=5)

            elif keyboard.is_pressed('k'):  # Stop all motors
                print("Stopping all motors")
                left_right_motor.disable_motor()
                up_down_motor.disable_motor()
                break

            time.sleep(0.1)  # Short delay to prevent excessive input processing

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        left_right_motor.close()
        up_down_motor.close()

if __name__ == "__main__":
    control_motor_with_keyboard()
