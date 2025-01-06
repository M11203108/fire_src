from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time
import struct

# 將浮點數轉換為16進位表示的字串
def float_to_hex(f):
    packed = struct.pack('!f', f)
    return packed.hex()

# 讀取編碼器的原始值（未轉換為圈數）
def read_encoder_raw(client, unit):
    position_high = client.read_holding_registers(0x0007, 1, unit=unit).registers[0]
    position_low = client.read_holding_registers(0x0008, 1, unit=unit).registers[0]
    encoder_value = (position_high << 16) | position_low
    if encoder_value & 0x80000000:  # 處理負值
        encoder_value = -(0xFFFFFFFF - encoder_value + 1)
        # 新增 print 語句以顯示讀取的編碼器值
        print(f"[DEBUG] Encoder raw value: {encoder_value} (High: {position_high}, Low: {position_low})")
    return encoder_value

# 控制馬達移動至目標圈數
def move_motor(client, unit, direction, target_circle, set_rpm):
    ieee_circle = float_to_hex(target_circle)
    ieee_circle_high = int('0x' + ieee_circle[0:4], 16)
    ieee_circle_low = int('0x' + ieee_circle[4:8], 16)

    ieee_754_rpm = float_to_hex(set_rpm)
    ieee_754_rpm_high = int('0x' + ieee_754_rpm[0:4], 16)
    ieee_754_rpm_low = int('0x' + ieee_754_rpm[4:8], 16)

    client.write_register(0x004F, 0x0001, unit=unit)  # 啟動馬達
    client.write_register(0x0053, ieee_754_rpm_high, unit=unit)  # 設定轉速高位
    client.write_register(0x0054, ieee_754_rpm_low, unit=unit)  # 設定轉速低位
    client.write_register(0x0050, direction, unit=unit)  # 設置方向
    client.write_register(0x0055, ieee_circle_high, unit=unit)  # 設置目標圈數高位
    client.write_register(0x0056, ieee_circle_low, unit=unit)  # 設置目標圈數低位
    # print(f"Motor moving: direction={direction}, target_circle={target_circle:.4f}, set_rpm={set_rpm}")
    print(f"[DEBUG] Motor command sent -> Direction: {direction}, Target circle: {target_circle:.4f}, RPM: {set_rpm}")

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(10, 50)):  # 最低 RPM 設為 10
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        # self.integral += error
        self.integral = max(min(self.integral, 100), -100)  # 限制積分項累積範圍

        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 限制輸出 RPM
        min_output, max_output = self.output_limits
        output = max(min(output, max_output), min_output)

        self.previous_error = error
        print(f"[DEBUG] PID Compute -> Error: {error}, Integral: {self.integral}, Derivative: {derivative}, Output: {output}")
        
        return output

# # 根據角度差進行兩段式減速
# def calculate_rpm(angle_diff, max_rpm):
#     abs_angle_diff = abs(angle_diff)
    
#     if abs_angle_diff > 60.0:  # 大於 60 度，速度區間 80 ~ 50
#         rpm = 50 + (30 * (abs_angle_diff - 60.0) / 120.0)  # 線性從 50 增加到 80
#     else:  # 小於等於 60 度，速度區間 50 ~ 25
#         rpm = 25 + (25 * abs_angle_diff / 60.0)  # 線性從 25 增加到 50

#     return min(rpm, max_rpm)  # 確保 RPM 不會超過 max_rpm


# 根據編碼器值計算目標圈數並移動到指定位置
def move_to_target_position(client, unit, current_encoder, target_encoder, max_rpm, resolution):
    # print(f"Moving to target position: {target_encoder} from {current_encoder}")

    # 計算初始的角度差
    angle_diff = (target_encoder - current_encoder) * 360.0 / resolution

    # 根據角度差動態調整最小 RPM
    min_rpm = max(10, int(abs(angle_diff) * 0.5))  # 確保 RPM 不低於 10
    pid = PIDController(kp=1.5, ki=0.05, kd=0.01, output_limits=(min_rpm, max_rpm))

    while True:
        current_encoder = read_encoder_raw(client, unit)  # 讀取當前編碼器值
        angle_diff = (target_encoder - current_encoder) * 360.0 / resolution  # 更新角度差

        # 檢查是否到達目標位置
        if abs(angle_diff) < 0.5:  # 誤差範圍內
            client.write_register(0x004F, 0x0000, unit=unit)  # 停止馬達
            print(f"Reached target position: {current_encoder}")
            break

        # 計算方向與 RPM
        direction = 0x0000 if angle_diff > 0 else 0x0001
        set_rpm = pid.compute(setpoint=0, measurement=angle_diff)  # 使用 PID 計算 RPM
        target_circle = abs(angle_diff / 360.0)

        # 發送移動指令
        move_motor(client, unit, direction, target_circle, set_rpm)

        # print(f"Current angle diff: {angle_diff:.2f} degrees, Set RPM: {set_rpm:.2f}")
        time.sleep(0.1)

        
# 執行歸零校準
def zero_calibration(client, unit, resolution):
    print("Starting zero calibration...")

    # 左極限
    print("Moving to left limit...")
    left_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0000, 1.0, 50)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 10:
            left_limit = current_encoder
            break

    print(f"Left limit: {left_limit}")

    # 右極限
    print("Moving to right limit...")
    right_limit = None
    stable_count = 0
    previous_encoder = None
    while True:
        move_motor(client, unit, 0x0001, 1.0, 50)  # 嘗試大範圍移動
        current_encoder = read_encoder_raw(client, unit)
        if previous_encoder == current_encoder:
            stable_count += 1
        else:
            stable_count = 0

        previous_encoder = current_encoder

        if stable_count >= 10:
            right_limit = current_encoder
            break

    print(f"Right limit: {right_limit}")

    # 設定零點
    zero_encoder = (left_limit + right_limit) // 2
    print(f"Zero position set to: {zero_encoder}")

    print(f"Left limit: {left_limit}, Left limit angle: {(left_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")
    print(f"Right limit: {right_limit}, Right limit angle: {(right_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")

    # 移動到零點
    move_to_target_position(client, unit, read_encoder_raw(client, unit), zero_encoder, 50, resolution)
    return zero_encoder, left_limit, right_limit

# 主程式入口
def main():
    client = ModbusClient(
        method='rtu',
        port='/dev/LRttyUSB',
        baudrate=9600,
        parity='N',
        stopbits=2,
        bytesize=8,
        timeout=3
    )

    UNIT = 0x00
    resolution = 3600 * 4  # 編碼器分辨率
    max_rpm = 50  # 最大轉速

    if not client.connect():
        print("Cannot connect to Modbus device.")
        return

    print("Connected")

    # 執行歸零校準
    zero_encoder, left_limit, right_limit = zero_calibration(client, UNIT, resolution)
    print(f"Calibration complete. Zero encoder: {zero_encoder}, Left limit: {left_limit}, Right limit: {right_limit}")
    print(f"Left limit angle: {(left_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")
    print(f"Right limit angle: {(right_limit - zero_encoder) * 360.0 / resolution:.2f} degrees")

    # 進入角度控制模式
    while True:
        user_input = input("Enter target angle (e.g., 20 or -15), or 'q' to quit: ").strip()
        if user_input.lower() == 'q':
            print("Exiting angle control mode.")
            break

        try:
            target_angle = float(user_input)
            target_encoder = zero_encoder + int(target_angle * resolution / 360.0)
            move_to_target_position(client, UNIT, read_encoder_raw(client, UNIT), target_encoder, max_rpm, resolution)

        except ValueError:
            print("Invalid input. Please enter a valid angle or 'q' to quit.")

    client.close()
    print("Connection closed")

if __name__ == "__main__":
    main()
