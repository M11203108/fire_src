# from motor_control.motor1 import MotorController

# def main():
#     motor1 = MotorController(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
#     if not motor1.connect():
#         print("Cannot connect to Motor 1.")
#         return
#     print("Motor 1 connected.")

#     try:
#         print("Starting zero calibration...")
#         zero_encoder, left_limit, right_limit = motor1.zero_calibration()
#         print(f"Motor 1 calibration result: {zero_encoder}, {left_limit}, {right_limit}")
#     except Exception as e:
#         print(f"[ERROR] Motor 1 calibration failed: {e}")
#     finally:
#         motor1.close()
# import threading

# lock = threading.Lock()

# def calibrate_motor(motor, result_dict, name):
#     with lock:  # 確保資源鎖定
#         try:
#             print(f"Starting calibration for {name}...")
#             zero_encoder, left_limit, right_limit = motor.zero_calibration()
#             result_dict.update({
#                 "zero_encoder": zero_encoder,
#                 "left_limit": left_limit,
#                 "right_limit": right_limit
#             })
#             print(f"{name} calibration complete: {result_dict}")
#         except Exception as e:
#             print(f"[ERROR] {name} calibration failed: {e}")

# if __name__ == "__main__":
#     main()
from motor_control.motor1 import MotorControllerupdo
from motor_control.motor2 import MotorControllerLR
import threading

lock = threading.Lock()

def calibrate_motor(motor, result_dict, name):
    try:
        print(f"Starting calibration for {name}...")
        zero_encoder, left_limit, right_limit = motor.zero_calibration()
        result_dict.update({
            "zero_encoder": zero_encoder,
            "left_limit": left_limit,
            "right_limit": right_limit
        })
        print(f"{name} calibration complete: {result_dict}")
    except Exception as e:
        print(f"[ERROR] {name} calibration failed: {e}")

def main():
    # 初始化兩個控制器
    motor1 = MotorControllerupdo(port='/dev/updottyUSB', baudrate=9600, unit=0x00)
    motor2 = MotorControllerLR(port='/dev/LRttyUSB', baudrate=9600, unit=0x01)

    # 連接控制器
    if not motor1.connect():
        print("Cannot connect to Motor 1.")
        return
    print("Motor 1 connected.")

    if not motor2.connect():
        print("Cannot connect to Motor 2.")
        return
    print("Motor 2 connected.")

    # 校準結果存儲字典
    motor1_result = {}
    motor2_result = {}

    # 啟動多執行緒校準
    thread1 = threading.Thread(target=calibrate_motor, args=(motor1, motor1_result, "Motor1"))
    thread2 = threading.Thread(target=calibrate_motor, args=(motor2, motor2_result, "Motor2"))

    thread1.start()
    print('test')
    thread2.start()

    thread1.join()
    thread2.join()

    # 打印校準結果
    print(f"Motor 1 calibration result: {motor1_result}")
    print(f"Motor 2 calibration result: {motor2_result}")

    # 關閉連接
    motor1.close()
    motor2.close()

if __name__ == "__main__":
    main()
