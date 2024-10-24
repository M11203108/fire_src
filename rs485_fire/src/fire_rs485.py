from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import struct
import time

def float_to_registers(f):
    # 將浮點數轉換為IEEE 754的16位寄存器高位和低位
    packed = struct.pack('!f', f)
    return struct.unpack('!HH', packed)

def registers_to_float(high, low):
    # 將兩個16位寄存器轉換回浮點數
    packed = struct.pack('!HH', high, low)
    return struct.unpack('!f', packed)[0]


client = ModbusClient(
    method='rtu',
    port='/dev/ttyUSB7',
    baudrate=9600,
    parity='N',
    stopbits=2,
    bytesize=8,
    timeout=3
)

circle = 0.10  # 目標轉動圈數
set_rpm = 20  # 設定的轉速
direction = 0x0000  # 轉向

# 轉換浮點數到高低位寄存器
rpm_high, rpm_low = float_to_registers(set_rpm)
circle_high, circle_low = float_to_registers(circle)

UNIT = 0x00

if client.connect():
    print("Connected")
    
    # 設置使能
    enable_result = client.write_register(0x004F, 0x0001, unit=UNIT)
    if not enable_result.isError():
        print("Enable success")
    
    # 設置轉速
    client.write_register(0x0053, rpm_high, unit=UNIT)
    client.write_register(0x0054, rpm_low, unit=UNIT)
    print(f"Set {set_rpm} rpm")

    # 設置轉向
    client.write_register(0x0050, direction, unit=UNIT)
    print("Set direction")

    # 設置圈數
    client.write_register(0x0055, circle_high, unit=UNIT)
    client.write_register(0x0056, circle_low, unit=UNIT)
    print(f"Set circle {circle}")

    # 開始運動
    # time.sleep(5)

    # current_circle = 0.0
    # while current_circle < circle:
    #     # 讀取當前位置
    #     position_high = client.read_holding_registers(0x0007, 1, unit=UNIT)
    #     position_low = client.read_holding_registers(0x0007, 1, unit=UNIT)
        
    #     # 將寄存器轉換回浮點數表示的圈數
    #     current_circle = registers_to_float(position_high.registers[0], position_low.registers[0])
    #     print(f"Current position: {current_circle} circles")
        
    #     # 檢查是否達到目標圈數
    #     if current_circle >= circle:
    #         print("Target reached, stopping motor")
    #         break
        
    #     time.sleep(1)  # 每秒檢查一次當前位置

    # 停止馬達
    # client.write_register(0x004F, 0x0000, unit=UNIT)
    # print("Motor stopped")

    client.close()
    print("Close connection")
else:
    print("Cannot connect")
    exit()
