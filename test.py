import struct

def float_to_hex(f):
    # 將浮點數轉換為IEEE 754的32位格式
    packed = struct.pack('!f', f)
    # 將二進制數據轉換為16進制字符串
    hex_str = packed.hex()
    return hex_str

# 測試範例
def float_to_comp():
    set_rpm = 30
    ieee_754_rpm = float_to_hex(set_rpm)
    ieee_754_rpm_high = int('0x'+ieee_754_rpm[0:4], 16)
    ieee_754_rpm_low = int('0x'+ieee_754_rpm[4:8], 16)
    circle = 1
    ieee_circle = float_to_hex(circle)
    ieee_circle_high = int('0x'+ieee_circle[0:4], 16)
    ieee_circle_low = int('0x'+ieee_circle[4:8], 16)

a= float_to_comp.ieee_754_rpm_high
print(a)
