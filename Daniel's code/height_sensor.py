import time
import board
import busio
import adafruit_vl53l0x

class HeightSensor:
    def __init__(self, offset_cm=9.3, default_height_m=30.0):
        """
        :param offset_cm: 传感器光心到相机光心的垂直偏移量(厘米)
        :param default_height_m: 传感器无数据/异常时返回的默认高度(米)
        """
        self.offset_m = offset_cm / 100.0
        self.default_height = default_height_m
        self.vl53 = None
        self._init_sensor()

    def _init_sensor(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
            # 提升抗干扰能力（50ms 可滤除部分环境光噪声）
            self.vl53.measurement_timing_budget = 50000  
            print("✅ VL53L0X Height Sensor Initialized")
        except Exception as e:
            print(f"⚠️ VL53L0X Init Failed: {e} | Fallback to {self.default_height}m")
            self.vl53 = None

    def get_height_m(self):
        """返回相机距离地面的实时高度（米），无上下限限制"""
        if self.vl53 is None:
            return self.default_height
        try:
            dist_mm = self.vl53.range  # 单位：毫米
            
            # 🔑 仅过滤无效值：0=未检测到/超量程，<50mm=近场盲区噪声
            # 不再限制上限，传感器能读多少就用多少
            if dist_mm and dist_mm > 50:
                height = (dist_mm / 1000.0) - self.offset_m
                return height  # ✅ 直接返回原始计算值，完全解除限制
            return self.default_height
        except Exception as e:
            print(f"⚠️ Height Read Error: {e}")
            return self.default_height