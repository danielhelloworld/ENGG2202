import time
import board
import busio
import adafruit_vl53l0x

def main():
    # Initialize I2C bus and sensor.
    i2c = busio.I2C(board.SCL, board.SDA)
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)

    print("VL53L0X Distance Measurement (Adafruit Library)")
    print("Press CTRL+C to exit")
    print("-" * 30)

    try:
        while True:
            # Get distance measurement
            distance = vl53.range  # in millimeters

            if distance:
                # Print distance in mm and cm
                print(f"Distance: {distance} mm | {distance/10:.1f} cm")
            else:
                print("Invalid measurement")

            # Wait between measurements (Adafruit library may need at least 20ms)
            time.sleep(0.02)  # 20ms delay

    except KeyboardInterrupt:
        print("\nMeasurement stopped by user")

    # The Adafruit library doesn't require explicit stop_ranging, but you can deinitialize the I2C if needed.
    # Note: The Adafruit library doesn't have a stop_ranging method.

if __name__ == "__main__":
    main()