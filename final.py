import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import serial
import pynmea2
import pyrebase

# Firebase configuration
config = {
}

# Initialize Firebase
firebase = pyrebase.initialize_app(config)
db = firebase.database()

# Create the I2C bus using the available I2C port (1, 3, 2)
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)
# Function to convert sensor resistance to gas concentration (linear interpolation)
def convert_to_gas_concentration(sensor_value, sensitivity_curve):
    # Assuming sensitivity_curve is a list of tuples (sensor_resistance, gas_concentration)
    # and the values are sorted in ascending order of sensor resistance

    # Find the two points around the current sensor value
    for i in range(len(sensitivity_curve) - 1):
        if sensitivity_curve[i][0] <= sensor_value <= sensitivity_curve[i + 1][0]:
            x0, y0 = sensitivity_curve[i]
            x1, y1 = sensitivity_curve[i + 1]

            # Linear interpolation formula
            gas_concentration = y0 + (sensor_value - x0) * (y1 - y0) / (x1 - x0)

            return gas_concentration

    # If the sensor value is outside the range of the sensitivity curve, return None
    return None



# Sensitivity characteristics for Smoke, LPG, Methane, and Hydrogen (replace with actual values)
smoke_curve = [
    (200, 17.1), (500, 20.9), (1000, 17.2), (1500, 11.9), (2000, 16.3),
    (2500, 15.4), (3000, 12.2), (3500, 14.6), (4000, 14.2), (4500, 19.0),
    (5000, 16.7), (5500, 12.8), (6000, 15.2), (6500, 14.5), (7000, 13.8),
    (7500, 12.1), (8000, 11.4), (8500, 10.7), (9000, 10.0), (9500, 9.3),
    (10000, 8.6), (10500, 7.9), (11000, 7.2), (11500, 6.5), (12000, 5.8),
    (14000, 1.5), (16000, 1.2), (18000, 1.0), (20000, 0.9),
    (25000, 0.8), (30000, 0.7), (35000, 0.6), (40000, 0.5), (45000, 0.4),
    (50000, 0.3)
    # Add more points as needed
]

lpg_curve = [
    (200, 3.2), (500, 2.7), (1000, 4.2), (1500, 3.5), (2000, 3.5),
    (2500, 4.0), (3000, 2.7), (3500, 3.0), (4000, 4.0), (4500, 2.8),
    (5000, 4.3), (5500, 4.1), (6000, 3.7), (6500, 3.5), (7000, 3.3),
    (7500, 3.1), (8000, 2.9), (8500, 2.7), (9000, 2.5), (9500, 2.3),
    (10000, 2.1), (10500, 1.9), (11000, 1.7), (11500, 1.5), (12000, 1.3),    (14000, 16.8), (16000, 16.3), (18000, 15.8), (20000, 15.3),
    (25000, 14.8), (30000, 14.3), (35000, 13.8), (40000, 13.3), (45000, 12.8),
    (50000, 12.3)
    # Add more points as needed
]

methane_curve = [
    (200, 7.4), (500, 7.8), (1000, 9.2), (1500, 15.0), (2000, 8.7),
    (2500, 8.2), (3000, 8.6), (3500, 9.2), (4000, 11.1), (4500, 6.3),
    (5000, 6.9), (5500, 8.2), (6000, 7.4), (6500, 6.7), (7000, 5.9),
    (7500, 5.1), (8000, 4.3), (8500, 3.5), (9000, 2.7), (9500, 1.9),
    (10000, 1.1), (10500, 0.3), (11000, 0.1), (11500, 0.0), (12000, 0.0),    (14000, 23.5), (16000, 23.0), (18000, 22.5), (20000, 22.0),
    (25000, 21.5), (30000, 21.0), (35000, 20.5), (40000, 20.0), (45000, 19.5),
    (50000, 19.0)
    # Add more points as needed
]

hydrogen_curve = [
    (200, 9.4), (500, 8.0), (1000, 7.1), (1500, 6.5), (2000, 7.1),
    (2500, 7.8), (3000, 6.7), (3500, 8.5), (4000, 7.7), (4500, 8.7),
    (5000, 7.7), (5500, 10.6), (6000, 7.8), (6500, 7.1), (7000, 6.4),
    (7500, 5.7), (8000, 5.0), (8500, 4.3), (9000, 3.6), (9500, 2.9),
    (10000, 2.2), (10500, 1.5), (11000, 0.8), (11500, 0.1), (12000, 0.0),    (14000, 24.4), (16000, 23.9), (18000, 23.4), (20000, 22.9),
    (25000, 22.4), (30000, 21.9), (35000, 21.4), (40000, 20.9), (45000, 20.4),
    (50000, 19.9)
    # Add more points as needed
]
# Add more data points to each curve

def read_gps_data(serial_port):
    try:
        while True:
            line = serial_port.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                print("Latitude:", msg.latitude)
                print("Longitude:", msg.longitude)
                return msg.latitude, msg.longitude
    except KeyboardInterrupt:
        pass
    finally:
        serial_port.close()

try:
    while True:
        sensor_value = chan.value
        smoke_concentration = convert_to_gas_concentration(sensor_value, smoke_curve)
        lpg_concentration = convert_to_gas_concentration(sensor_value, lpg_curve)
        methane_concentration = convert_to_gas_concentration(sensor_value, methane_curve)
        hydrogen_concentration = convert_to_gas_concentration(sensor_value, hydrogen_curve)

        latitude, longitude = read_gps_data(serial.Serial('/dev/serial0', baudrate=9600, timeout=1))

        # Print and send data to Firebase
        print(f'Sensor Value: {sensor_value}')
        print(f'Smoke: {smoke_concentration:.1f}, LPG: {lpg_concentration:.1f}, Methane: {methane_concentration:.1f}, Hydrogen: {hydrogen_concentration:.1f}')
        print(f'Latitude: {latitude}, Longitude: {longitude}')

        # Send data to Firebase
        data = {
            "sensor_value": sensor_value,
            "smoke_concentration": smoke_concentration,
            "lpg_concentration": lpg_concentration,
            "methane_concentration": methane_concentration,
            "hydrogen_concentration": hydrogen_concentration,
            "latitude": latitude,
            "longitude": longitude,
            "timestamp": int(time.time())
        }

        db.child("sensor_data").push(data)

        time.sleep(2)  # Adjust the delay based on your requirements

except KeyboardInterrupt:
    pass
