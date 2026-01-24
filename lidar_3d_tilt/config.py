# config.py
ARDUINO_PORT = "/dev/ttyACM1"
ARDUINO_BAUD = 115200

LIDAR_LAUNCH = ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"]

MIN_RANGE = 0.10
MAX_RANGE = 12.0

ANGLE_RESOLUTION_DEG = 0.5
PUBLISH_RATE_HZ = 5
MAX_BUFFER_POINTS = 120_000

# Servo sweep detection
SWEEP_RESET_THRESHOLD = -14.5  # degrees

# Lever-arm compensation (MEASURE THESE)
LIDAR_OFFSET_X = 0.03   # meters
LIDAR_OFFSET_Z = 0.05   # meters
