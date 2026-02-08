# config.py
IMU_PORT = "/dev/ttyACM0"
SERVO_PORT = "/dev/ttyUSB1"
BAUD = 115200

# Sweep limits
TILT_MIN = 70.0
TILT_MAX = 110.0
TILT_CENTER = 90.0

# Resolution and Memory
ANGLE_RESOLUTION_DEG = .5
PUBLISH_RATE_HZ = 10
MAX_BUFFER_POINTS = 200_000

# Recording Settings
SWEEPS_TO_RECORD = 4  # How many full up/down cycles to record
OUTPUT_FILENAME = "lidar_sweep_data.pkl"

# Physical Offsets
LIDAR_OFFSET_X = 0.01 
LIDAR_OFFSET_Z = 0.03

LIDAR_LAUNCH = ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"]