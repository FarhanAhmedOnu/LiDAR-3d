Here is a short documentation on how to interpret and use the recorded `.pkl` data file.

### **LiDAR Data File Documentation (`.pkl`)**

**Format:** Python Pickle (Binary)
**Content:** A list of dictionaries, where each dictionary represents one distinct LiDAR packet (scan) and the robot's state at that exact moment.

---

### **1. How to Load the Data**

You do not need ROS 2 running to open this file. You only need standard Python.

```python
import pickle

FILENAME = "lidar_sweep_data.pkl"

with open(FILENAME, "rb") as f:
    data = pickle.load(f)

print(f"Loaded {len(data)} frames.")

```

### **2. Data Structure**

The variable `data` is a **Python List**.
Each item in the list is a **Dictionary** with the following keys:

| Key | Type | Description |
| --- | --- | --- |
| `'timestamp'` | `float` | Time in seconds since recording started. |
| `'tilt'` | `float` | The servo/IMU tilt angle in **radians** (approx 1.22 to 1.92 rad). |
| `'scan'` | `LaserScan` | The raw ROS 2 `LaserScan` object containing range data. |

### **3. accessing the LaserScan Object**

The `'scan'` key contains the raw sensor message. You access its fields using dot notation:

* **`scan.ranges`**: A list of float distances (in meters).
* **`scan.intensities`**: (Optional) A list of signal strengths.
* **`scan.angle_min`**: Start angle of the scan (radians).
* **`scan.angle_increment`**: Step size between measurements (radians).

### **4. Example: processing Loop**

Use this template to iterate through the data and apply your custom algorithms.

```python
import math
import numpy as np

# Loop through every recorded frame
for i, frame in enumerate(data):
    
    # 1. Get Inputs
    tilt_angle = frame['tilt']
    scan_msg = frame['scan']
    
    # 2. Convert Ranges to Angles
    # Create an array of angles matching the ranges
    num_points = len(scan_msg.ranges)
    azimuths = np.linspace(scan_msg.angle_min, 
                           scan_msg.angle_max, 
                           num_points)
                           
    # 3. Access Specific Points
    for r, azimuth in zip(scan_msg.ranges, azimuths):
        if r < 0.1 or r > 12.0: 
            continue # Skip invalid points
            
        # YOUR MATH HERE: Convert Polar (r, azimuth, tilt) -> Cartesian (x,y,z)
        # x = ...
        # y = ...
        # z = ...

```

### **5. Dependencies**

To load the file, your environment usually needs the class definition of the objects inside. Since we stored ROS messages, you generally need the ROS libraries installed, **OR** you need a dummy class if opening it on a non-ROS machine.

* **On Robot/ROS Machine:** Works natively.
* **On Non-ROS Laptop:** You may need to install the message types:
```bash
pip install sensor-msgs-py

```


*(If strictly standard pickle fails, ensure `sensor_msgs.msg.LaserScan` is importable).*