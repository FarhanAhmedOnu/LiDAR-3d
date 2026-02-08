# 3D LiDAR Scanning System with Programmable Tilt Mechanism

## System Overview

This system creates a **3D scanning solution** by combining a 2D LiDAR (RPLIDAR C1) with a tilt mechanism. The servo rotates the LiDAR vertically while it scans horizontally, transforming 2D distance measurements into 3D point clouds.

## System Architecture

```mermaid
graph TD
    A[LiDAR Hardware] -->|2D Scans| B[ROS2 /scan topic]
    C[Servo Mechanism] -->|Tilt Angle| D[ServoController]
    B --> E[PointBuilder]
    D -->|current_tilt_rad| E
    E -->|3D Points| F[CloudPublisher]
    F -->|PointCloud2| G[RVIZ/ROS Network]
    
    H[Main Node] -->|Orchestrates| B
    H -->|Controls| D
    H -->|Manages| F
```

## Core Mathematical Transformation

### 2D to 3D Coordinate Conversion

The system performs a **Y-axis rotation transformation** on each 2D LiDAR point:

**Given:**
- 2D LiDAR range: `r` (meters)
- Horizontal angle: `θ` (radians)
- Tilt angle: `α` (radians, from servo)
- LiDAR offset from servo axis: `(offset_x, offset_z)`

**Transformation:**
```
1. Convert to LiDAR frame (2D):
   xl = r × cos(θ)
   yl = r × sin(θ)
   zl = 0

2. Apply tilt rotation + offset compensation:
   x = xl × cos(α) + offset_x × cos(α)
   y = yl
   z = -xl × sin(α) + offset_z
```

**Visual Representation:**
```
Before Tilt:      After Tilt:
   ↗ z              ↗ z (tilted)
   │                │   /
   │                │  /
   └───→ x          └───→ x
                    │
                    ↓ y (unchanged)
```

## Component Workflow

```mermaid
sequenceDiagram
    participant Main Node
    participant ServoController
    participant LiDAR
    participant PointBuilder
    participant CloudPublisher

    Main Node->>LiDAR: Start Process
    Main Node->>ServoController: Initialize Serial
    ServoController->>Arduino: Send ANGLE commands
    ServoController->>PointBuilder: Provide tilt_rad
    
    loop Every Scan (~5Hz)
        LiDAR->>PointBuilder: 2D Ranges + Angles
        PointBuilder->>PointBuilder: Transform to 3D
        PointBuilder->>CloudPublisher: Accumulated Points
    end
    
    loop Every 200ms (5Hz)
        CloudPublisher->>ROS: Publish PointCloud2
    end
```

## Key Design Features

1. **Continuous Sweep Pattern**: Servo oscillates between -15° to +15° at 0.5° resolution
2. **Angle Binning**: Points grouped by tilt angle for memory efficiency
3. **Lever-Arm Compensation**: Accounts for physical offset between servo axis and LiDAR
4. **Thread-Safe Design**: Separate threads for servo control and ROS2 operations

## Data Flow Visualization

```mermaid
graph LR
    A[360° 2D Scan] --> B{Point Filter}
    B -->|Valid Range| C[2D→3D Transform]
    B -->|Invalid| D[Discard]
    C --> E[Angle Bin Storage]
    F[Servo Angle] --> C
    E --> G[5Hz Publisher]
    G --> H[PointCloud2 Message]
```

## Performance Characteristics

- **3D Points/Second**: ~108,000 (360 points × 60 tilt positions × 5Hz)
- **Servo Update Rate**: 33.3Hz (30ms intervals)
- **Angle Resolution**: 0.5° vertical, ~1° horizontal (LiDAR native)
- **Memory Usage**: Fixed buffer of 120,000 points (~1.1 seconds of data)

## Error Handling Strategy

The system implements **graceful degradation**:
1. **Serial failures**: Servo centers itself and closes port
2. **ROS2 node failures**: Clean shutdown with proper cleanup
3. **LiDAR process failures**: SIGINT → SIGTERM → SIGKILL escalation
4. **Thread safety**: Mutex locks prevent race conditions during shutdown

This design provides a **robust, production-ready** 3D scanning solution that transforms affordable 2D LiDAR hardware into a 3D scanning system through precise mechanical control and mathematical transformations.