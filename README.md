# Three‑Stage Stair‑Climbing Robot – F Module (General Support Layer)
## Module Overview
The F Module serves as the **general support layer** for the entire three‑stage stair‑climbing robot project. It provides cross‑module shared data types, global configuration, utility functions, and logging functionality. This layer offers unified infrastructure for all upper‑level modules (sensors, actuators, state machines, etc.), ensuring code portability and maintainability.

---
## File Structure
include/
├── config.h    # Configuration for global hardware pins, sensor thresholds, motion parameters, etc.
├── types.h     # Definitions of common data types, enumerations, and structs
├── utils.h     # Declarations of utility functions (clamping, filtering, timeout detection)
└── logger.h    # Logging interface declarations

src/
├── utils.cpp   # Implementation of utility functions
└── logger.cpp  # Logging implementation (with timestamp and level output)

---
## File Descriptions
### `config.h`
Stores all hardware‑related constant configurations, including:
- I2C bus, PCA9685 address, PWM frequency
- Pin mapping for motor drivers, servos, and linear actuators
- Threshold and timeout parameters for sensors (ultrasonic, downward‑looking, IMU)
- Geometric dimensions, movement speed, safety limits, etc.

**Example of important constants**:
```cpp
namespace RobotConfig {    
namespace GPIO {        
constexpr int FRONT_L_IN1 = 17;   // Front-left motor direction pin    
}    
namespace Motion {        
constexpr float APPROACH_SPEED = 0.35f;    
}    
namespace Safety {        
constexpr float MAX_SAFE_PITCH_DEG = 25.0f;    
}
}
```

### `types.h`
Defines common data types and enumerations used across the project:
- MotionState: robot state machine states (idle, approaching step, front‑stage climbing, etc.)
- FaultCode: fault code enumeration
- Structs: PoseData (posture), DistanceReading (distance), DownwardReading (downward‑looking), AxisState (linear axis state), etc.
- Callback function types: PoseCallback, DistanceCallback, etc.

### `utils.h` / `utils.cpp`
Provides general utility functions:
- `clamp()`: value clamping
- `lowPassFilter()`: first‑order low‑pass filtering
- `hasTimedOut()` / `isFresh()`: data freshness judgment

### `logger.h` / `logger.cpp`
A simple logging system supporting four levels:
- `Logger::debug()`: debug information
- `Logger::info()`: general information
- `Logger::warn()`: warning
- `Logger::error()`: error

Log output format: `[HH:MM:SS] [LEVEL] message`

## Usage
1. Include header files
Include the corresponding headers in source files that require configuration, types, or utilities:
```cpp
#include "config.h"
#include "types.h"
#include "utils.h"
#include "logger.h"
```

2. Use logging
```cpp
Logger::info("Robot initializing...");
if (error_occurred) {    
Logger::error("Failed to start IMU sensor");
}
```

3. Use utility functions
