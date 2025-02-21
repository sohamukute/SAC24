
# **SAC 24**  
**ESP32 Line-Following Robot | PID-Enhanced LSRB Algorithm**  

This repository implements a high-performance line-following system for the **ESP32 microcontroller**, combining the classic LSRB (Left-Straight-Right-Back) logic with PID control and Finite State Machine (FSM) architecture. Designed for precision and adaptability, the codebase emphasizes real-time sensor fusion, motor dynamics tuning, and intersection handling for complex tracks.  

### **Key Features**  
- **Hybrid LSRB-PID Control**:  
  - `calculate_pid()`: Dynamically adjusts motor speeds using error-weighted PID terms to stabilize sharp turns.  
  - `follow_line()`: Implements sensor-thresholded LSRB logic with fallback to PID correction.  
- **Intersection Detection**:  
  - `check_intersection()`: Identifies cross/junction nodes via multi-sensor triggers for path decision-making.  
- **Hardware Abstraction**:  
  - Modular GPIO/PWM configuration (`gpio_set_direction`, `ledcSetup`) for IR sensors and motor drivers.  
  - Calibration routines (`calibrate_sensors()`) for ambient light adaptation.  
- **ESP32-Specific Optimization**:  
  - Leverages FreeRTOS tasks for parallel sensor polling and motor control.  
  - Wi-Fi-enabled telemetry (optional) for real-time parameter tuning.  

### **Theory & Architecture**  
- **State-Driven Navigation**:  
  - FSM with states (`LINE_FOLLOW`, `INTERSECTION`, `UTURN`) to manage track transitions.  
- **Sensor Fusion**:  
  - 5x IR sensor array mapped to ESP32 ADC channels for sub-millisecond line detection.  
- **Motor Dynamics**:  
  - PWM-driven speed asymmetry (`set_motor_speeds()`) for smooth turns.  
