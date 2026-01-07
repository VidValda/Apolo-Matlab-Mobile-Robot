# Apolo-Matlab-Mobile-Robot

A comprehensive mobile robot navigation system using MATLAB and the Apolo simulator. This project implements an Extended Kalman Filter (EKF) for localization, A\* path planning, PID control, and reactive obstacle avoidance for a differential drive robot.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Project Structure](#project-structure)
4. [Apolo Initialization](#apolo-initialization)
5. [Calibration](#calibration)
6. [Main Code Execution](#main-code-execution)
7. [Extended Kalman Filter (EKF)](#extended-kalman-filter-ekf)
8. [Configuration](#configuration)
9. [Usage Examples](#usage-examples)

---

## Prerequisites

- **MATLAB** (64-bit version)
- **Apolo Simulator** (64-bit, compiled for Windows 10)
- **MATLAB Toolboxes**: Image Processing Toolbox (for A\* path planning)

---

## Installation

### 1. Install Apolo

1. Install Apolo (64-bit version compatible with Windows 10 and 64-bit MATLAB)
   - Repository: https://github.com/mhernando/Apolo
2. The installation creates the following structure:
   ```
   Apolo/
   ├── data/          # XML environment files
   ├── Matlab/        # Apolo MATLAB functions
   └── doc/           # Documentation
   ```

### 2. Configure MATLAB Path

Add the Apolo MATLAB directory to your MATLAB path:

```matlab
addpath('C:\Path\To\Apolo\Matlab');
```

Alternatively, you can add it permanently:

- Go to `Home` → `Set Path` → `Add Folder`
- Select the `Apolo/Matlab` directory
- Click `Save`

### 3. Clone/Download This Project

```bash
git clone https://github.com/VidValda/Apolo-Matlab-Mobile-Robot.git
cd Apolo-Matlab-Mobile-Robot
```

---

## Project Structure

```
Apolo-Matlab-Mobile-Robot/
├── main.m                    # Main execution script
├── config.m                  # Configuration parameters
├── modules/
│   ├── controllers/
│   │   ├── ctrl_pid.m       # PID controller
│   │   └── ctrl_reactive.m  # Reactive obstacle avoidance
│   ├── estimators/
│   │   └── ekf.m            # Extended Kalman Filter
│   ├── planners/
│   │   └── a_star.m         # A* path planning algorithm
│   ├── robots/
│   │   └── differential_robot.m  # Robot motion interface
│   └── sensors/
│       ├── beacons_sensor.m # Beacon/landmark sensor
│       └── odometry.m       # Odometry processing
├── utils/
│   ├── mission_planner.m    # Multi-goal mission planning
│   ├── init_visualizer.m    # Visualization initialization
│   ├── update_visualizer.m  # Visualization updates
│   ├── wrapToPi.m           # Angle wrapping utility
│   └── generate_map.m       # Map generation utilities
├── tests/
│   ├── calibration/
│   │   └── calibration.m    # Sensor calibration script
│   └── conection/
│       ├── test_connection.m    # Connection test GUI
│       └── read_sensors.m       # Sensor reading test
├── models/
│   ├── Factory_Map_2026_G1.xml      # Factory environment
│   ├── Factory_Map_two_cone.xml     # Alternative environment
│   └── *.mat                        # Precomputed maps
└── README.md                 # This file
```

---

## Apolo Initialization

### 1. Launch Apolo

1. Open Apolo from the Windows Start Menu
2. Load an environment XML file:
   - Go to `File` → `Loas World XML`
   - Navigate to `Apolo/data/` or `models/` directory
   - Select an XML file (e.g., `Factory_Map_2026_G1.xml`)

### 2. Environment XML Structure

The XML file defines:

- **WorldInfo**: World parameters and sensor variance settings
- **Pioneer3ATSim**: The differential drive robot (named "Marvin" by default)
- **LMS100Sim**: SICK LMS100 laser range finder
- **LandMark**: Beacon positions in the world

Key XML elements:

```xml
<WorldInfo>
    <RegisterNumber number="12345678"/>  <!-- Student ID affects sensor variance -->
</WorldInfo>

<Pioneer3ATSim name="Marvin">
    <!-- Robot physical properties -->
</Pioneer3ATSim>

<LMS100Sim name="LMS100" linkTo="$Marvin$">
    <!-- Laser sensor configuration -->
</LMS100Sim>

<LandMark mark_id="1">
    <position>{6.00, 7.00, 0}</position>
    <!-- Beacon position in world coordinates -->
</LandMark>
```

### 3. MATLAB Connection Test

Before running the main code, verify the connection:

```matlab
cd tests/conection
test_connection
```

This opens a GUI to test:

- Robot movement commands
- Odometry readings
- Laser scan data
- Beacon/landmark detection

---

## Calibration

The calibration process estimates the noise covariance matrices **Q** (process noise) and **R** (measurement noise) for the EKF.

### Running Calibration

```matlab
cd tests/calibration
calibration
```

### Calibration Process

1. **Robot Movement Sequence**: The robot executes predefined movements:

   - Forward motion at various speeds
   - Rotations (left/right)
   - Combined motions

2. **Data Collection**:

   - **Odometry Errors**: Compares commanded velocities (v, ω) with measured odometry
   - **Sensor Errors**: Compares true beacon ranges/bearings with sensor readings

3. **Variance Calculation**:

   ```matlab
   var_v = var(errors_v);        % Linear velocity variance
   var_w = var(errors_w);        % Angular velocity variance
   var_range = var(errors_range);    % Range measurement variance
   var_bearing = var(errors_bearing); % Bearing measurement variance
   ```

4. **Covariance Matrices**:

   ```matlab
   Q = diag([var_v, var_w]) * tuning_factor;  % Process noise
   R = diag([var_range, var_bearing]);        % Measurement noise
   ```

5. **Results**: Saved to `stats_error_motores_odometria.mat`

### Calibration Parameters

- **correction_factor**: Bias correction for range measurements (default: 0.02)
- **sensor_y_offset**: Offset of sensor from robot center (default: 0.1 m)
- **tuning_Q**: Scaling factor for process noise (default: 1.0)

### Output

- `calibration_results.png`: Visualization of error distributions
- `stats_error_motores_odometria.mat`: Calibrated Q and R matrices

---

## Main Code Execution

### Quick Start

```matlab
main
```

### Execution Flow

1. **Initialization** (`config.m`):

   - Load map and environment
   - Configure robot, planner, controller, estimator, and sensors
   - Set goal poses

2. **Mission Planning**:

   - Generate path using A\* algorithm for each goal
   - Concatenate paths with hold times between goals

3. **Main Loop** (for each time step):

   ```
   a) Read laser data (for obstacle avoidance)
   b) Read beacon measurements (for EKF)
   c) Compute nominal control (PID)
   d) Apply reactive layer (obstacle avoidance)
   e) Move robot
   f) Process odometry
   g) Update EKF estimate
   h) Update visualization
   ```

4. **Results**:
   - Real-time visualization of robot pose and uncertainty
   - Error plots comparing true vs. estimated position

### Key Functions

- **`mission_planner()`**: Generates reference trajectory through multiple goals
- **`ctrl_pid()`**: Computes control commands to follow reference
- **`ctrl_reactive()`**: Modifies control to avoid obstacles
- **`ekf()`**: Updates state estimate using odometry and beacon measurements
- **`odometry()`**: Converts odometry readings to velocity commands

---

## Extended Kalman Filter (EKF)

### State Vector

The robot state is represented as:

```
x = [x, y, θ]ᵀ
```

where:

- `x, y`: Position in world coordinates (meters)
- `θ`: Orientation angle (radians)

### Process Model

The robot follows a differential drive model with velocities `v` (linear) and `ω` (angular).

#### Case 1: Non-zero Angular Velocity (ω ≠ 0)

When the robot is turning, it moves along an arc:

**State Prediction**:

```
x_k+1 = x_k + (v/ω) * [sin(θ_k + ω*dt) - sin(θ_k)]
y_k+1 = y_k - (v/ω) * [cos(θ_k + ω*dt) - cos(θ_k)]
θ_k+1 = θ_k + ω*dt
```

**Derivation**: The robot moves along a circular arc with radius `R = v/ω`. The arc length is `s = v*dt = R*ω*dt`. The change in position is:

- `Δx = R*[sin(θ + ω*dt) - sin(θ)]`
- `Δy = -R*[cos(θ + ω*dt) - cos(θ)]` (negative because y-axis convention)

**Jacobian F_k** (partial derivatives w.r.t. state):

```
F_k = [1, 0, (v/ω)*(cos(θ + ω*dt) - cos(θ));
       0, 1, (v/ω)*(sin(θ + ω*dt) - sin(θ));
       0, 0, 1]
```

**Input Jacobian L_k** (partial derivatives w.r.t. control):

```
L_k = [(sin(θ + ω*dt) - sin(θ))/ω,  (v/ω²)*(sin(θ) - sin(θ + ω*dt)) + (v/ω)*dt*cos(θ + ω*dt);
       -(cos(θ + ω*dt) - cos(θ))/ω, -(v/ω²)*(cos(θ) - cos(θ + ω*dt)) + (v/ω)*dt*sin(θ + ω*dt);
       0,                           dt]
```

#### Case 2: Zero Angular Velocity (ω ≈ 0)

When the robot moves straight:

**State Prediction**:

```
x_k+1 = x_k + v*cos(θ_k)*dt
y_k+1 = y_k + v*sin(θ_k)*dt
θ_k+1 = θ_k
```

**Jacobian F_k**:

```
F_k = [1, 0, -v*dt*sin(θ_k);
       0, 1,  v*dt*cos(θ_k);
       0, 0,  1]
```

**Input Jacobian L_k**:

```
L_k = [cos(θ_k)*dt, 0;
       sin(θ_k)*dt, 0;
       0,           dt]
```

### Covariance Prediction

```
P_k+1|k = F_k * P_k|k * F_kᵀ + L_k * Q * L_kᵀ
```

where:

- `P_k|k`: Current covariance matrix
- `Q`: Process noise covariance (from calibration)
- `F_k`: State transition Jacobian
- `L_k`: Input noise Jacobian

### Measurement Model

The robot observes beacons (landmarks) using range and bearing measurements.

**Sensor Position** (accounting for offset):

```
x_sensor = x + sensor_y_offset * cos(θ)
y_sensor = y + sensor_y_offset * sin(θ)
```

**Predicted Measurements** (for beacon `i` at position `[b_x, b_y]`):

```
dx = b_x - x_sensor
dy = b_y - y_sensor
D = √(dx² + dy²)

z_pred = [D,                    % Range
          atan2(dy, dx) - θ]    % Bearing (relative to robot heading)
```

**Measurement Jacobian H_k**:

For range measurement:

```
∂D/∂x = -dx/D
∂D/∂y = -dy/D
∂D/∂θ = (sensor_y_offset/D) * (dx*sin(θ) - dy*cos(θ))
```

For bearing measurement:

```
∂β/∂x = dy/D²
∂β/∂y = -dx/D²
∂β/∂θ = -(sensor_y_offset*(dx*cos(θ) + dy*sin(θ))/D²) - 1
```

**Complete H_k** (for each beacon):

```
H_k = [-dx/D,  -dy/D,  (sensor_y_offset/D)*(dx*sin(θ) - dy*cos(θ));
        dy/D², -dx/D², -(sensor_y_offset*(dx*cos(θ) + dy*sin(θ))/D²) - 1]
```

### Measurement Update

1. **Innovation**:

   ```
   y_k = z_k - z_pred
   ```

   (Bearing error is wrapped to [-π, π])

2. **Innovation Covariance**:

   ```
   S_k = H_k * P_k+1|k * H_kᵀ + R_k
   ```

   where `R_k` is the measurement noise covariance (block-diagonal for multiple beacons)

3. **Kalman Gain**:

   ```
   K_k = P_k+1|k * H_kᵀ * S_k⁻¹
   ```

4. **State Update**:

   ```
   x_k+1|k+1 = x_k+1|k + K_k * y_k
   ```

5. **Covariance Update**:
   ```
   P_k+1|k+1 = (I - K_k * H_k) * P_k+1|k
   ```

### Key Implementation Details

- **Angle Wrapping**: Bearing measurements are wrapped to `[0, 2π]` and errors to `[-π, π]`
- **Multiple Beacons**: Measurement covariance `R` is block-diagonal (Kronecker product)
- **Sensor Offset**: Accounts for sensor position relative to robot center
- **Numerical Stability**: Small epsilon check for `ω ≈ 0` to avoid division by zero

---

## Configuration

### Main Configuration (`config.m`)

#### Time Parameters

```matlab
dt = 0.2;        % Time step (seconds)
T_max = 40;      % Maximum mission time
```

#### Robot and Sensor Names

```matlab
robotName = 'Marvin';    % Robot name in Apolo
laserName = 'LMS100';    % Laser sensor name
```

#### Goal Poses

```matlab
goal_poses = [
    10,  10, 0;      % [x, y, θ] in meters and radians
    18,  18, pi/2;
    ...
];
```

#### Mission Parameters

```matlab
mission_params.hold_time = 5.0;      % Wait time at each goal (seconds)
mission_params.goal_tol  = 0.25;     % Position tolerance (meters)
mission_params.th_tol    = deg2rad(20); % Angular tolerance (radians)
```

#### Planner Parameters (A\*)

```matlab
planner_params.resolution = 0.01;        % Grid resolution (meters)
planner_params.lethal_radius = 0.08;     % Robot collision radius (meters)
planner_params.cost_gain = 80;           % Obstacle proximity penalty
planner_params.max_iterations = 3000000; % Maximum A* iterations
```

#### Controller Parameters (PID)

```matlab
controller_params.Kp_v = 1.5;  % Proportional gain for linear velocity
controller_params.Kp_w = 1.3;  % Proportional gain for angular velocity
```

#### Reactive Controller Parameters

```matlab
reactive_params.range_min = 0.05;        % Minimum laser range (meters)
reactive_params.range_max = 10.0;       % Maximum laser range (meters)
reactive_params.d_slow = 1.30;         % Distance to start slowing (meters)
reactive_params.d_stop = 0.45;         % Distance to stop (meters)
reactive_params.v_slow = 0.15;         % Safe velocity (m/s)
reactive_params.w_escape = 1.2;        % Escape rotation speed (rad/s)
```

#### EKF Parameters

```matlab
estimator_params.Q = diag([0.2871, 0.1084]*1.0e-03*20);  % Process noise
estimator_params.R = diag([0.0162, 0.0013]*20);          % Measurement noise
estimator_params.sensor_y_offset = 0.1;                  % Sensor offset (meters)
estimator_params.correction_factor = 0.02;               % Range bias correction
```

#### Sensor Parameters

```matlab
sensor_params.beacons_pos = [
    6.00,  7.00;    % Beacon positions [x, y] in world coordinates
    8.60, 11.00;
    ...
];
sensor_params.correction_factor = 0.02;
```

---

## Usage Examples

### Custom Mission

Modify `config.m` to set custom goals:

```matlab
goal_poses = [
    5,   5,  0;      % Start
    15, 10,  pi/2;   % Goal 1
    20, 15,  pi;     % Goal 2
    10, 18, -pi/2;   % Goal 3
];
```

Then run:

```matlab
main
```

### Testing Individual Components

```matlab
% Test EKF only
addpath(genpath('modules'));
x_est = [5; 6; 0];
P = diag([0.2, 0.2, 0.1]);
u = [0.5; 0.1];
z = [10.5; 0.3];  % [range, bearing]
beacons_xy = [15; 10];
[x_new, P_new] = ekf(x_est, u, z, beacons_xy, P, 0.2, estimator_params);
```

---

## Troubleshooting

### Apolo Not Responding

- Ensure Apolo is running and an environment is loaded
- Check that robot and sensor names match the XML file
- Verify MATLAB path includes Apolo/Matlab directory

### EKF Divergence

- Re-run calibration to update Q and R matrices
- Check beacon positions in `config.m` match the XML file
- Verify sensor offset and correction factor

### Path Planning Fails

- Ensure start and goal positions are in free space
- Check map resolution and obstacle inflation
- Increase `max_iterations` if path is complex

### Robot Collisions

- Adjust `lethal_radius` in planner parameters
- Increase `d_stop` and `d_slow` in reactive controller
- Check laser sensor is properly configured

---
