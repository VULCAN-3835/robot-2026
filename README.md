# FRC 2026 Robot

Command-based Java robot using WPILib 2026 with swerve drive and shooter mechanism.

## Quick Start

```bash
./gradlew build          # Build the project
./gradlew test           # Run tests
./gradlew simulateJava   # Run simulation
./gradlew deploy         # Deploy to robot
```

## Subsystems

| Subsystem | Purpose |
|-----------|---------|
| **Chassis** | 4-wheel swerve drive with NavX gyro, dual-camera vision pose estimation, and PathPlanner auto |
| **Shooter** | Turret with azimuth/elevation control, flywheel, and lookup-table distance aiming |
| **Intake** | Pivoting 4-bar arm with ground rollers, position-controlled with PID+feedforward |
| **Storage** | Ball transfer elevator with feed motor |

## Hardware

- **Motors:** CTRE TalonFX (CAN)
- **Encoders:** CTRE CANcoder
- **Gyro:** NavX-MXP (SPI)
- **Vision:** 2x PhotonVision cameras
- **Drive:** MK4i Swerve Modules

## Project Structure

```
src/main/java/frc/robot/
├── subsystems/      # Robot subsystems
├── commands/        # Commands and autos
├── Util/           # Utilities (SwerveModule, camera helpers)
├── Constants.java  # Robot constants
└── RobotContainer.java  # Bindings & config
```

## Key Features

- **Field-relative swerve** with vision-corrected odometry
- **Autonomous path following** via PathPlanner
- **Dynamic shooter aiming** based on robot pose
- **SysID routines** for drive characterization

## Requirements

- WPILib 2026.2.1+
- Java 17
- CTRE Phoenix 6
- PathPlannerLib

---
*Team FRC 2026*
