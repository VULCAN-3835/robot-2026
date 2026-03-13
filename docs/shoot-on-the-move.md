# Shoot-on-the-Move Documentation

**Status:** Partially Working (6/10)  
**Last Updated:** March 2026  
**Related Code:** `src/main/java/frc/robot/commands/ShootCMD.java`

## Overview

This feature enables the robot to accurately shoot while moving, compensating for:
1. System latency (30ms)
2. Robot velocity during projectile flight time
3. Alliance-specific coordinate system differences

## How It Works

### Algorithm Flow

1. **Get Current State**
   - Robot pose from odometry
   - Field-relative velocity from swerve modules
   - Gyro acceleration for latency compensation

2. **Latency Compensation**
   - Predict robot position 30ms ahead using:
     ```
     pose_future = pose_now + (velocity * dt) + (0.5 * acceleration * dt²)
     ```

3. **Iterative Target Prediction**
   - Run 10 iterations to converge on correct aim point
   - Each iteration:
     - Calculate where hub will be after ball's flight time
     - Account for robot movement during that time
     - Recalculate distance and update flight time
   
4. **Alliance Compensation** (FIXED ✓)
   - BLUE: Use velocity as-is
   - RED: Invert velocity before prediction
   - This fixes the coordinate system inversion on RED alliance

5. **Aim and Shoot**
   - Calculate turret azimuth to predicted target
   - Set hood angle based on distance lookup table
   - Set flywheel voltage based on distance lookup table

## Current Issues (Why 6/10)

### Working:
- ✓ Alliance color handling (RED vs BLUE) - **FIXED**
- ✓ Target prediction logic
- ✓ Distance-based parameter lookup
- ✓ Turret aiming calculation

### Not Working Optimally:
- ⚠ Accuracy varies with speed (better at low speeds)
- ⚠ Acceleration compensation may be noisy
- ⚠ Gyro acceleration might not be field-relative
- ⚠ Iterative convergence could be optimized
- ⚠ Lookup tables need tuning for actual ballistics
- ⚠ No feedback loop to verify hits

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kLatencyMs` | 30.0 | System latency in milliseconds |
| `ITERATIONS` | 10 | Convergence iterations for target prediction |
| Turret offset | 0.3m | Distance from robot center to turret |

## Lookup Tables

Located in `ShooterSubsystem.java`:

- `distanceToVoltageMap`: Flywheel voltage vs distance
- `distanceToTOF`: Time of flight vs distance  
- `distanceToPitch`: Hood angle vs distance

**Note:** These tables need calibration with actual shooting data.

## Debugging

Dashboard logs available:
- `log_predX/Y` - Predicted robot position
- `log_actualX/Y` - Actual robot position
- `log_vx/vy` - Field-relative velocity
- `log_targetX/Y` - Predicted target position
- `log_distance` - Distance to predicted target
- `log_ax/ay` - Acceleration values

## Future Improvements

1. **Add Vision Feedback**
   - Use camera to verify ball trajectory
   - Closed-loop adjustment based on where ball actually goes

2. **Better Acceleration Handling**
   - Filter gyro acceleration data
   - Consider using velocity derivative instead

3. **Dynamic Lookup Tables**
   - Auto-calibrate based on shot results
   - Account for battery voltage drops

4. **Shooting Modes**
   - Stationary shot (high accuracy)
   - Slow movement shot (medium accuracy)
   - Fast movement shot (spray and pray)

5. **Convergence Optimization**
   - Use Newton-Raphson instead of fixed iterations
   - Add early exit when change is small

## Testing Tips

1. Start with robot stationary - verify basic aiming works
2. Test at slow speed (0.5 m/s) moving toward/away from target
3. Test at medium speed (1.0 m/s)
4. Test lateral movement (strafing while shooting)
5. Compare hit rates between BLUE and RED alliances
6. Log data and analyze prediction vs actual

## Related Files

- `ShootCMD.java` - Main command implementation
- `ShooterSubsystem.java` - Shooter hardware and lookup tables
- `ChassisSubsystem.java` - Odometry and velocity calculations
- `Constants.java` - Field dimensions and hub position

## Alliance Bug Fix History

**Problem:** Shooter missed on RED alliance while working on BLUE  
**Root Cause:** Field coordinates are consistent (+X toward RED side), but robot orientation differs. When moving toward hub on RED, velocity is negative, but we were subtracting it, effectively adding to target position.

**Solution:** Invert velocity components for RED alliance before target prediction:
```java
double vx = isBlue ? fieldSpeeds.vxMetersPerSecond : -fieldSpeeds.vxMetersPerSecond;
double vy = isBlue ? fieldSpeeds.vyMetersPerSecond : -fieldSpeeds.vyMetersPerSecond;
```

**Status:** ✓ Fixed and tested
