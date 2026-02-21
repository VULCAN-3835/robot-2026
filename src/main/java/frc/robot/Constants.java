// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants {

    // TalonFX device IDs for the shooter subsystem
    public static final int kTurretMotorID = 0;
    public static final int kHoodMotorID = 0;
    public static final int kFlywheelMotorID = 0;
  // CANcoder device IDs for the shooter subsystem
  public static final int kHoodCANcoderID = 0;
  public static final int kTurretCANcoderID = 0;
    
  // PID and feedforward constants for shooter subsystem
  // Hood (angle) controller
  public static final double kHoodP = 0.0;
  public static final double kHoodI = 0.0;
  public static final double kHoodD = 0.0;
  public static final double kHoodKS = 0.0;
  public static final double kHoodKV = 0.0;
  public static final double kHoodKA = 0.0;
  public static final double kHoodMaxVel = 0.0; // deg/s or appropriate units
  public static final double kHoodMaxAccel = 0.0; // deg/s^2 or appropriate units
  public static final double kHoodTolerance = 3.0;

  // Turret (angle) controller
  public static final double kTurretP = 0.0;
  public static final double kTurretI = 0.0;
  public static final double kTurretD = 0.0;
  public static final double kTurretKS = 0.0;
  public static final double kTurretKV = 0.0;
  public static final double kTurretKA = 0.0;
  public static final double kTurretMaxVel =     0.0; // deg/s or appropriate units
  public static final double kTurretMaxAccel = 0.0; // deg/s^2 or appropriate units
  public static final double kTurretTolerance = 5.0;

  }
}
