// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.1;
  }

  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Module wheel diameter in meters
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 5.27; // Module drive motor gear ratio
    public static final double kSteerMotorGearRatio = 12.8; // Module steer motor gear ratio

    public static double kFeedforwardGainSteer = 0.11; // The feed forward gain for the module steer control

    public static Slot0Configs getSteerMotorGains() {
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(30); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }

    public static Slot0Configs getDriveMotorGains() {
      Slot0Configs kSteerMotorGains = new Slot0Configs();
      kSteerMotorGains.withKP(0.15); // The proportional gain for the module steer control
      return kSteerMotorGains;
    }

    public static SimpleMotorFeedforward leftFrontFF = new SimpleMotorFeedforward(0.21599, 2.2476, 0.040257);
    public static SimpleMotorFeedforward leftBackFF = new SimpleMotorFeedforward(0.20676, 2.1653, 0.040257); // previous
                                                                                                             // constants
                                                                                                             // -
                                                                                                             // (0.20676,
                                                                                                             // 2.1653,
                                                                                                             // 0.16537)
    public static SimpleMotorFeedforward rightFrontFF = new SimpleMotorFeedforward(0.1788, 2.257, 0.040257);
    public static SimpleMotorFeedforward rightBackFF = new SimpleMotorFeedforward(0.11961, 2.3274, 0.040257);

    public static double kModuleAngleDeadband = 0.001;

    /* Swerve Current Limiting */
    public static final int kSteerCurrentLimit = 25;
    public static final int kSteerCurrentThreshold = 30;
    public static final double kSteerCurrentThresholdTime = 0.1;
    public static final boolean kSteerEnableCurrentLimit = true;

    public static final int kDriveCurrentLimit = 30;
    public static final int kDriveCurrentThreshold = 40;
    public static final double kDriveCurrentThresholdTime = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;
  }

  public static class ChassisConstants {

    public static final double width = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top

    public static final String leftCamName = "Camera-left";
    public static final String rightCamName = "Camera-right";

    public static final double leftCamHeight = 0.22; // meters, height of the left camera from the ground
    public static final double rightCamHeight = 0.22; // meters, height of the right camera from the ground

    public static final double leftCamPitch = Math.toRadians(20); // radians, pitch angle of the left camera
    public static final double rightCamPitch = Math.toRadians(20); // radians, pitch angle of the right camera
    public static final double leftCamYaw = Math.toRadians(30); // radians, yaw angle of the left camera
    public static final double rightCamYaw = Math.toRadians(-30); // radians, yaw angle of the right camera

    // public static final Translation3d hubTopCenter = new Translation3d(
    // AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(DriverStation.getAlliance().get()
    // == DriverStation.Alliance.Blue ? 26 : 10).get().getX() + width / 2.0,
    // AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getFieldWidth()
    // / 2.0,
    // height);

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);

    // public static Translation3d getHubTopCenter() {
    // int tagID = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    // == DriverStation.Alliance.Blue ? 26
    // : 10;
    // return new Translation3d(
    // fieldLayout.getTagPose(tagID).get().getX() + width / 2.0,
    // fieldLayout.getFieldWidth() / 2.0,
    // height);
    // }
    public static Translation3d getHubTopCenter() {
      boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
      int tagID = isBlue ? 26 : 10;
      double xOffset = isBlue ? width / 2.0 : -width / 2.0;

      return new Translation3d(
          fieldLayout.getTagPose(tagID).get().getX() + xOffset,
          fieldLayout.getFieldWidth() / 2.0,
          height);
    }

    // LF - 3
    // RF - 2
    // LB - 1
    // RB - 0
    // Ports for driving motors
    public static final int kLeftFrontDriveID = 13; // CAN ID
    public static final int kRightFrontDriveID = 12; // CAN ID
    public static final int kLeftBackDriveID = 11; // CAN ID
    public static final int kRightBackDriveID = 10; // CAN ID

    // Ports for angle motors
    public static final int kLeftFrontSteerID = 23; // CAN ID
    public static final int kRightFrontSteerID = 22; // CAN ID
    public static final int kLeftBackSteerID = 21; // CAN ID
    public static final int kRightBackSteerID = 20; // CAN ID

    // Ports for encoders
    public static final int kLeftFrontEncID = 33; // CAN ID
    public static final int kRightFrontEncID = 32; // CAN ID
    public static final int kLeftBackEncID = 31; // CAN ID
    public static final int kRightBackEncID = 30; // CAN ID

    // Offsets for absolute encoders in rotations (i.e: 360 degrees = 1 rotation):
    // public static final double kLeftFrontOffset = -0.029296875;
    // public static final double kRightFrontOffset = -0.4111328125;
    // public static final double kLeftBackOffset = 0.406494140625;
    // public static final double kRightBackOffset = -0.228515625;

    public static final double kLeftFrontOffset = -0.1897264414062505; // -0.18824683203125006
    public static final double kRightFrontOffset = -0.1503956777343749 ; // -0.12744140625
    public static final double kLeftBackOffset = -0.47119140625 ; // 0.05028120312499994
    public static final double kRightBackOffset = 0.3088078671875003 ; // -0.18798828125000006
    // Which motors are inverted: public static final boolean frontLeftDriveInverted
    // = true;
    public static final boolean kLeftFrontInverted = true;
    public static final boolean kRightFrontInverted = true;
    public static final boolean kLeftBackInverted = true;
    public static final boolean kRightBackInverted = true;

    public static double kMaxDrivingVelocity = 4.5;
    public static double kTeleDriveMaxAccelerationUnitsPerSec = 5;
    public static double kTeleDriveMaxSpeedMetersPerSec = kMaxDrivingVelocity;
    public static double kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI * 1.8;

    // Distance between centers of right and left wheels on robot meters (synced
    // with PathPlanner settings.json robotTrackwidth)
    public static final double kTrackWidth = 0.546;
    // Distance between front and back wheels on robot meters (synced with
    // PathPlanner module positions)
    public static final double kWheelBase = 0.546;
    // Drive wheel radius in meters (synced with PathPlanner settings.json
    // driveWheelRadius)
    public static final double kWheelRadius = 0.051;

    // the mass of the robot in KG (synced with PathPlanner settings.json robotMass)
    public static final double kMassKG = 53.0;
    // the moment of inertia of the robot (synced with PathPlanner settings.json
    // robotMOI)
    public static final double kMOI = 4.3;

    // Swerve Kinematics:
    // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    //     new Translation2d(kWheelBase / 2, kTrackWidth / 2)

    //     //right back and left fron switch
    //     // new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    //     // new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    //     // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    //     // new Translation2d(kWheelBase / 2, kTrackWidth / 2)

    // );

    public static final SwerveDriveKinematics kDriveKinematics = new
    SwerveDriveKinematics(
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Left front
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Right back
    new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Right front
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Left back
       
    
    );

    public static final RobotConfig DEFAUL_ROBOT_CONFIG = new RobotConfig(kMassKG, kMOI,
        new ModuleConfig(kWheelRadius,
            kMaxDrivingVelocity,
            kWheelBase, DCMotor.getKrakenX60(4),
            ModuleConstants.kDriveCurrentLimit, 1),
        kDriveKinematics.getModules());

    public static RobotConfig getConfig() {
      RobotConfig config;
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        e.printStackTrace();
        config = DEFAUL_ROBOT_CONFIG;
      }
      return config;
    }
  }

  public static class ShooterConstants {

    // TalonFX device IDs for the shooter subsystem
    public static final int kHoodMotorID = 60;
    public static final int kFlywheelMotor1ID = 50;
    public static final int kFlywheelMotor2ID = 51;
    public static final int kFlywheelMotor3ID = 52;

    // CANcoder device IDs for the shooter subsystem
    public static final int kHoodCANcoderID = 59;
    public static final double MagnetOffset = 0.1298828125;

    // Gear ratios for the shooter subsystem
    public static final double kTurretGearRatio = (18.0 / 100.0) * (1 / 9.0);

    // offset for the azimuth angle calculation, in degrees
    public static final double kAzimuthOffset = 105;

    public static final double kHoodHorizontalAngle = 80; // encoder angle when hood is horizontal (for gravity FF)
    public static final double kHoodLowLimit = 5;
    public static final double kHoodHighLimit = 359;

    // public static final double kTurretLowLimit = 0;
    // public static final double kTurretHighLimit = 200;
    // PID and feedforward constants for shooter subsystem
    // Hood (angle) controller

    public static final double kHoodP = 0.055; // increased for PID-only control
    public static final double kHoodI = 0.00000; // 0.00006
    public static final double kHoodD = 0.00; // increased further to dampen oscillation

    public static final double kHoodKS = 0.00;
    public static final double kHoodKV = 0.13;
    public static final double kHoodKA = 0.0;
    public static final double kHoodKG = 0.0; // disabled for debugging

    public static final double kHoodMaxVel = 45.0; // deg/s
    public static final double kHoodMaxAccel = 60.0; // reduced from 90.0 deg/s^2
    public static final double kHoodTolerance = 5;

    // Turret (angle) controller
    public static final double kTurretP = 0.015;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0;

    public static final double kTurretKS = 0.0;
    public static final double kTurretKV = 0.0;
    public static final double kTurretKA = 0.001;

    public static final double kTurretMaxVel = 0.0; // deg/s or appropriate units
    public static final double kTurretMaxAccel = 0.0; // deg/s^2 or appropriate units
    public static final double kTurretTolerance = 3.5;

    // open code's code-check
    public static final double kAimP = 0.015;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0;
    public static final double kAimMaxVel = 0.0;
    public static final double kAimMaxAccel = 0.0;
    public static final double kAimTolerance = 3.5;

    // 2026 REBUILT Field dimensions (Rebuilt Welded)
    public static final double kFieldLength = 16.459; // meters (54ft)
    public static final double kFieldWidth = 8.231; // meters (27ft)
    public static final double kFieldCenterX = kFieldLength / 2;

    // 2026 REBUILT Field Trench Zone
    // The trench is the center lane that runs the length of the field
    // Blue trench zone (red is mirrored)
    public static final double kTrenchBlueMinX = 5.5;
    public static final double kTrenchBlueMaxX = 11.0;
    public static final double kTrenchBlueMinY = 2.5;
    public static final double kTrenchBlueMaxY = 5.7;
  }

  public static final class StorageConstants {

    public static enum StorageState {
      REST, RELOAD
    }

    // CAN IDs for storage subsystem motors
    public static final int elevatorMotor1ID = 30;
    public static final int elevatorMotor2ID = 31;
    public static final int feedMotorID = 32;

    public static final double reloadVoltage = 4;
    public static final double elevatorVoltage = 3.5;
    public static final double reloadTime = 0;
  }

  public static final class IntakeConstants {
    public static enum intakeStates {
      REST, INTAKE
    }

    public static final int armMotorID = 43;
    public static final int rollerMotorID = 44;

    public static final int armEncoderID = 52;

    // intake power in Voltage
    public static final double intakePower = 5;

    public static final double kArmGearRatio = 18 / 52.0;

    public static final double restPoint = 130;
    public static final double intakePoint = 15;
    public static final double midPoint = 77;

    public static final double kp = 0.03;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double kMaxVelocity = 100;
    public static final double kMaxAcceleration = 200;

    public static final double pidTolerance = 10;
    public static final double MagnetOffset = -0.97;

    // ArmFeedforward constants
    public static final double kS = 0; // 0.2
    public static final double kG = 0; // 0.3
    public static final double kV = 0; // 0.5
    public static final double kA = 0; // 3
    public static final double armHorizontalDeg = 120;

    // Shake command constants (for ShakeIntakeCMD)
    public static final double shakeAmplitude = 10; // degrees to oscillate from center
    public static final double shakePeriod = 0.5; // seconds per half-cycle

  }

  // Safe angle ranges (degrees) for barrier checks in periodic
  // These are conservative defaults — update to match your physical limits.
  public static final double kHoodMinAngleDeg = 0.0;
  public static final double kHoodMaxAngleDeg = 60.0;

  public static final double kTurretMinAngleDeg = -180.0;
  public static final double kTurretMaxAngleDeg = 180.0;

  // 2026 REBUILT Field dimensions
  public static final double kFieldLength = 16.459; // meters (54ft)
  public static final double kFieldWidth = 8.231; // meters (27ft)
  public static final double kFieldCenterX = kFieldLength / 2;

  // 2026 REBUILT Welded Field - Trench Zones
  // The field has 4 trench lanes where robots approach the reef
  public static final class TrenchZones {
    // Blue trench X bounds (near blue alliance station)
    public static final double kBlueMinX = 5.5;
    public static final double kBlueMaxX = 11.0;

    // Red trench X bounds (near red alliance station, mirrored)
    public static final double kRedMinX = 5.459;
    public static final double kRedMaxX = 10.959;

    // Lane 1 - Left side (Y: 0 - 2.1)
    public static final double kLane1MinY = 0.0;
    public static final double kLane1MaxY = 2.1;

    // Lane 2 - Center-left (Y: 2.1 - 4.1)
    public static final double kLane2MinY = 2.1;
    public static final double kLane2MaxY = 4.1;

    // Lane 3 - Center-right (Y: 4.1 - 6.1)
    public static final double kLane3MinY = 4.1;
    public static final double kLane3MaxY = 6.1;

    // Lane 4 - Right side (Y: 6.1 - 8.231)
    public static final double kLane4MinY = 6.1;
    public static final double kLane4MaxY = 8.231;
  }
}
