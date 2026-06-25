// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ShooterConstants;

import frc.robot.Constants.ChassisConstants;
import frc.robot.commands.SetChassisAngleCMD;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX hoodMotor;
  private TalonFX flyWheelMotor1;
  private TalonFX flyWheelMotor2;
  private TalonFX flyWheelMotor3;


  private CANcoder hoodCancoder;

  private ProfiledPIDController hoodPID;

  private ArmFeedforward hoodFF;

  private static InterpolatingDoubleTreeMap distanceToVoltageMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap();
  private static double angOffSetMap = 0;//40;
  private static double voltageOffSetMap = -0.1;
  private static double TOFOffset = 0;

  private ChassisSubsystem chassisSubsystem;
  

  public ShooterSubsystem(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.flyWheelMotor1 = new TalonFX(ShooterConstants.kFlywheelMotor1ID);
    this.flyWheelMotor2 = new TalonFX(ShooterConstants.kFlywheelMotor2ID);
    this.flyWheelMotor3 = new TalonFX(ShooterConstants.kFlywheelMotor3ID);
    
    

    TalonFXConfiguration motor1Config = new TalonFXConfiguration();
    motor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    this.flyWheelMotor1.getConfigurator().apply(motor1Config);

    this.flyWheelMotor2.setControl(new Follower(ShooterConstants.kFlywheelMotor1ID, MotorAlignmentValue.Opposed));
    this.flyWheelMotor3.setControl(new Follower(ShooterConstants.kFlywheelMotor1ID, MotorAlignmentValue.Opposed));


    this.hoodMotor = new TalonFX(ShooterConstants.kHoodMotorID);

    this.hoodCancoder = new CANcoder(ShooterConstants.kHoodCANcoderID);
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor.MagnetOffset = ShooterConstants.MagnetOffset;
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    this.hoodCancoder.getConfigurator().apply(canConfig);
    this.hoodPID = new ProfiledPIDController(
        ShooterConstants.kHoodP,
        ShooterConstants.kHoodI,
        ShooterConstants.kHoodD,
        new Constraints(
            ShooterConstants.kHoodMaxVel,
            ShooterConstants.kHoodMaxAccel));
    this.hoodPID.setTolerance(ShooterConstants.kHoodTolerance);

    this.hoodFF = new ArmFeedforward(
        ShooterConstants.kHoodKS,
        ShooterConstants.kHoodKG,
        ShooterConstants.kHoodKV,
        ShooterConstants.kHoodKA);


    this.hoodPID.setGoal(180);
    this.hoodCancoder.setPosition(5/360.0);
    initializeMaps();
  }

  private static void initializeMaps() {
    // Example data points for distance to Voltage mapping
    distanceToVoltageMap.put(1.5, 4.8 + voltageOffSetMap);
    distanceToVoltageMap.put(2.0, 4.9 + voltageOffSetMap);
    distanceToVoltageMap.put(2.25, 5.0 + voltageOffSetMap);
    distanceToVoltageMap.put(2.5, 5.1 + voltageOffSetMap);
    distanceToVoltageMap.put(2.75, 5.2 + voltageOffSetMap);
    distanceToVoltageMap.put(3.0, 5.3 + voltageOffSetMap);
    distanceToVoltageMap.put(3.25, 5.4 + voltageOffSetMap);
    distanceToVoltageMap.put(3.5, 5.5 + voltageOffSetMap);
    distanceToVoltageMap.put(3.75, 5.6 + voltageOffSetMap);
    distanceToVoltageMap.put(4.0, 5.7 + voltageOffSetMap);

    // Example data points for distance to Time of Flight (TOF) mapping
    distanceToTOF.put(1.5, 1.1 + TOFOffset);
    distanceToTOF.put(2.0, 1.1 + TOFOffset);
    distanceToTOF.put(2.25, 1.1 +TOFOffset);
    distanceToTOF.put(2.5, 1.1 + TOFOffset);
    distanceToTOF.put(2.75, 1.1 + TOFOffset);
    distanceToTOF.put(3.0, 1.1 + TOFOffset);
    distanceToTOF.put(3.25, 1.1 + TOFOffset);
    distanceToTOF.put(3.5, 1.1 + TOFOffset);
    distanceToTOF.put(3.75, 1.1 + TOFOffset);
    distanceToTOF.put(4.0, 1.1 + TOFOffset);

    // Example data points for distance to Pitch mapping
    distanceToPitch.put(1.5, 70.0 + angOffSetMap);
    distanceToPitch.put(2.0, 130.0 + angOffSetMap);
    distanceToPitch.put(2.25, 160.0 + angOffSetMap);
    distanceToPitch.put(2.5, 200.0 + angOffSetMap);
    distanceToPitch.put(2.75, 230.0 + angOffSetMap);
    distanceToPitch.put(3.0, 260.0 + angOffSetMap);
    distanceToPitch.put(3.25, 290.0 + angOffSetMap);
    distanceToPitch.put(3.5, 300.0 + angOffSetMap);
    distanceToPitch.put(3.75, 320.0 + angOffSetMap);
    distanceToPitch.put(4.0, 330.0 + angOffSetMap);
  }

  public static void scaleUpVoltage(){
    voltageOffSetMap += 0.1;
    initializeMaps();
  }
  public static void scaleDownVoltage(){
    voltageOffSetMap-=0.1;
    initializeMaps();
  }

  /**
   * @param distance The distance to the target in meters
   * @return The corresponding RPM for the flywheel based on the distance
   */
  public double getVoltageForDistance(double distance) {
    return distanceToVoltageMap.get(distance);
  }

  /**
   * Returns the Time of Flight (TOF) value for a given distance.
   * 
   * @param distance The distance to the target in meters
   * @return The corresponding TOF value for the given distance
   */
  public double getTOFForDistance(double distance) {
    return distanceToTOF.get(distance);
  }

  /**
   * Returns the required pitch angle for a given distance.
   * 
   * @param distance The distance to the target in meters
   * @return The corresponding pitch angle for the given distance
   */
  public double getPitchForDistance(double distance) {
    return distanceToPitch.get(distance);
  }

  public Angle getHoodAngle() {
    return this.hoodCancoder.getAbsolutePosition().getValue();
  }

  public double getHoodAngleDegs() {
    double rawAngle = this.hoodCancoder.getAbsolutePosition().getValue().in(Degrees);
    // Convert from signed range (-180 to +180) to unsigned range (0 to 360)
    if (rawAngle < 0) {
      rawAngle += 360.0;
    }
    return rawAngle;
  }

  public double getChassisAngleDegs() {
    return chassisSubsystem.getYaw();
  }

  public void setHoodAngle(double deg) {
    if (deg >= ShooterConstants.kHoodLowLimit && deg <= ShooterConstants.kHoodHighLimit) {
      this.hoodPID.setGoal(deg);

    }
  }

  public void setFlywheelVoltage(double V) {
    // VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    // this.flyWheelMotor.setControl(velocityVoltage.withVelocity(RPM / 60.0));
    this.flyWheelMotor1.setVoltage(V);
    }

  public double calculateAzimuthAngle(Pose2d robotPose, Translation3d target) {
    if (robotPose != null) {

      Translation2d chassisPosition = robotPose.getTranslation();

      Translation2d direction = target.toTranslation2d().minus(chassisPosition);

      SmartDashboard.putNumber("Shooter/direction x", direction.getX());
      SmartDashboard.putNumber("Shooter/direction y", direction.getY());

      double fieldAngleDeg = direction.getAngle().getDegrees();
      SmartDashboard.putNumber("Shooter/atan", fieldAngleDeg);

      double chassisAngleDeg = fieldAngleDeg - robotPose.getRotation().getDegrees();
      SmartDashboard.putNumber("Shooter/chassis angle deg", chassisAngleDeg);

      return MathUtil.inputModulus(chassisAngleDeg, -180, 180);
    } else {
      return -1;
    }
  } 

  public double getAzimuth(Pose2d robotPose, Translation3d target) {
    double azimuth = ShooterConstants.kAzimuthOffset - calculateAzimuthAngle(robotPose, target);
    SmartDashboard.putNumber("Shooter/calculated azimuth", azimuth);

    return azimuth;
  }
//open code's code-check
  public double getTargetFieldHeading(Pose2d robotPose, Translation3d target) {
    Translation2d direction = target.toTranslation2d().minus(robotPose.getTranslation());
    return direction.getAngle().getDegrees();
}

  /**
   * Sets whether the turret should automatically home at startup.
   * 
   * @param shouldHome true to enable auto-homing, false to disable
   */


  /**
   * Returns whether the turret has completed homing.
   * 
   * @return true if homing is complete
   */


  /**
   * Checks if the robot is currently in any of the 4 trench zones of the 2026 REBUILT field.
   * Automatically checks based on alliance color.
   * 
   * @return true if the robot is in any trench lane, false otherwise
   */

  //TODO: make it work, or in periodic or in default command.
  public boolean isInTrench() {
    Pose2d robotPose = chassisSubsystem.getPose();
    return (robotPose.getX() > 4.3 && robotPose.getX()<5) || (robotPose.getX() <12.3 && robotPose.getX()>11.6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Check if robot is in trench zone - if so, set hood to 0
    // if (isInTrench()) {
    //   this.hoodPID.setGoal(0);
    // }

    // Calculate PID output
    double hoodPIDOutput = hoodPID.calculate(this.getHoodAngle().in(Degrees));

    // Calculate feedforward output using the setpoint velocity
    // Offset the angle so 0° = horizontal for correct gravity compensation
    double angleFromHorizontal = hoodPID.getSetpoint().position - ShooterConstants.kHoodHorizontalAngle;
    double hoodFFOutput = hoodFF.calculate(Math.toRadians(angleFromHorizontal), hoodPID.getSetpoint().velocity);

    // Combine PID and feedforward outputs
    
    //TODO: Uncomment this line to enable hood control
    // this.hoodMotor.setVoltage(hoodPIDOutput + hoodFFOutput);

    SmartDashboard.putNumber("Shooter/hood set point", hoodPID.getSetpoint().position);

    SmartDashboard.putNumber("Shooter/hood actual", this.getHoodAngle().in(Degrees));
    SmartDashboard.putNumber("Shooter/turret actual", this.getChassisAngleDegs());

    SmartDashboard.putNumber("Shooter/hood PID output", hoodPIDOutput);
    SmartDashboard.putNumber("Shooter/hood FF output", hoodFFOutput);

    SmartDashboard.putNumber("Shooter/azimuth",
        this.calculateAzimuthAngle(this.chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter()));
    SmartDashboard.putNumber("Shooter/flywheel RPS", flyWheelMotor1.getVelocity().getValue().in(RotationsPerSecond));
    SmartDashboard.putNumber("Shooter/TOF", this.getTOFForDistance(chassisSubsystem.getDistanceFromHub()));

    SmartDashboard.putNumber("Shooter/hood abs ang", this.hoodCancoder.getAbsolutePosition().getValueAsDouble());
  }
}
