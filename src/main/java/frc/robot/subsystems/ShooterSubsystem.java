// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ChassisConstants;
import edu.wpi.first.math.MathUtil;
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
  private TalonFX turretMotor;
  private TalonFX hoodMotor;
  private TalonFX flyWheelMotor;

  private TalonFX elevatorMotor;

  private CANcoder hoodCancoder;

  private ProfiledPIDController hoodPID;
  private ProfiledPIDController turretPID;

  private SimpleMotorFeedforward hoodFF;
  private SimpleMotorFeedforward turretFF;

  private DigitalInput limitSwitch;

  private static InterpolatingDoubleTreeMap distanceToVoltageMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap();

  private ChassisSubsystem chassisSubsystem;

  public ShooterSubsystem(ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.turretMotor = new TalonFX(ShooterConstants.kTurretMotorID);
    this.flyWheelMotor = new TalonFX(ShooterConstants.kFlywheelMotorID);
    this.elevatorMotor = new TalonFX(ShooterConstants.kElevatorMotorID);

    this.hoodMotor = new TalonFX(ShooterConstants.kHoodMotorID);

    this.hoodCancoder = new CANcoder(ShooterConstants.kHoodCANcoderID);
    this.hoodCancoder.setPosition(Degrees.of(5));

    this.limitSwitch = new DigitalInput(ShooterConstants.kLimitSwitchID);

    this.hoodPID = new ProfiledPIDController(
        ShooterConstants.kHoodP,
        ShooterConstants.kHoodI,
        ShooterConstants.kHoodD,
        new Constraints(
            ShooterConstants.kHoodMaxVel,
            ShooterConstants.kHoodMaxAccel));
    this.hoodPID.setTolerance(ShooterConstants.kHoodTolerance);

    this.turretPID = new ProfiledPIDController(
        ShooterConstants.kTurretP,
        ShooterConstants.kTurretI,
        ShooterConstants.kTurretD,
        new Constraints(
            ShooterConstants.kTurretMaxVel,
            ShooterConstants.kTurretMaxAccel));
    this.turretPID.setTolerance(ShooterConstants.kTurretTolerance);

    this.hoodFF = new SimpleMotorFeedforward(
        ShooterConstants.kHoodKS,
        ShooterConstants.kHoodKV,
        ShooterConstants.kHoodKA);
    this.turretFF = new SimpleMotorFeedforward(
        ShooterConstants.kTurretKS,
        ShooterConstants.kTurretKV,
        ShooterConstants.kTurretKA);

    this.hoodPID.setGoal(0);
    initializeMaps();
  }

  private static void initializeMaps() {
    // Example data points for distance to Voltage mapping
    distanceToVoltageMap.put(2.0, 5.0);
    distanceToVoltageMap.put(2.5, 5.2);
    distanceToVoltageMap.put(3.0, 5.4);
    distanceToVoltageMap.put(3.25, 5.5);
    distanceToVoltageMap.put(3.5, 5.6);
    distanceToVoltageMap.put(4.0, 5.8);

    // Example data points for distance to Time of Flight (TOF) mapping
    //TODO: view vidoes of shooter and enter TOF
    distanceToTOF.put(4.0, 1.2);
    distanceToTOF.put(3.0,1.05);
    distanceToTOF.put(2.0, 1.0);

    // Example data points for distance to Pitch mapping
    distanceToPitch.put(2.0 , 60.0);
    distanceToPitch.put(2.5, 110.0);
    distanceToPitch.put(3.0, 145.0);
    distanceToPitch.put(3.25, 165.0);
    distanceToPitch.put(3.5, 190.0);
    distanceToPitch.put(4.0, 220.0);
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

  public Angle getHoodAngleDegs() {
    return this.hoodCancoder.getPosition().getValue();
  }

  public double getTurretAngleDegs() {
    return this.turretMotor.getPosition().getValue().in(Degrees) * ShooterConstants.kTurretGearRatio;
  }

  public void setTurretAngle(double deg) {
    this.turretPID.setGoal(deg);
  }

  public void setHoodAngle(double deg) {
    this.hoodPID.setGoal(deg);
  }

  public void setFlywheelVoltage(double V) {
    // VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    // this.flyWheelMotor.setControl(velocityVoltage.withVelocity(RPM / 60.0));
    this.flyWheelMotor.setVoltage(V);
  }

  public boolean getLimitSwitch() {
    return this.limitSwitch.get();
  }

  public double calculateAzimuthAngle(Pose2d robotPose, Translation3d target) {
    if (robotPose != null) {

      Translation2d turretPosition = robotPose.getTranslation();

      Translation2d direction = target.toTranslation2d().minus(turretPosition);

      SmartDashboard.putNumber("direction x", direction.getX());
      SmartDashboard.putNumber("direction y", direction.getY());

      double fieldAngleDeg = direction.getAngle().getDegrees();
      SmartDashboard.putNumber("atan", fieldAngleDeg);

      double turretAngleDeg = fieldAngleDeg - robotPose.getRotation().getDegrees();
      SmartDashboard.putNumber("turret angle deg", turretAngleDeg);

      return MathUtil.inputModulus(turretAngleDeg, -180, 180);
    } else {
      return -1;
    }
  }

  public void aimAtTarget(Pose2d robotPose, Translation3d target) {
    double azimuth = ShooterConstants.kAzimuthOffset - calculateAzimuthAngle(robotPose, target);
    SmartDashboard.putNumber("calculated azimuth", azimuth);
    setTurretAngle(azimuth);
  }
  public void bumpBallUp() {
    this.elevatorMotor.setVoltage(4.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate PID output
    double hoodPIDOutput = hoodPID.calculate(this.getHoodAngleDegs().in(Degrees));
    double turretPIDOutput = turretPID.calculate(this.getTurretAngleDegs());

    // Calculate feedforward output using the setpoint velocity
    double hoodFFOutput = hoodFF.calculate(hoodPID.getSetpoint().velocity);
    double turretFFOutput = turretFF.calculate(turretPID.getSetpoint().velocity);

    // Combine PID and feedforward outputs
    this.hoodMotor.set(hoodPIDOutput + hoodFFOutput);
    this.turretMotor.set(turretPIDOutput + turretFFOutput);

    SmartDashboard.putNumber("hood set point", hoodPID.getSetpoint().position);
    SmartDashboard.putNumber("turret set point", turretPID.getSetpoint().position);

    SmartDashboard.putNumber("hood actual", this.getHoodAngleDegs().in(Degrees));
    SmartDashboard.putNumber("turret actual", this.getTurretAngleDegs());

    SmartDashboard.putNumber("hood PID output", hoodPIDOutput);
    SmartDashboard.putNumber("hood FF output", hoodFFOutput);

    SmartDashboard.putNumber("turret PID output", turretPIDOutput);
    SmartDashboard.putNumber("turret FF output", turretFFOutput);

    // Reset the turret encoder position to 0 when the limit switch is triggered
    if (getLimitSwitch()) {
      this.turretMotor.setPosition(0);
    }

    // Prevent the turret from moving past the limit switch in the negative direction
    if (this.getTurretAngleDegs() <= 0 && turretMotor.getVelocity().getValue().in(RotationsPerSecond) < 0) {
      this.turretMotor.set(0);
    }
    // Prevent the turret from moving past the maximum angle in the positive direction
    if (this.getTurretAngleDegs() >= 200 && turretMotor.getVelocity().getValue().in(RotationsPerSecond) > 0) {
      this.turretMotor.set(0);
    }

    SmartDashboard.putNumber("azimuth",this.calculateAzimuthAngle(this.chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter()));
    SmartDashboard.putNumber("flywheel RPS", flyWheelMotor.getVelocity().getValue().in(RotationsPerSecond));

  }
}
