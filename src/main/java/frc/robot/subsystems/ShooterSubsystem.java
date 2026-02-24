// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX turretMotor;
  private TalonFX hoodMotor;
  private TalonFX flyWheelMotor;

  private CANcoder hoodCancoder;

  private ProfiledPIDController hoodPID;
  private ProfiledPIDController turretPID;

  private SimpleMotorFeedforward hoodFF;
  private SimpleMotorFeedforward turretFF;

  

  private static InterpolatingDoubleTreeMap distanceToRPMMap = new InterpolatingDoubleTreeMap(); 
  private static InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap(); 
  private static InterpolatingDoubleTreeMap distanceToPitch = new InterpolatingDoubleTreeMap(); 

  public ShooterSubsystem() {
  this.turretMotor = new TalonFX(ShooterConstants.kTurretMotorID);
  this.flyWheelMotor = new TalonFX(ShooterConstants.kFlywheelMotorID);
  this.hoodMotor = new TalonFX(ShooterConstants.kHoodMotorID);

  this.hoodCancoder = new CANcoder(ShooterConstants.kHoodCANcoderID);
  this.hoodCancoder.setPosition(Degrees.of(5));

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
    
    initializeMaps();
  }
  

  private static void initializeMaps(){
    // Example data points for distance to RPM mapping
    distanceToRPMMap.put(0.0, 0.0);
    distanceToRPMMap.put(0.0, 0.0);
    distanceToRPMMap.put(0.0, 0.0);

    // Example data points for distance to Time of Flight (TOF) mapping
    distanceToTOF.put(0.0, 0.0);
    distanceToTOF.put(0.0, 0.0);
    distanceToTOF.put(0.0, 0.0);

    // Example data points for distance to Pitch mapping
    distanceToPitch.put(0.0, 0.0);
    distanceToPitch.put(0.0, 0.0);
    distanceToPitch.put(0.0, 0.0);
  }

  /**
   * @param distance The distance to the target in meters
   * @return The corresponding RPM for the flywheel based on the distance
   */
  public double getRPMForDistance(double distance){
    return distanceToRPMMap.get(distance);
  }
  
  /**
   * Returns the Time of Flight (TOF) value for a given distance.
   * @param distance The distance to the target in meters
   * @return The corresponding TOF value for the given distance
   */
  public double getTOFForDistance(double distance){
    return distanceToTOF.get(distance);
  }

  /**
   * Returns the required pitch angle for a given distance.
   * @param distance The distance to the target in meters
   * @return The corresponding pitch angle for the given distance
   */
  public double getPitchForDistance(double distance){
    return distanceToPitch.get(distance);
  }

  public Angle getHoodAngle(){
    return this.hoodCancoder.getPosition().getValue();
  }
  public Angle getTurrentAngle(){
    return this.turretMotor.getPosition().getValue();
  }

  public void setTurretAngle(double deg){
    this.turretPID.setGoal(deg);
  }
  public void setHoodAngle(double deg){
    this.hoodPID.setGoal(deg);
  }

  public void setFlywheelRPM(double power){
    this.flyWheelMotor.set(power);
  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  
  // Calculate PID output
  double hoodPIDOutput = hoodPID.calculate(this.getHoodAngle().in(Degrees));
  double turretPIDOutput = turretPID.calculate(this.getTurrentAngle().in(Degrees));
  
  // Calculate feedforward output using the setpoint velocity
  double hoodFFOutput = hoodFF.calculate(hoodPID.getSetpoint().velocity);
  double turretFFOutput = turretFF.calculate(turretPID.getSetpoint().velocity);
  
  // Combine PID and feedforward outputs
  this.hoodMotor.set(hoodPIDOutput + hoodFFOutput);
  this.turretMotor.set(turretPIDOutput + turretFFOutput);

  SmartDashboard.putNumber("hood set point", hoodPID.getSetpoint().position);
  SmartDashboard.putNumber("turret set point", turretPID.getSetpoint().position);
  SmartDashboard.putNumber("hood actual", this.getHoodAngle().in(Degrees));
  SmartDashboard.putNumber("turret actual", this.getTurrentAngle().in(Degrees));

  SmartDashboard.putNumber("hood PID output", hoodPIDOutput);
  SmartDashboard.putNumber("turret PID output", turretPIDOutput);

  }
}
