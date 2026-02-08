// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX turretMotor;
  private TalonFX hoodMotor;
  private TalonFX flyWheelMotor;

  private CANcoder hoodCancoder;
  private CANcoder turretCancoder;

  private ProfiledPIDController hoodPID;
  private ProfiledPIDController turretPID;

  private SimpleMotorFeedforward hoodFF;
  private SimpleMotorFeedforward turretFF;

  private double turretTolerence = 5;
  private double hoodTolerence = 3;

  public ShooterSubsystem() {
    this.turretMotor = new TalonFX(0);
    this.flyWheelMotor = new TalonFX(0);
    this.hoodMotor = new TalonFX(0);

    this.hoodCancoder = new CANcoder(0);
    this.turretCancoder = new CANcoder(0);

    this.hoodPID = new ProfiledPIDController(0, 0, 0, null);
    this.hoodPID.setTolerance(hoodTolerence);
    this.turretPID = new ProfiledPIDController(0, 0, 0, null);
    this.turretPID.setTolerance(hoodTolerence);

    this.hoodFF = new SimpleMotorFeedforward(0, 0, 0);
    this.turretFF = new SimpleMotorFeedforward(0, 0, 0);

  }
  public Angle getHoodAngle(){
    return this.hoodCancoder.getAbsolutePosition().getValue();
  }
  public Angle getTurrentAngle(){
    return this.turretCancoder.getAbsolutePosition().getValue();
  }

  public void setTurretAngle(double deg){
    this.turretPID.setGoal(deg);
  }
  public void setHoodAngle(double deg){
    this.hoodPID.setGoal(deg);
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
  }
}
