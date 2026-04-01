// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX armMotor;
  private TalonFX rollerMotor;

  private ProfiledPIDController pidController;
  private ArmFeedforward armFeedforward;

  private CANcoder armEncoder;

  private intakeStates target;
  private double factor = 0.45;
  private boolean manualMode = false;
  private double manualPower = 0;

  public IntakeSubsystem() {

    this.armMotor = new TalonFX(IntakeConstants.armMotorID);

    this.rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    this.armMotor.getConfigurator().apply(config);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    this.rollerMotor.getConfigurator().apply(rollerConfig);

    this.pidController = new ProfiledPIDController(
        IntakeConstants.kp,
        IntakeConstants.ki,
        IntakeConstants.kd,
        new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));

    pidController.setTolerance(IntakeConstants.pidTolerance);

    this.armFeedforward = new ArmFeedforward(
        IntakeConstants.kS,
        IntakeConstants.kG,
        IntakeConstants.kV,
        IntakeConstants.kA);

    // configuring cancoder
    this.armEncoder = new CANcoder(IntakeConstants.armEncoderID);
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor.MagnetOffset = IntakeConstants.MagnetOffset;
    this.armEncoder.getConfigurator().apply(canConfig);
    this.pidController.setGoal(IntakeConstants.restPoint);
  }

  public double getArmAngleDegrees() {
    armEncoder.getPosition().refresh();
    // return armEncoder.getPosition().getValue().in(Degrees) *
    // IntakeConstants.kArmGearRatio + 40;
    return this.armEncoder.getAbsolutePosition().getValueAsDouble() * 180 + 44.46;
  }

  public boolean isAtSetpoint() {
    return pidController.atGoal();
  }

  /**
   * sets the target based on the desired state
   * Automatically disables manual mode to allow PID control.
   *
   * @param state The desired state of the four bar
   */
  public void setArmState(intakeStates state) {
    disableManualMode();

    switch (state) {

      case REST:
        pidController.setGoal(IntakeConstants.restPoint);
        target = intakeStates.REST;
        break;

      case INTAKE:
        pidController.setGoal(IntakeConstants.intakePoint);
        target = intakeStates.INTAKE;
        break;

    }

  }

  /**
   * Sets a custom goal position for the arm.
   * Used by ShakeIntakeCMD to oscillate around a center point.
   * Automatically disables manual mode to allow PID control.
   *
   * @param goal The target angle in degrees
   */
  public void setArmGoal(double goal) {
    disableManualMode();
    pidController.setGoal(goal);
  }

  /**
   * sets the power of the arm motors (manual mode)
   * 
   * @param power The desired power of the arm motors
   */
  public void setArmPower(double power) {
    manualMode = true;
    manualPower = power;
    armMotor.set(power);
  }

  /**
   * Disables manual mode and returns to PID control
   */
  public void disableManualMode() {
    manualMode = false;
    manualPower = 0;
  }

  /**
   * sets the power of the roller motors based on the desired state
   * 
   * @param powered The desired state of the roller motors (powered or unpowered)
   */
  public void setRollerState(boolean powered) {
    setRollerVoltage(powered ? IntakeConstants.intakePower : 0);
  }

  public Command setMidPoint() {
    return new InstantCommand(() -> this.setArmGoal(77));
  }

  /**
   * set the power of the roller motors
   * 
   * @param voltage The desired power of the roller motors
   */
  public void setRollerVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  public ParallelCommandGroup toIntake() {
    return new ParallelCommandGroup(

        // 1. Starts rolling the rollers for intake
        new InstantCommand(() -> setRollerState(true)),

        // 2. Sets the arm down to the intake position
        new InstantCommand(() -> setArmState(intakeStates.INTAKE))

    );
  }

  public SequentialCommandGroup toRest() {
    return new SequentialCommandGroup(

        // 1. Returns the arm into its rest position
        new InstantCommand(() -> setArmState(intakeStates.REST)),

        // 2. Stops rolling the rollers
        new InstantCommand(() -> setRollerState(false))

    );
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("arm ang abs", this.armEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("arm ang", this.getArmAngleDegrees());
    SmartDashboard.putNumber("arm ang radians",
        Units.degreesToRadians(this.getArmAngleDegrees()) * IntakeConstants.kArmGearRatio);
    SmartDashboard.putNumber("arm target", pidController.getGoal().position);

    // If in manual mode, skip PID and use manual power
    if (manualMode) {
      this.armMotor.set(manualPower);
    } else {
      // PID output (in volts)
      double pidOutput = pidController.calculate(getArmAngleDegrees());

      // Feedforward: convert encoder degrees to radians relative to horizontal
      double angleRad = Math.toRadians(getArmAngleDegrees() - IntakeConstants.armHorizontalDeg);
      double velocityRadPerSec = Math.toRadians(pidController.getSetpoint().velocity);
      double ffOutput = armFeedforward.calculate(angleRad, velocityRadPerSec);

      SmartDashboard.putNumber("pid output", pidOutput);
      SmartDashboard.putNumber("ff output", ffOutput);
      SmartDashboard.putNumber("current velc", this.pidController.getVelocityError());

      if (this.pidController.getGoal().position == IntakeConstants.restPoint) {
        factor = 0.7;
      }

      if (this.pidController.getGoal().position == IntakeConstants.intakePoint) {
        factor = 1.5;
        pidOutput *=1.2;
      }
      double totalOutput = pidOutput + ffOutput;

      if (isAtSetpoint()) {
        this.armMotor.setVoltage(0);
      } else {
        this.armMotor.setVoltage(totalOutput * factor); // scale down for safety
      }
    }

    if (this.getArmAngleDegrees() < 50) {
      Constants.ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI;
    } else {
      Constants.ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec = Math.PI * 1.8;
    }
  }
}
