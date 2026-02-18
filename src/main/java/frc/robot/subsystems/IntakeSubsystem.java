// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX armMotor;
  private TalonFX rollerMotor;

  private ProfiledPIDController pidController;

  private DigitalInput restLimitSwitch;
  private DigitalInput intakeLimitSwitch;

  private CANcoder armEncoder;

  private intakeStates target;

  public IntakeSubsystem() {

    this.armMotor = new TalonFX(IntakeConstants.armMotorID);

    this.rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
    this.pidController = new ProfiledPIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd, new Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    pidController.setTolerance(IntakeConstants.pidTolerance);

    pidController.setGoal(IntakeConstants.restPoint);
    this.target = intakeStates.REST;

    this.restLimitSwitch = new DigitalInput(IntakeConstants.restLimitSwitchID);
    this.intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID);

    this.armEncoder = new CANcoder(IntakeConstants.armEncoderID);

  }

  public boolean getRestLimitSwitch() {
      return restLimitSwitch.get();
  }

  public boolean getIntakeLimitSwitch() {
    return intakeLimitSwitch.get();
  }

  public Angle getArmAngle() {
    return armEncoder.getAbsolutePosition().getValue();
  }

  /**
   * sets the target based on the desired state
   * 
   * @param state The desired state of the four bar
   */
  public void setArmState(intakeStates state) {

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
   * sets the power of the arm motors
   * 
   * @param power The desired power of the arm motors
   */
  public void setArmPower(double power) {
    armMotor.set(power);
  }

  /**
   * sets the power of the roller motors based on the desired state
   * 
   * @param powered The desired state of the roller motors (powered or unpowered)
   */
  public void setRollerState(boolean powered) {
    setRollerPower(powered ? IntakeConstants.intakePower : 0);
  }

  /**
   * set the power of the roller motors
   * 
   * @param power The desired power of the roller motors
   */
  public void setRollerPower(double power) {
    rollerMotor.set(power);
  }

  public SequentialCommandGroup toIntake() {
    return new SequentialCommandGroup(

    
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
    // This method will be called once per scheduler run
    
    
    if (!(target == intakeStates.REST && getRestLimitSwitch()) && !(target == intakeStates.INTAKE && getIntakeLimitSwitch())) {
      double armOutput = pidController.calculate(getArmAngle().in(Degrees));
      setArmPower(armOutput);
    }
    else {
      setArmPower(0);
    }
  }
}
