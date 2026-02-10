// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;

  private TalonFX leftRollerMotor;
  private TalonFX rightRollerMotor;

  private ProfiledPIDController pidController;

  public IntakeSubsystem() {

    this.leftArmMotor = new TalonFX(IntakeConstants.leftArmMotorID); // Leader
    this.rightArmMotor = new TalonFX(IntakeConstants.rightArmMotorID); // Follower
    this.rightArmMotor.setControl(new Follower(leftArmMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    this.leftRollerMotor = new TalonFX(IntakeConstants.leftRollerMotorID); // Leader
    this.rightRollerMotor = new TalonFX(IntakeConstants.rightRollerMotorID); // Follower
    this.rightRollerMotor.setControl(new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    this.pidController = new ProfiledPIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd, new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    pidController.setTolerance(IntakeConstants.pidTolerance);

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
        break;

      case SOURCE:
        pidController.setGoal(IntakeConstants.sourcePoint);
        break;

    }

  }

  /**
   * sets the power of the arm motors
   * 
   * @param power The desired power of the arm motors
   */
  public void setArmPower(double power) {
    leftArmMotor.set(power);
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
    leftRollerMotor.set(power);
  }

  public SequentialCommandGroup toIntake() {
    return new SequentialCommandGroup(

    
      // 1. Starts rolling the rollers for intake
      new InstantCommand(() -> setRollerState(true)),

      // 2. Sets the arm down to the source position
      new InstantCommand(() -> setArmState(intakeStates.SOURCE))

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
    
    double currentPos = leftArmMotor.getPosition().getValueAsDouble();
    double output = pidController.calculate(currentPos);
    setArmPower(output);
  }
}
