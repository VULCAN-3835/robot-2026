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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.StorageConstants;
import frc.robot.Constants.StorageConstants.StorageState;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new StorageSubsystem. */
  private TalonFX feedMotor;
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;

  private StorageState state;

  public StorageSubsystem() {

    this.feedMotor = new TalonFX(StorageConstants.feedMotorID);
    this.elevatorMotor1 = new TalonFX(StorageConstants.elevatorMotor1ID);
    this.elevatorMotor2 = new TalonFX(StorageConstants.elevatorMotor2ID);

    this.elevatorMotor2.setControl(new Follower(StorageConstants.elevatorMotor1ID, MotorAlignmentValue.Aligned));

    // this.elevatorMotor.setControl(new Follower(feedMotor.getDeviceID(),
    // MotorAlignmentValue.Opposed));
    this.state = StorageState.REST;

  }

  public void setFeedMotorState(StorageState state) {
    this.state = state;
    switch (state) {
      case REST:
        setFeedMotorPower(0);
        break;
      case RELOAD:
        setFeedMotorPower(StorageConstants.reloadVoltage);
    }
  }

  public void setFeedMotorPower(double V) {
    feedMotor.setVoltage(V);
  }

  /**
   * Gets the current draw of the feed motor in amps.
   * 
   * @return The stator current of the feed motor
   */
  public double getFeedMotorCurrent() {
    return feedMotor.getStatorCurrent().getValueAsDouble();
  }

  public void setElevatorMotorPower(double V) {
    elevatorMotor1.setVoltage(V);
  }

  public Command setFeedMotorStateCMD(StorageState state) {
    return new InstantCommand(() -> setFeedMotorState(state));
  }

  public SequentialCommandGroup reloadFeedMotorCMD() {
    StorageState currentState = this.state;
    return new SequentialCommandGroup(
        setFeedMotorStateCMD(StorageState.RELOAD),
        new WaitCommand(StorageConstants.reloadTime),
        setFeedMotorStateCMD(currentState));
  }

  /**
   * Runs both storage motors using power from constants.
   * Elevator uses elevatorPower, feed uses reloadPower.
   * 
   * @return Command that runs until interrupted
   */
  public ParallelCommandGroup runStorage() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> this.setFeedMotorPower(StorageConstants.reloadVoltage)),
        new InstantCommand(() -> this.setElevatorMotorPower(StorageConstants.elevatorVoltage)));
  }

  /**
   * Stops both storage motors.
   * 
   * @return Command that stops motors immediately
   */
  public ParallelCommandGroup stopStorage() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> this.setFeedMotorPower(0)),
        new InstantCommand(() -> this.setElevatorMotorPower(0)));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}