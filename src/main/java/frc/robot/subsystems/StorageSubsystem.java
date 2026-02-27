// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.StorageConstants;
import frc.robot.Constants.StorageConstants.StorageState;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new StorageSubsystem. */
  private TalonFX feedMotor;
  private TalonFX ElevatorMotor;

  private StorageState state;

  public StorageSubsystem() {

    this.feedMotor = new TalonFX(StorageConstants.feedMotorID);
    this.ElevatorMotor = new TalonFX(StorageConstants.elevatorMotorID);
    this.state = StorageState.REST;

  }

  public void setFeedMotorState(StorageState state) {
    this.state = state;
    switch (state) {
      case REST:
        setFeedMotorPower(0);
        break;
      case RELOAD:
        setFeedMotorPower(StorageConstants.reloadPower);
    }
  }

  private void setFeedMotorPower(double power) {
    feedMotor.set(power);
  }

  public Command setFeedMotorStateCMD(StorageState state) {
    return new InstantCommand(() -> setFeedMotorState(state));
  }

  public SequentialCommandGroup reloadFeedMotorCMD() {
    StorageState currentState = this.state;
    return new SequentialCommandGroup(
      setFeedMotorStateCMD(StorageState.RELOAD),
      new WaitCommand(StorageConstants.reloadTime),
      setFeedMotorStateCMD(currentState)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}