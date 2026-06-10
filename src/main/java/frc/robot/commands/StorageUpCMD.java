// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.StorageConstants.StorageState;
import frc.robot.subsystems.StorageSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StorageUpCMD extends Command {
  /** Creates a new StorageUpCMD. */
  private StorageSubsystem storageSubsystem;

  public StorageUpCMD(StorageSubsystem storageSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storageSubsystem = storageSubsystem;
    addRequirements(storageSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storageSubsystem.setElevatorMotorPower(Constants.StorageConstants.elevatorPower);
    storageSubsystem.setFeedMotorState(StorageState.RELOAD);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storageSubsystem.setElevatorMotorPower(0);
    storageSubsystem.setFeedMotorState(StorageState.REST);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
