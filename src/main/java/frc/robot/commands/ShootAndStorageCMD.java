// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.ChassisSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndStorageCMD extends SequentialCommandGroup {
  /** Creates a new ShootAndStorageCMD. */
  private ShooterSubsystem shooterSubsystem;
  private StorageSubsystem storageSubsystem;
  private ChassisSubsystem chassisSubsystem;

  public ShootAndStorageCMD(ShooterSubsystem shooterSubsystem,StorageSubsystem storageSubsystem,ChassisSubsystem chassisSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.storageSubsystem = storageSubsystem;
    this.chassisSubsystem = chassisSubsystem;
    addCommands(
        new InstantCommand(() -> {
            double dist = chassisSubsystem.getDistanceFromHub();
            shooterSubsystem.setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(dist));
        }),
        new WaitCommand(1.2),
        new ParallelCommandGroup(
            new InstantCommand(() -> storageSubsystem.setElevatorMotorPower(StorageConstants.elevatorVoltage)),
            new InstantCommand(() -> storageSubsystem.setFeedMotorPower(StorageConstants.reloadVoltage))));
    
  }
}
