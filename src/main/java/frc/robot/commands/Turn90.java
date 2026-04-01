// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChassisSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Turn90 extends Command {
  /** Creates a new Turn90. */
  private ChassisSubsystem chassisSubsystem;
  private ProfiledPIDController pidController;

  public Turn90(ChassisSubsystem chassisSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.pidController = new ProfiledPIDController(0.05, 0, 0, new Constraints(0, 0));

    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pidController.setGoal(chassisSubsystem.getYaw() + 90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = this.pidController.calculate(chassisSubsystem.getYaw());
    chassisSubsystem.drive(0, 0, output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.drive(0,0,0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.pidController.atGoal();
  }
}
