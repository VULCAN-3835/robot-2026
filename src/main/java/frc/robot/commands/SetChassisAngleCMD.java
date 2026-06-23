// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetChassisAngleCMD extends Command {
  /** Creates a new SetChassisAngleCMD. */
  private ChassisSubsystem chassisSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private ProfiledPIDController pidController;

  public SetChassisAngleCMD(ChassisSubsystem chassisSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pidController = new ProfiledPIDController(
        ShooterConstants.kAimP,
        ShooterConstants.kAimI,
        ShooterConstants.kAimD,
        new Constraints(
            ShooterConstants.kAimMaxVel,
            ShooterConstants.kAimMaxAccel));
    this.pidController.setTolerance(ShooterConstants.kAimTolerance);
    pidController.enableContinuousInput(-180, 180);

    addRequirements(chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pidController.setGoal(this.shooterSubsystem.getTargetFieldHeading(chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter()));

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
