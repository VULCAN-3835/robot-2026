// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCMD extends Command {
  /** Creates a new ShootCMD. */

  private ChassisSubsystem chassisSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private final int ITERATIONS = 2;
  private Translation3d target = Constants.ChassisConstants.getHubTopCenter();

  public ShootCMD(ChassisSubsystem chassisSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = chassisSubsystem.getPose();
    Pose2d turretPose = new Pose2d(robotPose.getTranslation().minus(new Translation2d(0.3,0)),robotPose.getRotation());
    ChassisSpeeds fieldSpeeds = chassisSubsystem.getFieldRelativeSpeeds();
    Translation2d predictedTarget = Constants.ChassisConstants.getHubTopCenter().toTranslation2d();
    double distance = chassisSubsystem.getDistanceFromHub();
    double tof = shooterSubsystem.getTOFForDistance(distance);

    for(int i = 0; i< ITERATIONS; i++){
      predictedTarget = target.toTranslation2d().minus(
        new Translation2d(
          fieldSpeeds.vyMetersPerSecond * tof,
          fieldSpeeds.vxMetersPerSecond * tof
        )
      );
      distance = turretPose.getTranslation().getDistance(predictedTarget);
      tof = shooterSubsystem.getTOFForDistance(distance);
    }

    Translation3d predictedTranslation3d = new Translation3d(
      predictedTarget.getX(),
      predictedTarget.getY(),
      target.getZ()
    );

    shooterSubsystem.aimAtTarget(turretPose, predictedTranslation3d);
    shooterSubsystem.setHoodAngle(shooterSubsystem.getPitchForDistance(distance));
    shooterSubsystem.setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(distance));
    SmartDashboard.putNumber("vx", fieldSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("vy", fieldSpeeds.vyMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
