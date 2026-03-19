// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
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
  private final int ITERATIONS = 10;
  private Translation3d target = Constants.ChassisConstants.getHubTopCenter();
  private static final double kLatencyMs = 30.0;
  private double prevVx = 0;
  private double prevVy = 0;
  private boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
  public ShootCMD(ChassisSubsystem chassisSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassisSubsystem = chassisSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec = 2;
    Constants.ChassisConstants.kMaxDrivingVelocity = 0.7;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds fieldSpeeds = chassisSubsystem.getFieldRelativeSpeeds();
    Pose2d robotPose = chassisSubsystem.getPose();
    
    double dt = kLatencyMs / 1000.0;
    double ay = chassisSubsystem.getGyro().getWorldLinearAccelY() * 9.81;
    double ax = chassisSubsystem.getGyro().getWorldLinearAccelX() * 9.81;
    // double ax = (fieldSpeeds.vxMetersPerSecond - prevVx) / 0.02;
    // double ay = (fieldSpeeds.vyMetersPerSecond - prevVy) / 0.02;

    robotPose = new Pose2d(
        robotPose.getX() + fieldSpeeds.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
        robotPose.getY() + fieldSpeeds.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
        robotPose.getRotation()
    );
    prevVx = fieldSpeeds.vxMetersPerSecond;
    prevVy = fieldSpeeds.vyMetersPerSecond;
    
    SmartDashboard.putNumber("log_predX", robotPose.getX());
    SmartDashboard.putNumber("log_predY", robotPose.getY());
    SmartDashboard.putNumber("log_actualX", chassisSubsystem.getPose().getX());
    SmartDashboard.putNumber("log_actualY", chassisSubsystem.getPose().getY());
    SmartDashboard.putNumber("log_vx", fieldSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("log_vy", fieldSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("log_speed", Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond));
    SmartDashboard.putNumber("log_ax", ax);
    SmartDashboard.putNumber("log_ay", ay);
    
    Pose2d turretPose = new Pose2d(robotPose.getTranslation().minus(new Translation2d(0.3,0)),robotPose.getRotation());
    
    Translation2d predictedTarget = Constants.ChassisConstants.getHubTopCenter().toTranslation2d();
    double distance = chassisSubsystem.getDistanceFromHub();
    double tof = shooterSubsystem.getTOFForDistance(distance);

    // Invert velocity for red alliance - field coordinates are flipped
    double vx = isBlue ? fieldSpeeds.vxMetersPerSecond : -fieldSpeeds.vxMetersPerSecond;
    double vy = isBlue ? fieldSpeeds.vyMetersPerSecond : -fieldSpeeds.vyMetersPerSecond;

    for(int i = 0; i< ITERATIONS; i++){
      predictedTarget = target.toTranslation2d().minus(
        new Translation2d(
          vx * tof,
          vy * tof
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

    SmartDashboard.putNumber("log_targetX", predictedTarget.getX());
    SmartDashboard.putNumber("log_targetY", predictedTarget.getY());
    SmartDashboard.putNumber("log_distance", distance);
    SmartDashboard.putNumber("log_hubX", Constants.ChassisConstants.getHubTopCenter().toTranslation2d().getX());
    SmartDashboard.putNumber("log_hubY", Constants.ChassisConstants.getHubTopCenter().toTranslation2d().getY());

    shooterSubsystem.aimAtTarget(turretPose, predictedTranslation3d);
    shooterSubsystem.setHoodAngle(shooterSubsystem.getPitchForDistance(distance));
    shooterSubsystem.setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(distance));
    
    SmartDashboard.putNumber("vx", isBlue ? fieldSpeeds.vxMetersPerSecond : -fieldSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("vy", isBlue ? fieldSpeeds.vyMetersPerSecond : -fieldSpeeds.vyMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelVoltage(0);
    Constants.ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec = 4;
    Constants.ChassisConstants.kMaxDrivingVelocity = 4.5;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
