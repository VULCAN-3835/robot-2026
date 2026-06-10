// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StorageConstants.StorageState;
import frc.robot.subsystems.StorageSubsystem;

/**
 * A smart storage command that monitors feed motor current draw.
 * If the current exceeds the threshold (indicating a jam), 
 * the motor reverses for 0.3 seconds then resumes normal operation.
 */
public class SmartStorageCMD extends Command {
  private final StorageSubsystem storageSubsystem;
  private final double elevatorPower;
  private final double feedPower;
  private final boolean runElevator;
  private final boolean runFeed;
  
  // Current monitoring state
  private boolean isReversing = false;
  private final Timer reverseTimer = new Timer();
  private static final double REVERSE_DURATION = 0.3; // seconds
  
  // Current threshold in amps - adjust this based on your motor
  private static final double CURRENT_THRESHOLD = 30.0;

  /**
   * Creates a SmartStorageCMD that runs both motors.
   * 
   * @param storageSubsystem The storage subsystem
   * @param elevatorPower Power for the elevator motor (-1.0 to 1.0)
   * @param feedPower Power for the feed/rollers motor (-1.0 to 1.0)
   */
  public SmartStorageCMD(StorageSubsystem storageSubsystem, double elevatorPower, double feedPower) {
    this.storageSubsystem = storageSubsystem;
    this.elevatorPower = elevatorPower;
    this.feedPower = feedPower;
    this.runElevator = true;
    this.runFeed = true;
    addRequirements(storageSubsystem);
  }

  /**
   * Creates a SmartStorageCMD with only feed motor control.
   * 
   * @param storageSubsystem The storage subsystem
   * @param feedPower Power for the feed/rollers motor (-1.0 to 1.0)
   */
  public SmartStorageCMD(StorageSubsystem storageSubsystem, double feedPower) {
    this.storageSubsystem = storageSubsystem;
    this.elevatorPower = 0;
    this.feedPower = feedPower;
    this.runElevator = false;
    this.runFeed = true;
    addRequirements(storageSubsystem);
  }

  /**
   * Creates a SmartStorageCMD that stops both motors.
   * 
   * @param storageSubsystem The storage subsystem
   */
  public SmartStorageCMD(StorageSubsystem storageSubsystem) {
    this.storageSubsystem = storageSubsystem;
    this.elevatorPower = 0;
    this.feedPower = 0;
    this.runElevator = true;
    this.runFeed = true;
    addRequirements(storageSubsystem);
  }

  @Override
  public void initialize() {
    isReversing = false;
    reverseTimer.stop();
    reverseTimer.reset();
  }

  @Override
  public void execute() {
    // Set elevator motor power if needed
    if (runElevator) {
      storageSubsystem.setElevatorMotorPower(elevatorPower);
    }
    
    // Handle feed motor with current monitoring
    if (runFeed) {
      if (isReversing) {
        // We're in reversal mode - check if we should exit
        if (reverseTimer.get() >= REVERSE_DURATION) {
          // Reversal complete, resume normal operation
          isReversing = false;
          reverseTimer.stop();
          reverseTimer.reset();
          storageSubsystem.setFeedMotorPower(feedPower);
        } else {
          // Continue reversing
          storageSubsystem.setFeedMotorPower(-feedPower);
        }
      } else {
        // Normal operation - check for overcurrent
        double current = storageSubsystem.getFeedMotorCurrent();
        if (current > CURRENT_THRESHOLD && feedPower != 0) {
          // Current too high - start reversal
          isReversing = true;
          reverseTimer.reset();
          reverseTimer.start();
          storageSubsystem.setFeedMotorPower(-feedPower);
        } else {
          // Normal operation
          storageSubsystem.setFeedMotorPower(feedPower);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop both motors when command ends
    if (runElevator) {
      storageSubsystem.setElevatorMotorPower(0);
    }
    if (runFeed) {
      storageSubsystem.setFeedMotorState(StorageState.REST);
    }
    reverseTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until explicitly interrupted
  }
}
