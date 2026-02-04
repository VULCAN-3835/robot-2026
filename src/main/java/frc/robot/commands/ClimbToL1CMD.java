// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbSubsystemConstants.climbStatesHeight;
import frc.robot.Constants.ClimbSubsystemConstants.climbStatesVelocities;
import frc.robot.subsystems.ClimbSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToL1CMD extends SequentialCommandGroup {
  /** Creates a new ClimbToL1CMD. */
  ClimbSubsystem climbSubsystem;
  climbStatesHeight heightState;
  climbStatesVelocities speedState;
  public ClimbToL1CMD(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.climbSubsystem = climbSubsystem;
    this.heightState = climbStatesHeight.POS1;
    this.speedState = climbStatesVelocities.L1IN;

    addCommands(
      //1. set height to the PID
      new InstantCommand(() -> climbSubsystem.setHeight(heightState)),
      //2. climb to L1
      new InstantCommand(() -> climbSubsystem.setSpeed(speedState)),
      //3. check if we reached the goal
      new WaitUntilCommand(() -> climbSubsystem.isAtSetPoint()),
      //4. stop the motor
      new InstantCommand(() -> climbSubsystem.stopMotor())
    );
  }
}
