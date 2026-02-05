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
public class FullClimbCMD extends SequentialCommandGroup {
  /** Creates a new FullClimbCMD. */
   ClimbSubsystem climbSubsystem;
  
   climbStatesHeight heightState2;
   climbStatesHeight heightState3;

   climbStatesVelocities speedStateInL2;
   climbStatesVelocities speedStateOutL2;
   climbStatesVelocities speedStateInL3;
   climbStatesVelocities speedStateOutL3;
   
  public FullClimbCMD(ClimbSubsystem climbSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.climbSubsystem = climbSubsystem;
    this.heightState2 = climbStatesHeight.POS2;
    this.heightState3 = climbStatesHeight.POS1;
    this.speedStateOutL2 = climbStatesVelocities.L2OUT;
    this.speedStateInL2 = climbStatesVelocities.L2IN;
    this.speedStateOutL3 = climbStatesVelocities.L3OUT;
    this.speedStateInL3 = climbStatesVelocities.L3IN;
    addCommands(
      //1. climbs to L1
      new ClimbToL1CMD(climbSubsystem),
      //2. set the goal height to L2 for outer elevator
      new InstantCommand(() -> climbSubsystem.setHeight(heightState2)),
      //3.sets the speed
      new InstantCommand(() -> climbSubsystem.setSpeed(speedStateOutL2)),
      //4.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> climbSubsystem.isAtSetPoint()),
      //5.stops the motor 
      new InstantCommand(() -> climbSubsystem.stopMotor()),
      //6. set the goal height to L2 for inner elevator
      new InstantCommand(() -> climbSubsystem.setHeight(heightState3)),
      //7.sets the speed
      new InstantCommand(() -> climbSubsystem.setSpeed(speedStateInL2)),
      //8.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> climbSubsystem.isAtSetPoint()),
      //9.stops the motor
      new InstantCommand(() -> climbSubsystem.stopMotor()),
      //10. set the goal height to L3 for outer elevator
      new InstantCommand(() -> climbSubsystem.setHeight(heightState2)),
      //11.sets the speed
      new InstantCommand(() -> climbSubsystem.setSpeed(speedStateOutL3)),
      //12.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> climbSubsystem.isAtSetPoint()),
      //13.stops the motor
      new InstantCommand(() -> climbSubsystem.stopMotor())
    );
  }
}
