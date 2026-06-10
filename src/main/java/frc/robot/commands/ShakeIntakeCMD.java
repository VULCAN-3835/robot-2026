// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants.intakeStates;

/**
 * Command that shakes the intake arm up and down with direct motor power (no
 * PID).
 * Goes up for 0.3s, then down for 0.3s, repeats 4 times, then returns to rest.
 */
public class ShakeIntakeCMD extends SequentialCommandGroup {

  public ShakeIntakeCMD(IntakeSubsystem intakeSubsystem) {
    double seconds = 0.25;
    double power = 0.15;
    addRequirements(intakeSubsystem);

    addCommands(
        new InstantCommand(() -> intakeSubsystem.setRollerState(true)),
        new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)),
        new InstantCommand(() -> intakeSubsystem.setArmGoal(45)),
        new WaitUntilCommand(() -> intakeSubsystem.isAtSetpoint()),
        // Shake up and down 4 times
        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),
        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),
        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),
        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),
        new InstantCommand(() -> intakeSubsystem.setArmPower(power), intakeSubsystem),
        new WaitCommand(seconds),

        new InstantCommand(() -> intakeSubsystem.setArmPower(-power), intakeSubsystem),
        new WaitCommand(seconds),

        // Stop motor and return to rest (manual mode auto-disabled by setArmState)
        new InstantCommand(() -> intakeSubsystem.setArmPower(0), intakeSubsystem),
        new InstantCommand(() -> intakeSubsystem.setRollerState(false)),
        new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.REST), intakeSubsystem));
  }
}