// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.commands.DefaultTeleopCommand;

import frc.robot.commands.ShootDelayCMD;
import frc.robot.commands.StorageUpCMD;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.StorageSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(chassisSubsystem);
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();

  private final CommandXboxController xboxControllerDrive = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController xboxControllerButton = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot",
        new ParallelCommandGroup(
            new InstantCommand(() -> shooterSubsystem
                .setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(chassisSubsystem.getDistanceFromHub()))),
            new InstantCommand(() -> shooterSubsystem
                .setHoodAngle(shooterSubsystem.getPitchForDistance(chassisSubsystem.getDistanceFromHub()))),
            new InstantCommand(
                () -> shooterSubsystem.aimAtTarget(chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter())),
            storageSubsystem.runStorage()));

    NamedCommands.registerCommand("intake", new ParallelCommandGroup(
        new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)),
        new InstantCommand(() -> intakeSubsystem.setRollerVoltage(Constants.IntakeConstants.intakePower))));
    NamedCommands.registerCommand("close shooter", new ParallelCommandGroup(
        new InstantCommand(() -> shooterSubsystem.setFlywheelVoltage(0), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setHoodAngle(0)),
        storageSubsystem.stopStorage()));

    NamedCommands.registerCommand("shootMove",
        new ParallelCommandGroup(new ShootDelayCMD(shooterSubsystem, storageSubsystem, chassisSubsystem)));

    NamedCommands.registerCommand("storage", new StorageUpCMD(storageSubsystem));

    NamedCommands.registerCommand("intakeUp", new ParallelCommandGroup(
        new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.REST)),
        new InstantCommand(() -> intakeSubsystem.setRollerState(true))));

    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      e.printStackTrace();
      autoChooser = new SendableChooser<>();
    }

    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Left trigger intakes while true, and turns off rollers when released
    xboxControllerDrive.leftTrigger().whileTrue(new ParallelCommandGroup(
        new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)),
        new InstantCommand(() -> intakeSubsystem.setRollerState(true))));
    xboxControllerDrive.leftTrigger().onFalse(new InstantCommand(() -> intakeSubsystem.setRollerState(false)));

    //B button to close the intake
    xboxControllerDrive.b()
        .onTrue(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.REST), intakeSubsystem));

    //Right bumper sets to mid point while true, and returns to intake when released
    xboxControllerDrive.rightBumper().whileTrue(intakeSubsystem.setMidPoint());
    xboxControllerDrive.rightBumper()
        .toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)));


    //Right trigger shoots while true, and turns off flywheel, hood, and storage when released
    xboxControllerDrive.rightTrigger().whileTrue(new ParallelCommandGroup(
        new ShootDelayCMD(shooterSubsystem, storageSubsystem, chassisSubsystem),
        new InstantCommand(() -> xboxControllerDrive.setRumble(RumbleType.kBothRumble, 0.7))));
    xboxControllerDrive.rightTrigger().toggleOnFalse(new ParallelCommandGroup(
        new InstantCommand(() -> shooterSubsystem.setFlywheelVoltage(0), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.setHoodAngle(0)),
        storageSubsystem.stopStorage(), 
        new InstantCommand(() -> xboxControllerDrive.setRumble(RumbleType.kBothRumble, 0)),
        new InstantCommand(() -> intakeSubsystem.setRollerState(false))));

    //D-pad up and down scale the flywheel voltage up and down by 10%
    xboxControllerDrive.povUp().onTrue(new InstantCommand(()->shooterSubsystem.scaleUpVoltage()));
    xboxControllerDrive.povDown().onTrue(new InstantCommand(()->shooterSubsystem.scaleDownVoltage()));

    setUpContollers(true);

  }

  private void setUpContollers(boolean oneController) {

    chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(chassisSubsystem,
        () -> xboxControllerDrive.getLeftY(),
        () -> xboxControllerDrive.getLeftX(),
        () -> -xboxControllerDrive.getRightX()));

    xboxControllerDrive.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    configureButtonBinding(oneController ? xboxControllerDrive : xboxControllerButton);
  }

  private void configureButtonBinding(CommandXboxController cmdXboxController) {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("Left-Nuetral-Depot");
    // return new PathPlannerAuto("Left-Depot");
    return new PathPlannerAuto("Depot-Nuetral");
    
  }
}
