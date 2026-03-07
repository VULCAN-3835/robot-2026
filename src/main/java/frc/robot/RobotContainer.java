// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.Constants.StorageConstants.StorageState;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.StorageSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private StorageSubsystem storageSubsystem = new StorageSubsystem();

  private final CommandXboxController xboxControllerDrive =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController xboxControllerButton =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //  autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.setDefaultOption("EMPTY", null);
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
      m_driverController.b().onTrue(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.REST)));
      m_driverController.a().onTrue(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)));
      m_driverController.leftBumper().onTrue(new InstantCommand(()->intakeSubsystem.setRollerState(true)));
      m_driverController.leftBumper().onFalse(new InstantCommand(()->intakeSubsystem.setRollerState(false)));


    // xboxControllerDrive.leftTrigger().whileTrue(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(Constants.StorageConstants.reloadPower)));
    xboxControllerDrive.leftTrigger().whileTrue(new SequentialCommandGroup(new InstantCommand(()->storageSubsystem.setFeedMotorState(StorageState.RELOAD)),new InstantCommand(()->storageSubsystem.setElevatorMotorPower(Constants.StorageConstants.reloadPower))));
    xboxControllerDrive.leftTrigger().whileFalse(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(0)));
  }
  private void setUpContollers(boolean oneController) {

    // chassisSubsystem.setDefaultCommand(new DefaultTeleopCommand(chassisSubsystem,
    //     () -> xboxControllerDrive.getLeftY(),
    //     () -> xboxControllerDrive.getLeftX(),
    //     () -> xboxControllerDrive.getRightX()));

    // xboxControllerDrive.start().onTrue(new InstantCommand(() -> chassisSubsystem.zeroHeading()));

    

    
    configureButtonBinding(oneController ? xboxControllerDrive : xboxControllerButton);
  }

  private void configureButtonBinding(CommandXboxController cmdXboxController) {
    // Here we will configure the button bindings

    // uses


  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; // autoChooser.getSelected();
  }
}
