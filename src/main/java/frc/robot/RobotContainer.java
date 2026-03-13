// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.Constants.StorageConstants.StorageState;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShootCMD;
import frc.robot.commands.Turn90;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.StorageSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem(chassisSubsystem);
  private StorageSubsystem storageSubsystem = new StorageSubsystem();

  private final CommandXboxController xboxControllerDrive =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController xboxControllerButton =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("EMPTY", null);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("shoot",
    new ParallelCommandGroup(
      new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(chassisSubsystem.getDistanceFromHub()))),
      new InstantCommand(()->shooterSubsystem.setHoodAngle(shooterSubsystem.getPitchForDistance(chassisSubsystem.getDistanceFromHub()))),
      new InstantCommand(()->shooterSubsystem.aimAtTarget(chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter()))));
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
    
      xboxControllerDrive.b().onTrue(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.REST)));
      xboxControllerDrive.a().onTrue(new InstantCommand(() -> intakeSubsystem.setArmState(intakeStates.INTAKE)));
      xboxControllerDrive.leftBumper().onTrue(new InstantCommand(()->intakeSubsystem.setRollerState(true)));
      xboxControllerDrive.leftBumper().onFalse(new InstantCommand(()->intakeSubsystem.setRollerState(false)));


    // xboxControllerDrive.leftTrigger().whileTrue(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(Constants.StorageConstants.reloadPower)));
    xboxControllerDrive.leftTrigger().whileTrue(new SequentialCommandGroup(new InstantCommand(()->storageSubsystem.setFeedMotorState(StorageState.RELOAD)),new InstantCommand(()->storageSubsystem.setElevatorMotorPower(Constants.StorageConstants.reloadPower))));
    xboxControllerDrive.leftTrigger().whileFalse(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(0)));
   
    xboxControllerDrive.leftTrigger().whileTrue(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(0.5)));
    xboxControllerDrive.leftTrigger().toggleOnFalse(new InstantCommand(()->storageSubsystem.setElevatorMotorPower(0)));
    
    xboxControllerDrive.rightTrigger().whileTrue(new SequentialCommandGroup(new InstantCommand(()->storageSubsystem.setFeedMotorPower(-0.7)),new WaitCommand(0.1),storageSubsystem.setFeedMotorStateCMD(StorageState.RELOAD)));
    xboxControllerDrive.rightTrigger().toggleOnFalse(new InstantCommand(()->storageSubsystem.setFeedMotorState(StorageState.REST)));

    xboxControllerDrive.x().whileTrue(new ParallelCommandGroup(
      new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(shooterSubsystem.getVoltageForDistance(chassisSubsystem.getDistanceFromHub()))),
      new InstantCommand(()->shooterSubsystem.setHoodAngle(shooterSubsystem.getPitchForDistance(chassisSubsystem.getDistanceFromHub()))),
      new InstantCommand(()->shooterSubsystem.aimAtTarget(chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter()))));
    xboxControllerDrive.x().toggleOnFalse(new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(0)));
    setUpContollers(true);

    xboxControllerDrive.a().whileTrue(new SequentialCommandGroup(new InstantCommand(()->Constants.ChassisConstants.kMaxDrivingVelocity = 1),new ShootCMD(chassisSubsystem, shooterSubsystem)));
    xboxControllerDrive.a().toggleOnFalse(new SequentialCommandGroup(new InstantCommand(()->Constants.ChassisConstants.kMaxDrivingVelocity = 4.5),new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(0),shooterSubsystem),new InstantCommand(()->shooterSubsystem.setHoodAngle(0))));

    // xboxControllerDrive.x().onTrue(new InstantCommand(()->shooterSubsystem.aimAtTarget(chassisSubsystem.getPose(), ChassisConstants.getHubTopCenter())));
    xboxControllerDrive.y().whileTrue(new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(5.7)));
    xboxControllerDrive.y().toggleOnFalse(new InstantCommand(()->shooterSubsystem.setFlywheelVoltage(0)));
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
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}

