// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ArmStates;
import frc.robot.Constants.IntakeConstants.RollerStates;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX armMotor;
  private TalonFX rollerMotor;

  private ProfiledPIDController pidController;

  private DigitalInput restLimitSwitch;
  private DigitalInput intakeLimitSwitch;

  private CANcoder armEncoder;

  private ArmStates armTarget;
  private RollerStates rollerState;

  public IntakeSubsystem() {

    this.armMotor = new TalonFX(IntakeConstants.armMotorID);

    this.rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
    this.pidController = new ProfiledPIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd, new Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    pidController.setTolerance(IntakeConstants.pidTolerance);

    pidController.setGoal(IntakeConstants.restPoint);
    this.armTarget = ArmStates.REST;

    this.restLimitSwitch = new DigitalInput(IntakeConstants.restLimitSwitchID);
    this.intakeLimitSwitch = new DigitalInput(IntakeConstants.intakeLimitSwitchID);

    this.armEncoder = new CANcoder(IntakeConstants.armEncoderID);

  }

  private boolean getRestLimitSwitch() {
      return restLimitSwitch.get();
  }

  private boolean getIntakeLimitSwitch() {
    return intakeLimitSwitch.get();
  }

  private Angle getArmAngle() {
    return armEncoder.getAbsolutePosition().getValue();
  }

  public void setArmState(ArmStates state) {
    armTarget = state;
    switch (state) {

      case REST:
        pidController.setGoal(IntakeConstants.restPoint);
        break;

      case INTAKE:
        pidController.setGoal(IntakeConstants.intakePoint);
        break;
      

    }

  }

  private void setArmPower(double power) {
    armMotor.set(power);
  }

  public void setRollerState(RollerStates state) {
    rollerState = state;
    switch (state) {
      case REST:
        setRollerPower(0);
        break;
      case INTAKE:
        setRollerPower(IntakeConstants.intakePower);
        break;
      case RELEASE:
        setRollerPower(IntakeConstants.releasePower);
    }
  }

  private void setRollerPower(double power) {
    rollerMotor.set(power);
  }

  public SequentialCommandGroup toIntake() {
    return new SequentialCommandGroup(

    
      // 1. Starts rolling the rollers for intake
      new InstantCommand(() -> setRollerState(RollerStates.INTAKE)),

      // 2. Sets the arm down to the intake position
      new InstantCommand(() -> setArmState(ArmStates.INTAKE))

    );
  }

  public SequentialCommandGroup toRest() {
    return new SequentialCommandGroup(

      // 1. Returns the arm into its rest position
      new InstantCommand(() -> setArmState(ArmStates.REST)),

      // 2. Stops rolling the rollers
      new InstantCommand(() -> setRollerState(RollerStates.REST))

    );
  }

  public SequentialCommandGroup releaseRollerCMD() {
    RollerStates currentRollerState = rollerState;
    return new SequentialCommandGroup(

      // 1. Set rollers to release balls that are stuck
      new InstantCommand(() -> setRollerState(RollerStates.RELEASE)),

      // 2. Wait for balls to release
      new WaitCommand(IntakeConstants.releaseTime),

      // 3. Set the roller to its original state
      new InstantCommand(() -> setRollerState(currentRollerState))

    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
    if (!(armTarget == ArmStates.REST && getRestLimitSwitch()) && !(armTarget == ArmStates.INTAKE && getIntakeLimitSwitch())) {
      double armOutput = pidController.calculate(getArmAngle().in(Degrees));
      setArmPower(armOutput);
    }
    else {
      setArmPower(0);
    }
  }
}
