// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX leftArmMotor;
  private TalonFX rightArmMotor;

  private TalonFX leftRollerMotor;
  private TalonFX rightRollerMotor;

  private PIDController pidController;

  private double target;

  public IntakeSubsystem() {

    this.leftArmMotor = new TalonFX(IntakeConstants.leftArmMotorID); // Leader
    this.rightArmMotor = new TalonFX(IntakeConstants.rightArmMotorID); // Follower
    this.rightArmMotor.setControl(new Follower(leftArmMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    this.leftRollerMotor = new TalonFX(IntakeConstants.leftRollerMotorID); // Leader
    this.rightRollerMotor = new TalonFX(IntakeConstants.rightRollerMotorID); // Follower
    this.rightRollerMotor.setControl(new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    this.pidController = new PIDController(IntakeConstants.kp, IntakeConstants.ki, IntakeConstants.kd);
    pidController.setTolerance(IntakeConstants.pidTolerance);

    this.target = IntakeConstants.restPoint;
  }

  /**
   * sets the target based on the desired state
   * 
   * @param state The desired state of the four bar
   */
  public void setArmState(intakeStates state) {
    switch (state) {

      case REST:
        target = IntakeConstants.restPoint;
        break;

      case SOURCE:
        target = IntakeConstants.sourcePoint;
        break;

      default:
        target = IntakeConstants.restPoint;
    }

  }

  /**
   * sets the power of the arm motors
   * 
   * @param power The desired power of the arm motors
   */
  public void setArmPower(double power) {
    leftArmMotor.set(power);
  }

  /**
   * sets the power of the roller motors based on the desired state
   * 
   * @param powered The desired state of the roller motors (powered or unpowered)
   */
  public void setRollerState(boolean powered) {
    setRollerPower(powered ? IntakeConstants.intakePower : 0);
  }

  /**
   * set the power of the roller motors
   * 
   * @param power The desired power of the roller motors
   */
  public void setRollerPower(double power) {
    leftRollerMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double currentPos = leftArmMotor.getPosition().getValueAsDouble();
    double output = pidController.calculate(currentPos, target);
    setArmPower(output);
  }
}
