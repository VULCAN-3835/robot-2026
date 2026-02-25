// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants.climbStatesHeight;
import frc.robot.Constants.ClimbSubsystemConstants.climbStatesVelocities;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

  TalonFX climbMotor;
  
  DigitalInput limitSwitch;

  ProfiledPIDController pidController;
  double output;
  double currentHeight;

  climbStatesHeight heightStateOutUPL1;
  climbStatesHeight heightStateOutDOWNL1;
  climbStatesHeight heightState1;
  climbStatesHeight heightState2;

  climbStatesVelocities speedStateGetDown;
  climbStatesVelocities speedStateInL1;
  climbStatesVelocities speedStateOutL1;
  climbStatesVelocities speedStateInL2;
  climbStatesVelocities speedStateOutL2;
  climbStatesVelocities speedStateInL3;
  climbStatesVelocities speedStateOutL3;

  public ClimbSubsystem() {
    this.climbMotor = new TalonFX(ClimbSubsystemConstants.climbMotor1Port);
   
    this.limitSwitch = new DigitalInput(ClimbSubsystemConstants.limitSwitchPort);

    this.pidController = new ProfiledPIDController(ClimbSubsystemConstants.kp,
     ClimbSubsystemConstants.ki,
     ClimbSubsystemConstants.kd,
     new Constraints(ClimbSubsystemConstants.maxVelocity,
      ClimbSubsystemConstants.maxAcceleration));

     this.output = 0;
     this.currentHeight = 0;  

  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  public void setHeight(climbStatesHeight state){
    switch (state) {
      case POS1:
        pidController.setGoal(ClimbSubsystemConstants.POS1Height);
        break;
      case POS2:
        pidController.setGoal(ClimbSubsystemConstants.POS2Height);
        break;
      case L1OUTDOWN:
        pidController.setGoal(ClimbSubsystemConstants.L1OUTDOWN);  
      case L1OUTUP:
        pidController.setGoal(ClimbSubsystemConstants.L1OUTUP);  
      default:
        pidController.setGoal(ClimbSubsystemConstants.RESTHeight);
        break;
    }
  }

  public void setSpeed(climbStatesVelocities state){
    //TODO: Check which elevator goes clockwise and counter clockwise and change speeds accordingly
    switch (state) {
      case L1OUTUP:
        climbMotor.set(ClimbSubsystemConstants.L1Speed);
        break;
      case L1OUTDOWN:
        climbMotor.set(-ClimbSubsystemConstants.L1Speed);
        break;
      case L1IN:
        climbMotor.set(ClimbSubsystemConstants.L1Speed);
        break;
      case L2OUT:
        climbMotor.set(-ClimbSubsystemConstants.L2Speed);
        break;
      case L2IN:
        climbMotor.set(ClimbSubsystemConstants.L2Speed);
        break;
      case L3OUT:
        climbMotor.set(-ClimbSubsystemConstants.L3Speed);
        break;
       case L3IN:
        climbMotor.set(ClimbSubsystemConstants.L3Speed);
        break;
      default:
        stopMotor();
        break;
    }
  }

  private boolean isAtSetPoint(){
    return pidController.atGoal();
  }

  public void stopMotor(){
    climbMotor.set(0);
  }

  public double getHeight() {
    Angle angle = (this.climbMotor.getPosition().getValue());
    return ClimbSubsystemConstants.distancePerRotation.timesDivisor(angle).in(Centimeters);
  }

   public SequentialCommandGroup climbByHeight(climbStatesHeight heightState,climbStatesVelocities speedState){
    return new SequentialCommandGroup(
    new InstantCommand(() -> setHeight(heightState)),
    new InstantCommand(() -> setSpeed(speedState)),
    new WaitUntilCommand(() -> isAtSetPoint()),
    new InstantCommand(() -> stopMotor())
    );
  }

  public SequentialCommandGroup ClimbL1PeriodicCMD(){
    this.speedStateGetDown = climbStatesVelocities.L1OUTDOWN;
    this.heightStateOutDOWNL1 = climbStatesHeight.L1OUTDOWN;
    return new SequentialCommandGroup(
      climbByHeight(heightStateOutUPL1, speedStateOutL1),
      climbByHeight(heightState2,speedStateInL1),
      climbByHeight(heightStateOutDOWNL1,speedStateGetDown)
    );
  }

  public SequentialCommandGroup FullClimbCMD(){
    this.speedStateInL1 = climbStatesVelocities.L1IN;
    this.speedStateOutL1 = climbStatesVelocities.L1OUTUP;
    this.speedStateInL2 = climbStatesVelocities.L2IN;
    this.speedStateOutL2 = climbStatesVelocities.L2OUT;
    this.speedStateInL3 = climbStatesVelocities.L3IN;
    this.speedStateOutL3 = climbStatesVelocities.L3OUT;

    this.heightStateOutUPL1 = climbStatesHeight.L1OUTUP;
    this.heightState1 = climbStatesHeight.POS1;
    this.heightState2 = climbStatesHeight.POS2;

    return new SequentialCommandGroup(
    climbByHeight(heightStateOutUPL1,speedStateInL1),
    climbByHeight(heightState1,speedStateInL1),
    climbByHeight(heightState2,speedStateOutL2),
    climbByHeight(heightState1,speedStateInL2),
    climbByHeight(heightState2,speedStateOutL3),
    climbByHeight(heightState1,speedStateInL3)
    );
  }

  public boolean isConnected() {
   boolean limitSwitchLivness = DIOJNI.checkDIOChannel(ClimbSubsystemConstants.limitSwitchPort);

   boolean motorLiveness = this.climbMotor.isConnected() && this.climbMotor.isAlive(); 

   return limitSwitchLivness && motorLiveness;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLimitSwitch()) {
      stopMotor();
    }

    currentHeight = getHeight();

    output = pidController.calculate(currentHeight);
  }
}
