// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Centimeters;

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

  climbStatesHeight heightState;
  climbStatesHeight getDownHeightState;
  climbStatesHeight heightState2;
  climbStatesHeight heightState3;

  climbStatesVelocities speedState;
  climbStatesVelocities getDownSpeedState;
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
      case L1GETDOWN:
        pidController.setGoal(ClimbSubsystemConstants.L1GETDOWN);  
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
        climbMotor.set(ClimbSubsystemConstants.L1Speed);
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

  public void climbBySpeed(climbStatesHeight heightState,climbStatesVelocities speedState){
    setHeight(heightState);
    setSpeed(speedState);
  }

  public boolean isAtSetPoint(){
    return pidController.atGoal();
  }

  public void stopMotor(){
    climbMotor.set(0);
  }

  public double getHeight() {
    Angle angle = (this.climbMotor.getPosition().getValue());
    return ClimbSubsystemConstants.distancePerRotation.timesDivisor(angle).in(Centimeters);
  }

  public boolean isConnected() {
   boolean limitSwitchLivness = DIOJNI.checkDIOChannel(ClimbSubsystemConstants.limitSwitchPort);

   boolean motorLiveness = this.climbMotor.isConnected() && this.climbMotor.isAlive(); 

   return limitSwitchLivness && motorLiveness;
  }

  public SequentialCommandGroup GetDownFromL1(){
    this.getDownHeightState = climbStatesHeight.L1GETDOWN;
    this.getDownSpeedState = climbStatesVelocities.L1OUTDOWN;

    return new SequentialCommandGroup(
      //1. set height to the PID
      new InstantCommand(() -> setHeight(getDownHeightState)),
      //2.Move the outer elevator a little up then we fall
      new InstantCommand(() -> setSpeed(getDownSpeedState)),
      //3. check if we reached the goal
      new WaitUntilCommand(() -> isAtSetPoint()),
      //4. stop the motor
      new InstantCommand(() -> stopMotor())
    );
  }

  public SequentialCommandGroup ClimbToL1PeriodicCMD(){
    this.heightState = climbStatesHeight.POS1;
    this.speedState = climbStatesVelocities.L1OUTUP;
    return new SequentialCommandGroup(
      //1. set height to the PID and climb to L1
      new InstantCommand(() -> climbBySpeed(getDownHeightState, getDownSpeedState)),
      //2. check if we reached the goal
      new WaitUntilCommand(() -> isAtSetPoint()),
      //3. stop the motor
      new InstantCommand(() -> stopMotor())
      );
  }
   public SequentialCommandGroup ClimbToL1EndGameCMD(){
    this.heightState = climbStatesHeight.POS1;
    this.speedState = climbStatesVelocities.L1IN;
    return new SequentialCommandGroup(
      //1. set height to the PID and climb to L1
      new InstantCommand(() -> climbBySpeed(getDownHeightState, getDownSpeedState)),
      //2. check if we reached the goal
      new WaitUntilCommand(() -> isAtSetPoint()),
      //3. stop the motor
      new InstantCommand(() -> stopMotor())
      );
  }

   public SequentialCommandGroup FullClimbCMD(){
    this.heightState2 = climbStatesHeight.POS2;
    this.heightState3 = climbStatesHeight.POS1;
    this.speedStateOutL2 = climbStatesVelocities.L2OUT;
    this.speedStateInL2 = climbStatesVelocities.L2IN;
    this.speedStateOutL3 = climbStatesVelocities.L3OUT;
    this.speedStateInL3 = climbStatesVelocities.L3IN;

    return new SequentialCommandGroup(
      //1. climbs to L1
      ClimbToL1EndGameCMD(),
      //2. set the goal height to L2 for outer elevator and sets the speed
      new InstantCommand(() ->climbBySpeed(heightState2,speedStateOutL2)),
      //3.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> isAtSetPoint()),
      //4.stops the motor 
      new InstantCommand(() -> stopMotor()),
      //5. set the goal height to L2 for inner elevator and sets the speed
      new InstantCommand(() -> climbBySpeed(heightState3,speedStateInL2)),
      //6.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> isAtSetPoint()),
      //7.stops the motor
      new InstantCommand(() -> stopMotor()),
      //8. set the goal height to L3 for outer elevator and sets the speed
      new InstantCommand(() -> climbBySpeed(heightState2,speedStateOutL3)),
      //9.checks if the elevator is at the setpoint
      new WaitUntilCommand(() -> isAtSetPoint()),
      //10.stops the motor
      new InstantCommand(() -> stopMotor())
    );
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
