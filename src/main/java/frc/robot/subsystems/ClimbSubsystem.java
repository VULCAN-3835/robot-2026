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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
      case REST: 
        pidController.setGoal(ClimbSubsystemConstants.RESTHeight);
        break;
      case POS1:
        pidController.setGoal(ClimbSubsystemConstants.POS1Height);
        break;
      case POS2:
        pidController.setGoal(ClimbSubsystemConstants.POS2Height);
        break;  
      default:
        pidController.setGoal(ClimbSubsystemConstants.RESTHeight);
        break;
    }
  }

  public void setSpeed(climbStatesVelocities state){//TODO: Check which elevator goes clockwise and counter clockwise and change speeds accordingly
    switch (state) {
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
        climbMotor.set(0);
        break;
    }
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentHeight = getHeight();

    output = pidController.calculate(currentHeight);
  }
}
