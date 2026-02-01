// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants.climbStates;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */

    TalonFX climbMotor1;//master
    TalonFX climbMotor2;//slave

    DigitalInput limitSwitch;

    ProfiledPIDController pidController;

  public ClimbSubsystem() {
    this.climbMotor1 = new TalonFX(ClimbSubsystemConstants.climbMotor1Port);
    this.climbMotor2 = new TalonFX(ClimbSubsystemConstants.climbMotor2Port);
    this.climbMotor1.setControl(new Follower(ClimbSubsystemConstants.climbMotor2Port,null));
    this.limitSwitch = new DigitalInput(ClimbSubsystemConstants.limitSwitchPort);

    this.pidController = new ProfiledPIDController(ClimbSubsystemConstants.kp,
     ClimbSubsystemConstants.ki,
     ClimbSubsystemConstants.kd,
     null);


  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  public void setHeight(climbStates state){
    switch (state) {
      case REST:
        pidController.setGoal(ClimbSubsystemConstants.restHeight);
        break;
      case L1:
        pidController.setGoal(ClimbSubsystemConstants.L1Height);
        break;
      case L2:
        pidController.setGoal(ClimbSubsystemConstants.L2Height);
        break;
      case L3:
        pidController.setGoal(ClimbSubsystemConstants.L3Height);
        break;
      default:
        pidController.setGoal(ClimbSubsystemConstants.restHeight);
        break;
    }
  }

  public boolean isAtSetPoint() {
    return pidController.atGoal();
  }


  public boolean isConnected() {
   // boolean limitSwitchLivness = DIOJNI.checkDIOChannel(ClimbSubsystemConstants.limitSwitchPort);

    boolean motor1Liveness = this.climbMotor1.isConnected() && this.climbMotor1.isAlive(); 
    boolean motor2Liveness = this.climbMotor2.isConnected() && this.climbMotor2.isAlive();

    return /*limitSwitchLivness &&*/ motor1Liveness && motor2Liveness;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
