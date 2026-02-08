// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX turretMotor;
  private TalonFX hoodMotor;
  private TalonFX flyWheelMotor;

  private CANcoder hoodCancoder;
  private CANcoder turretCancoder;
  public ShooterSubsystem() {
    this.turretMotor = new TalonFX(0);
    this.flyWheelMotor = new TalonFX(0);
    this.hoodMotor = new TalonFX(0);

    this.hoodCancoder = new CANcoder(0);
    this.turretCancoder = new CANcoder(0);
  }
  public Angle getHoodAngle(){
    return this.hoodCancoder.getAbsolutePosition().getValue();
  }
  public Angle getTurrentAngle(){
    return this.turretCancoder.getAbsolutePosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
