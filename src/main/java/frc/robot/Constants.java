// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Rotation;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Per;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ClimbSubsystemConstants {
    public static enum climbStatesVelocities{L1OUTDOWN,L1OUTUP,L1IN,L2OUT,L2IN,L3OUT,L3IN}
    public static enum climbStatesHeight{REST,POS1,POS2,L1OUTDOWN,L1OUTUP}

    public static final int climbMotor1Port = 0;
    public static final int limitSwitchPort = 0;
    
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;

    public static final double RESTHeight = 0;
    public static final double POS1Height = 0;
    public static final double POS2Height = 0;
    public static final double L1OUTDOWN = 0;
    public static final double L1OUTUP = 0;

    public static final double L1Speed = 0;
    public static final double L2Speed = 0;
    public static final double L3Speed = 0;

    public static Per<DistanceUnit, AngleUnit> distancePerRotation = Centimeters.of(0).div(Rotation.of(0));


  }
}
