package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;

/**
 * Rotates the chassis to continuously face the hub while allowing normal X/Y translation.
 * Bound to leftBumper (whileTrue) for "big dumper" mode.
 */
public class TurnToHubCMD extends Command {

    private static final double kP = 0.05;

    private final ChassisSubsystem chassisSubsystem;
    private final Supplier<Double> xVelocitySupplier, yVelocitySupplier;
    private final SlewRateLimiter xLimiter, yLimiter;

    public TurnToHubCMD(ChassisSubsystem chassisSubsystem,
                        Supplier<Double> xVelocitySupplier,
                        Supplier<Double> yVelocitySupplier) {
        this.chassisSubsystem = chassisSubsystem;
        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.xLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        this.yLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSec);
        addRequirements(chassisSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xVelocity = xVelocitySupplier.get();
        double yVelocity = yVelocitySupplier.get();

        xVelocity = Math.abs(xVelocity) > Constants.OperatorConstants.kDeadband ? xVelocity : 0;
        yVelocity = Math.abs(yVelocity) > Constants.OperatorConstants.kDeadband ? yVelocity : 0;

        xVelocity = xLimiter.calculate(xVelocity) * ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;
        yVelocity = yLimiter.calculate(yVelocity) * ChassisConstants.kTeleDriveMaxSpeedMetersPerSec;

        // Angle from robot to hub, updated every loop so tracking is continuous
        Translation2d hubPos = ChassisConstants.getHubTopCenter().toTranslation2d();
        Translation2d robotPos = chassisSubsystem.getPose().getTranslation();
        double targetYaw = hubPos.minus(robotPos).getAngle().getDegrees();

        double error = MathUtil.inputModulus(targetYaw - chassisSubsystem.getYaw(), -180, 180);
        double rotOutput = MathUtil.clamp(
                kP * error,
                -ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec,
                ChassisConstants.kTeleDriveMaxAngulerSpeedRadiansPerSec);

        SmartDashboard.putNumber("TurnToHub/targetYaw", targetYaw);
        SmartDashboard.putNumber("TurnToHub/yawError", error);

        chassisSubsystem.drive(xVelocity, yVelocity, rotOutput, true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
