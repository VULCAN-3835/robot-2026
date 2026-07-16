package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * Simulates everything above the drivetrain: intake pickup of fuel from the
 * field, flywheel spin-up physics, and shooting fuel as physical projectiles
 * that maple-sim's 2026 hub scores automatically.
 *
 * <p>Only ever constructed in simulation. It observes what the real subsystem
 * code commands its motors to do (via TalonFX sim states) and mirrors the
 * physical consequences in the sim world — the subsystem code itself is
 * untouched and runs identically to the real robot.
 */
public class SuperstructureSimulation {

    private final ChassisSubsystem chassis;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    private final IntakeSimulation intakeSim;

    private final FlywheelSim flywheelSim;
    private final TalonFXSimState flywheelMotorSim;
    private final TalonFXSimState rollerMotorSim;
    private final TalonFXSimState feedMotorSim;

    private final Timer timeSinceLastShot = new Timer();

    private final IntegerPublisher fuelCountPublisher = NetworkTableInstance.getDefault()
            .getIntegerTopic("FieldSimulation/FuelInRobot").publish();
    private final StructArrayPublisher<Pose3d> componentPosesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Chassis/ComponentPoses", Pose3d.struct).publish();

    public SuperstructureSimulation(ChassisSubsystem chassis, IntakeSubsystem intake,
            StorageSubsystem storage, ShooterSubsystem shooter) {
        this.chassis = chassis;
        this.intake = intake;
        this.shooter = shooter;

        // Fuel-collecting zone attached to the front of the simulated drivetrain;
        // fuel it touches while running is stored inside the robot
        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                chassis.getChassisSimulation().getDriveSim(),
                Meters.of(SimulationConstants.kIntakeWidthMeters),
                Meters.of(SimulationConstants.kIntakeExtensionMeters),
                IntakeSimulation.IntakeSide.FRONT,
                SimulationConstants.kIntakeCapacity);
        this.intakeSim.register();
        this.intakeSim.setGamePiecesCount(SimulationConstants.kPreloadedFuel);

        // Flywheel plant driven by the voltage the ShooterSubsystem applies
        this.flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getKrakenX60(3),
                        SimulationConstants.kFlywheelMOI,
                        1.0),
                DCMotor.getKrakenX60(3));

        this.flywheelMotorSim = shooter.getFlywheelMotor().getSimState();
        this.rollerMotorSim = intake.getRollerMotor().getSimState();
        this.feedMotorSim = storage.getFeedMotor().getSimState();

        this.timeSinceLastShot.start();
    }

    /** Must run every sim loop (called from Robot.simulationPeriodic). */
    public void update() {
        flywheelMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        rollerMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        feedMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        updateFlywheel();
        updateIntake();
        updateShooting();

        fuelCountPublisher.set(intakeSim.getGamePiecesAmount());
        publishComponentPoses();
    }

    private void updateFlywheel() {
        flywheelSim.setInputVoltage(flywheelMotorSim.getMotorVoltage());
        flywheelSim.update(0.02);

        // Report the spin back so the shooter's velocity signal works in sim
        double rotorRPS = flywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
        flywheelMotorSim.setRotorVelocity(rotorRPS);
    }

    private void updateIntake() {
        boolean intaking = intake.getTargetState() == intakeStates.INTAKE
                && rollerMotorSim.getMotorVoltage() > SimulationConstants.kIntakeMinRollerVoltage;

        if (intaking) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }
    }

    private void updateShooting() {
        double flywheelRPS = flywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI);

        boolean shooting = feedMotorSim.getMotorVoltage() > SimulationConstants.kMinFeedVoltage
                && flywheelRPS > SimulationConstants.kMinShootFlywheelRPS
                && intakeSim.getGamePiecesAmount() > 0
                && timeSinceLastShot.hasElapsed(SimulationConstants.kSecondsBetweenShots);

        if (!shooting || !intakeSim.obtainGamePieceFromIntake()) {
            return;
        }
        timeSinceLastShot.restart();

        // Fuel exit speed scales with flywheel surface speed; pitch is a fixed
        // placeholder because the hood isn't physically simulated
        double launchSpeed = flywheelRPS * 2 * Math.PI
                * SimulationConstants.kFlywheelRadiusMeters
                * SimulationConstants.kFuelLaunchSpeedFactor;

        Pose2d robotPose = chassis.getChassisSimulation().getActualPose();
        SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                new Translation2d(), // shooter at robot center
                chassis.getChassisSimulation().getDriveSim().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                robotPose.getRotation(),
                Meters.of(SimulationConstants.kShooterHeightMeters),
                MetersPerSecond.of(launchSpeed),
                Degrees.of(SimulationConstants.kShooterPitchDegrees)));
    }

    /**
     * Publishes mechanism poses (robot-relative) so AdvantageScope can animate
     * articulated components once the team's CAD model is imported.
     * Component 0: intake arm. Component 1: shooter hood.
     * The pivot locations/signs are placeholders — calibrate against the CAD.
     */
    private void publishComponentPoses() {
        double armPitchRad = Math.toRadians(IntakeConstants.restPoint - intake.getArmTargetDegrees());
        Pose3d armPose = new Pose3d(
                SimulationConstants.kArmPivotXMeters, 0, SimulationConstants.kArmPivotZMeters,
                new Rotation3d(0, armPitchRad, 0));

        double hoodPitchRad = Math.toRadians(shooter.getHoodAngleDegs() - 20);
        Pose3d hoodPose = new Pose3d(0, 0, SimulationConstants.kShooterHeightMeters,
                new Rotation3d(0, hoodPitchRad, 0));

        componentPosesPublisher.set(new Pose3d[] { armPose, hoodPose });
    }

    /** Fuel currently held by the simulated robot. */
    public int getFuelCount() {
        return intakeSim.getGamePiecesAmount();
    }
}
