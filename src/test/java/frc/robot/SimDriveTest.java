package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.IntakeConstants.intakeStates;
import frc.robot.sim.SuperstructureSimulation;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

/**
 * Smoke test for the maple-sim integration: commands the drivetrain forward and
 * verifies the full chain (velocity control -> TalonFX sim state -> maple-sim
 * physics -> simulated encoders -> odometry) produces movement.
 */
public class SimDriveTest {

    @Test
    public void robotDrivesForwardInSimulation() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        ChassisSubsystem chassis = new ChassisSubsystem();

        // Let Phoenix device configs apply and the enable state propagate
        Thread.sleep(500);
        DriverStationSim.notifyNewData();

        Pose2d start = chassis.getPose();

        // ~3 seconds of robot loops at 2 m/s, robot-relative straight ahead
        for (int i = 0; i < 150; i++) {
            chassis.drive(2.0, 0, 0, false);
            chassis.periodic();
            chassis.simulationPeriodic();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20); // Phoenix sim signals update in real time

            if (i % 30 == 0) {
                StringBuilder sb = new StringBuilder("[SimDriveTest] i=" + i);
                for (var pos : chassis.getModPositions()) {
                    sb.append(String.format(" | ang=%.1fdeg d=%.2fm",
                            pos.angle.getDegrees(), pos.distanceMeters));
                }
                System.out.println(sb);

                StringBuilder sb2 = new StringBuilder("[SimDriveTest] maple i=" + i);
                for (var modSim : chassis.getChassisSimulation().getDriveSim().getModules()) {
                    var st = modSim.getCurrentState();
                    sb2.append(String.format(" | v=%.2fm/s ang=%.1fdeg U=%.2fV",
                            st.speedMetersPerSecond, st.angle.getDegrees(),
                            modSim.getDriveMotorAppliedVoltage().in(edu.wpi.first.units.Units.Volts)));
                }
                System.out.println(sb2);
            }
        }

        Pose2d end = chassis.getPose();
        double distance = end.getTranslation().getDistance(start.getTranslation());
        System.out.println("[SimDriveTest] start=" + start + " end=" + end
                + " distance=" + distance + " m, gyro yaw=" + chassis.getYaw());

        assertTrue(distance > 0.5,
                "Expected the simulated robot to drive at least 0.5 m, moved " + distance + " m");

        // ── Rotation phase: +PI rad/s (CCW) for ~1.5 s ──────────────────────────
        for (int i = 0; i < 75; i++) {
            chassis.drive(0, 0, Math.PI, false);
            chassis.periodic();
            chassis.simulationPeriodic();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20);
        }

        Pose2d odo = chassis.getPose();
        Pose2d truth = chassis.getChassisSimulation().getActualPose();
        System.out.println("[SimDriveTest] after spin: odometry=" + odo + " truth=" + truth
                + " gyro yaw=" + chassis.getYaw());

        // Positive commanded omega must read as CCW-positive on the gyro (sign convention)
        assertTrue(chassis.getYaw() > 30,
                "Expected CCW-positive yaw after +omega command, got " + chassis.getYaw());
        // Odometry should stay near ground truth. It can't be exact: maple-sim models
        // wheel scrub during the spin-up transient, and skid is invisible to wheel
        // odometry (on the real robot, vision corrects this)
        double odoError = odo.getTranslation().getDistance(truth.getTranslation());
        assertTrue(odoError < 0.8,
                "Odometry drifted " + odoError + " m from ground truth during spin");

        // ── Combined phase: translate + rotate for ~2.5 s ──────────────────────
        // This is where a module-order mismatch in the pose estimator shows up:
        // wheel angles differ per corner, so swapped rear modules corrupt the twist
        Pose2d odoBefore = chassis.getPose();
        Pose2d truthBefore = chassis.getChassisSimulation().getActualPose();
        double gapBefore = odoBefore.getTranslation().getDistance(truthBefore.getTranslation());

        for (int i = 0; i < 125; i++) {
            chassis.drive(1.5, 0, Math.PI / 2, false);
            chassis.periodic();
            chassis.simulationPeriodic();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20);
        }

        Pose2d odoAfter = chassis.getPose();
        Pose2d truthAfter = chassis.getChassisSimulation().getActualPose();
        double gapAfter = odoAfter.getTranslation().getDistance(truthAfter.getTranslation());
        System.out.println(String.format(
                "[SimDriveTest] combined drive+spin: odo-vs-truth gap %.3f m -> %.3f m (grew %.3f m)",
                gapBefore, gapAfter, gapAfter - gapBefore));
        System.out.println("[SimDriveTest] combined phase: odometry=" + odoAfter + " truth=" + truthAfter);

        // ── Intake phase: drive over a fuel with the intake running ────────────
        IntakeSubsystem intake = new IntakeSubsystem();
        StorageSubsystem storage = new StorageSubsystem();
        ShooterSubsystem shooter = new ShooterSubsystem(chassis);
        SuperstructureSimulation superSim = new SuperstructureSimulation(chassis, intake, storage, shooter);

        // Teleport to open carpet (mid-field, away from the trench structures),
        // then spawn a fuel 1.2 m ahead
        chassis.resetOdometry(new Pose2d(8.2, 4.1, new edu.wpi.first.math.geometry.Rotation2d()));
        Translation2d fuelSpot = new Translation2d(9.4, 4.1);
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(fuelSpot));

        int fuelBefore = superSim.getFuelCount();
        intake.setArmState(intakeStates.INTAKE);
        intake.setRollerVoltage(3);

        for (int i = 0; i < 75; i++) {
            chassis.drive(1.0, 0, 0, false);
            chassis.periodic();
            chassis.simulationPeriodic();
            superSim.update();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20);

            if (i % 25 == 0) {
                System.out.println(String.format(
                        "[SimDriveTest] intake dbg i=%d truth=%s rollerV=%.2f count=%d",
                        i, chassis.getChassisSimulation().getActualPose().getTranslation(),
                        intake.getRollerMotor().getSimState().getMotorVoltage(),
                        superSim.getFuelCount()));
            }
        }

        System.out.println("[SimDriveTest] intake: fuel count " + fuelBefore + " -> " + superSim.getFuelCount());
        // May collect more than the spawned one — the arena pre-places field fuel
        assertTrue(superSim.getFuelCount() > fuelBefore,
                "Expected to pick up fuel, count went " + fuelBefore + " -> " + superSim.getFuelCount());

        // ── Shooting phase: spin up flywheel, then run the feed ────────────────
        chassis.drive(0, 0, 0, false);
        intake.setRollerVoltage(0);
        shooter.setFlywheelVoltage(5);

        for (int i = 0; i < 60; i++) { // spin-up
            chassis.periodic();
            chassis.simulationPeriodic();
            superSim.update();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20);
        }
        double flywheelRPS = shooter.getFlywheelMotor().getVelocity().getValueAsDouble();
        System.out.println("[SimDriveTest] flywheel at 5V spun up to " + flywheelRPS + " RPS");
        assertTrue(flywheelRPS > 10, "Flywheel did not spin up in sim, RPS=" + flywheelRPS);

        int fuelLoaded = superSim.getFuelCount();
        storage.setFeedMotorPower(7);
        for (int i = 0; i < 60; i++) {
            chassis.periodic();
            chassis.simulationPeriodic();
            superSim.update();
            SimulatedArena.getInstance().simulationPeriodic();
            DriverStationSim.notifyNewData();
            Thread.sleep(20);
        }

        System.out.println("[SimDriveTest] shooting: fuel count " + fuelLoaded + " -> " + superSim.getFuelCount());
        assertTrue(superSim.getFuelCount() < fuelLoaded,
                "Expected fuel to be launched, count stayed at " + superSim.getFuelCount());

        HAL.shutdown();
    }
}
