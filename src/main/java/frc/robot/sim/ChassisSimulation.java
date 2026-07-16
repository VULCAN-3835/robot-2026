package frc.robot.sim;

// TalonFX/CANcoder sim-state bridge pattern adapted from the official maple-sim CTRE template:
// https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim (MIT license)

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Util.SwerveModule;
import frc.robot.subsystems.ChassisSubsystem.Wheels;

/**
 * Bridges the real robot code to the maple-sim physics engine.
 *
 * <p>Only ever constructed in simulation. Each loop, maple-sim reads the voltages the
 * TalonFXs are applying (via their sim states), computes the physical response of the
 * drivetrain (with mass, wheel grip and collisions), and writes the resulting encoder
 * readings back into the TalonFX/CANcoder sim states. The NavX yaw is injected through
 * its WPILib sim device, so the exact same subsystem code runs in sim and on the robot.
 */
public class ChassisSimulation {

    private final SwerveDriveSimulation driveSim;
    private final SimDouble gyroYawSim;

    /**
     * @param modules the real SwerveModule objects, indexed by {@link Wheels} ordinal
     * @param startingPose the pose the simulated robot spawns at
     */
    public ChassisSimulation(SwerveModule[] modules, Pose2d startingPose) {
        DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(ChassisConstants.kMassKG))
                .withGyro(COTS.ofNav2X())
                // maple-sim module order follows these translations (FL, FR, BL, BR)
                .withCustomModuleTranslations(ChassisConstants.kDriveKinematics.getModules())
                .withBumperSize(
                        Meters.of(SimulationConstants.kBumperLengthXMeters),
                        Meters.of(SimulationConstants.kBumperWidthYMeters))
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getFalcon500(1),
                        ModuleConstants.kDriveMotorGearRatio,
                        ModuleConstants.kSteerMotorGearRatio,
                        Volts.of(SimulationConstants.kDriveFrictionVoltage),
                        Volts.of(SimulationConstants.kSteerFrictionVoltage),
                        Meters.of(ModuleConstants.kWheelDiameterMeters / 2.0),
                        KilogramSquareMeters.of(SimulationConstants.kSteerRotationalInertia),
                        SimulationConstants.kWheelCOF));

        this.driveSim = new SwerveDriveSimulation(config, startingPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        // Wire each maple-sim module (kinematics order: FL, FR, BL, BR) to the physical
        // module at that corner (the Wheels enum uses a different ordering)
        SwerveModuleSimulation[] moduleSims = driveSim.getModules();
        hookModule(moduleSims[0], modules[Wheels.LEFT_FRONT.ordinal()]);
        hookModule(moduleSims[1], modules[Wheels.RIGHT_FRONT.ordinal()]);
        hookModule(moduleSims[2], modules[Wheels.LEFT_BACK.ordinal()]);
        hookModule(moduleSims[3], modules[Wheels.RIGHT_BACK.ordinal()]);

        // The Studica AHRS registers a WPILib sim device; MXP SPI shows up as
        // "navX-Sensor[4]". Writing "Yaw" makes imu.getAngle()/getYaw() work in sim.
        this.gyroYawSim = new SimDeviceSim("navX-Sensor", 4).getDouble("Yaw");
    }

    private void hookModule(SwerveModuleSimulation sim, SwerveModule module) {
        sim.useDriveMotorController(new TalonFXMotorControllerSim(module.getDriveMotor()));
        sim.useSteerMotorController(new TalonFXMotorControllerWithRemoteCanCoderSim(
                module.getSteerMotor(), module.getAbsEncoder()));
    }

    /**
     * Injects the maple-sim gyro reading into the NavX and syncs the battery voltage.
     * Must run every sim loop, after {@code SimulatedArena.simulationPeriodic()}.
     */
    public void update() {
        // The chassis code treats imu.getAngle() as CCW-positive (matches the physical
        // mounting on the real robot), so the CCW-positive sim heading is fed directly.
        // If the robot spins the wrong way in sim vs. the sticks, negate this value.
        gyroYawSim.set(driveSim.getGyroSimulation().getGyroReading().getDegrees());

        // Makes RobotController.getBatteryVoltage() reflect simulated battery sag
        RoboRioSim.setVInVoltage(SimulatedBattery.getBatteryVoltage().in(Volts));
    }

    /** The ground-truth pose of the robot in the physics world (not odometry). */
    public Pose2d getActualPose() {
        return driveSim.getSimulatedDriveTrainPose();
    }

    /** Direct access to the underlying maple-sim drivetrain (diagnostics/tests). */
    public SwerveDriveSimulation getDriveSim() {
        return driveSim;
    }

    /** Teleports the simulated robot, e.g. when odometry is reset at auto start. */
    public void setPose(Pose2d pose) {
        driveSim.setSimulationWorldPose(pose);
    }

    /**
     * Reads the voltage a TalonFX is applying and feeds the simulated rotor
     * position/velocity back into it.
     */
    private static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private final TalonFXSimState talonFXSimState;

        TalonFXMotorControllerSim(TalonFX talonFX) {
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    /** Same as above, but also feeds the mechanism angle into a remote CANcoder (steer). */
    private static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
        private final CANcoderSimState remoteCancoderSimState;

        TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);
            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }
}
