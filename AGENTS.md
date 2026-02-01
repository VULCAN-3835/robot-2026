# AGENTS.md

Welcome! This is an FRC (FIRST Robotics Competition) robot project for the 2026 season.

## Project Overview

- **Language**: Java 17
- **Framework**: WPILib 2026.2.1 with Command-based programming
- **Build Tool**: Gradle
- **Robot Hardware**: FRC RoboRIO

## Quick Start for Agents

### Building the Project

```bash
./gradlew build
```

### Running Tests

```bash
./gradlew test
```

### Deploying to Robot (requires robot connection)

```bash
./gradlew deploy
```

### Running Simulation

```bash
./gradlew simulateJava
```

## Project Structure

```
src/main/java/frc/robot/
├── Main.java              # Entry point (don't modify)
├── Robot.java             # Main robot class with lifecycle methods
├── RobotContainer.java    # Configure subsystems, commands, and button bindings
├── Constants.java         # Robot-wide constants
├── commands/              # Command classes
│   ├── ExampleCommand.java
│   └── Autos.java
└── subsystems/            # Subsystem classes
    └── ExampleSubsystem.java
```

## Key Concepts

### Command-Based Programming

This project uses the WPILib Command-based framework:

- **Subsystems**: Represent robot hardware components (motors, sensors, etc.)
- **Commands**: Actions that run on subsystems
- **Triggers**: Bind inputs (controller buttons) to commands

### Common Patterns

1. **Creating a new subsystem**:
   - Extend `SubsystemBase`
   - Define hardware (motors, sensors) in constructor
   - Create methods for subsystem actions
   - Add periodic logic if needed

2. **Creating a new command**:
   - Extend `Command` or use inline commands
   - Declare required subsystems
   - Implement `initialize()`, `execute()`, `isFinished()`, `end()`

3. **Binding controls**:
   - Edit `RobotContainer.configureBindings()`
   - Use `Trigger` or controller button methods
   - Example: `m_driverController.a().onTrue(new MyCommand())`

## Important Notes

- **Don't modify** `Main.java` or core WPILib classes
- **Constants** should be defined in `Constants.java`
- All subsystems must be registered with the CommandScheduler
- Always test in simulation before deploying to the robot
- Follow Java naming conventions (camelCase for methods/variables, PascalCase for classes)

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [Command-based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [2026 WPILib Release Notes](https://github.com/wpilibsuite/allwpilib/releases)

## Testing Checklist

Before submitting changes:

- [ ] Code builds without errors (`./gradlew build`)
- [ ] Tests pass (`./gradlew test`)
- [ ] Code runs in simulation (`./gradlew simulateJava`)
- [ ] No hardcoded values (use Constants.java)
- [ ] Proper subsystem/command structure followed
- [ ] Comments added for complex logic

## Contact

For questions about the robot design or competition specifics, consult the team lead or documentation.
