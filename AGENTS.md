# AGENTS.md

Welcome! This is an FRC (FIRST Robotics Competition) robot project for the 2026 season.

## Agent Guidelines

When working on this codebase, follow these principles:

### 1. Simplest Solution First
- Always consider the simplest approach before complex ones
- Avoid over-engineering and premature optimization
- If a simple if-statement works, don't build a state machine
- Prefer existing patterns over new abstractions

### 2. Comment Your Code
- Add JavaDoc to all public methods explaining purpose, parameters, and return values
- Comment complex algorithms or non-obvious logic
- Explain "why" not "what" (the code shows what)
- Keep comments up-to-date when modifying code

### 3. Update Documentation
- Update README.md if you add/remove subsystems or major features
- Update this AGENTS.md if you change project structure or conventions
- Document new constants or configuration options
- If you change how something works, update the relevant docs immediately
- Update docs/ directory for complex features (see `docs/shoot-on-the-move.md` as example)

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
.
├── src/main/java/frc/robot/
│   ├── Main.java              # Entry point (don't modify)
│   ├── Robot.java             # Main robot class with lifecycle methods
│   ├── RobotContainer.java    # Configure subsystems, commands, and button bindings
│   ├── Constants.java         # Robot-wide constants
│   ├── commands/              # Command classes
│   │   ├── ShootCMD.java
│   │   └── Autos.java
│   ├── subsystems/            # Subsystem classes
│   │   ├── ChassisSubsystem.java
│   │   ├── ShooterSubsystem.java
│   │   ├── IntakeSubsystem.java
│   │   └── StorageSubsystem.java
│   └── Util/                  # Utility classes
├── docs/                      # Feature documentation
│   └── shoot-on-the-move.md   # Complex feature docs
├── README.md                  # Project overview
└── AGENTS.md                  # This file
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
- [ ] Documentation updated (README.md, AGENTS.md if applicable)

## Contact

For questions about the robot design or competition specifics, consult the team lead or documentation.
