# Coding Guidelines

## FRC Robot Code Standards

### File Organization

- **Constants.java**: All robot-wide constants (ports, tuning values, etc.)
- **RobotContainer.java**: Subsystem instantiation and button bindings
- **Robot.java**: Lifecycle methods (robotInit, autonomousInit, teleopInit, etc.)
- **Subsystems/**: One class per hardware subsystem
- **Commands/**: Command classes organized by function

### Naming Conventions

| Type | Convention | Example |
|------|-----------|---------|
| Classes | PascalCase | `DriveSubsystem`, `AutoBalanceCommand` |
| Methods | camelCase | `driveForward()`, `getEncoderPosition()` |
| Variables | camelCase | `m_driveMotor`, `kMaxSpeed` |
| Constants | kPascalCase or UPPER_SNAKE | `kDrivePort`, `MAX_VELOCITY` |
| Private members | m prefix | `m_driveSubsystem` |

### Subsystem Guidelines

1. **Single Responsibility**: Each subsystem manages one hardware component or related group
2. **Hardware Access**: All hardware access goes through the subsystem
3. **State Management**: Subsystems track their own state
4. **Safety**: Always include safety checks (limit switches, current limits)

Example:
```java
public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(Constants.DriveConstants.kLeftMotorPort);
    private final WPI_TalonSRX m_rightMotor = new WPI_TalonSRX(Constants.DriveConstants.kRightMotorPort);
    
    public DriveSubsystem() {
        // Configure motors
        m_leftMotor.setInverted(true);
    }
    
    public void arcadeDrive(double speed, double rotation) {
        m_leftMotor.set(speed + rotation);
        m_rightMotor.set(speed - rotation);
    }
    
    @Override
    public void periodic() {
        // Update dashboard, sensors, etc.
    }
}
```

### Command Guidelines

1. **Explicit Requirements**: Always use `addRequirements()` in constructor
2. **Clean Completion**: Ensure `isFinished()` is correct to avoid runaway commands
3. **Proper Cleanup**: Use `end()` or `end(boolean interrupted)` for cleanup
4. **Statelessness**: Commands should generally not maintain state across runs

Example:
```java
public class DriveDistanceCommand extends Command {
    private final DriveSubsystem m_drive;
    private final double m_distance;
    private double m_startPosition;
    
    public DriveDistanceCommand(DriveSubsystem drive, double distance) {
        m_drive = drive;
        m_distance = distance;
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        m_startPosition = m_drive.getPosition();
    }
    
    @Override
    public void execute() {
        m_drive.arcadeDrive(0.5, 0);
    }
    
    @Override
    public boolean isFinished() {
        return m_drive.getPosition() - m_startPosition >= m_distance;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0); // Stop motors
    }
}
```

### Constants Organization

```java
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotorPort = 1;
        public static final int kRightMotorPort = 2;
        public static final double kMaxSpeed = 1.0;
    }
    
    public static final class ArmConstants {
        public static final int kMotorPort = 3;
        public static final double kP = 0.1;
        public static final double kMaxAngle = 90.0;
    }
    
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
}
```

### Controller Bindings

Keep bindings organized and documented:

```java
private void configureBindings() {
    // Driver controls
    m_driverController.a().onTrue(new DriveToTargetCommand(m_drive, m_vision));
    m_driverController.b().whileTrue(new AutoBalanceCommand(m_drive));
    
    // Operator controls
    m_operatorController.leftBumper().whileTrue(new IntakeCommand(m_intake, 0.5));
    m_operatorController.rightBumper().whileTrue(new OuttakeCommand(m_intake, -0.5));
}
```

### Documentation Requirements

- **Class-level**: Explain what the subsystem/command does
- **Public methods**: Explain parameters, return values, and behavior
- **Complex logic**: Inline comments for non-obvious code
- **Hardware configuration**: Document port numbers and wiring

### Testing Requirements

All code should be testable:

1. **Unit tests**: Test logic in isolation
2. **Simulation**: Verify behavior in WPILib simulation
3. **Hardware test commands**: Create commands to test hardware individually

### Performance Considerations

- **20ms loop**: All periodic methods run every 20ms
- **Avoid heavy computation**: In periodic methods, keep it fast
- **NetworkTables**: Minimize NT updates to reduce bandwidth
- **Memory**: Avoid creating objects in periodic methods

### Git Best Practices

- Commit often with descriptive messages
- Test before committing
- Don't commit binary build artifacts (use .gitignore)
- Keep commits focused on single features/fixes

## Common Pitfalls to Avoid

1. **Forgetting addRequirements()** - Commands won't work properly
2. **Not handling end() properly** - Motors keep running after command ends
3. **Hardcoding values** - Makes tuning difficult
4. **Blocking calls** - Never use Thread.sleep() or blocking I/O in robot code
5. **Null pointers** - Always initialize objects in constructors
6. **Resource conflicts** - Two commands requiring same subsystem can't run together
