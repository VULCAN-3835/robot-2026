# Instructions for OpenCode

## Project Context

This is a **FIRST Robotics Competition (FRC) 2026 robot project** written in Java using WPILib. The robot runs on a RoboRIO controller and uses the Command-based programming framework.

## When Working on This Project

### Always Do

1. **Check Constants First**: Always look at `Constants.java` before hardcoding any values. Add new constants there if needed.

2. **Follow WPILib Patterns**: 
   - Subsystems extend `SubsystemBase`
   - Commands follow the Command interface
   - Use the CommandScheduler properly

3. **Test Your Changes**:
   - Run `./gradlew build` to ensure code compiles
   - Run `./gradlew test` for unit tests
   - Use `./gradlew simulateJava` to test logic

4. **Keep It Simple**: FRC robots have limited compute power. Avoid unnecessary complexity.

5. **Document Hardware**: When adding motors, sensors, or mechanisms, add comments explaining:
   - What the hardware is
   - Which port/ID it's on
   - What it does

6. **Update Documentation**: After making significant changes, update relevant `.md` files:
   - If you change project structure, update `AGENTS.md`
   - If you add new patterns or workflows, update `.opencode/instructions.md`
   - If you change coding standards, update `.opencode/guidelines.md`
   - Keep documentation in sync with code changes

### Never Do

1. **Don't modify** `Main.java` - it's the entry point and shouldn't change
2. **Don't hardcode** team numbers, port numbers, or tuning values
3. **Don't create memory leaks** - commands and subsystems must be managed properly
4. **Don't ignore simulation** - test in simulation before deploying

### Code Style

- Use meaningful variable names
- Add Javadoc comments to public methods
- Follow Java conventions (camelCase, PascalCase appropriately)
- Keep methods focused and single-purpose

### Common Tasks

**Adding a motor:**
```java
// In Constants.java
public static final int kMotorPort = 1;

// In Subsystem
private final PWMVictorSPX m_motor = new PWMVictorSPX(Constants.kMotorPort);
```

**Creating a command:**
```java
public class DriveCommand extends Command {
    private final DriveSubsystem m_drive;
    
    public DriveCommand(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive); // Critical!
    }
    
    @Override
    public void execute() {
        // Command logic here
    }
}
```

**Binding a button:**
```java
// In RobotContainer.configureBindings()
m_driverController.a().onTrue(new MyCommand(m_subsystem));
```

## Troubleshooting

**Build fails?**
- Check Java 17 is installed
- Run `./gradlew clean` then rebuild
- Check for syntax errors in modified files

**Simulation won't start?**
- Ensure no syntax errors
- Check that all hardware calls are simulation-safe

**Robot code crashes?**
- Check for null pointer exceptions
- Verify all subsystems are instantiated
- Check network table connections

## Questions?

If unsure about FRC-specific patterns or WPILib APIs:
1. Check the WPILib documentation: https://docs.wpilib.org/
2. Look at example projects in this repository
3. Ask for clarification if the robot design is unclear
