package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int    kPort       = 2;
    private static final int    kLength     = 20;
    private static final double kPoliceHz   = 1.5; // flashes per second (police speed)

    private final AddressableLED       led;
    private final AddressableLEDBuffer buffer;
    private final Timer                timer = new Timer();

    public LEDSubsystem() {
        led    = new AddressableLED(kPort);
        buffer = new AddressableLEDBuffer(kLength);
        led.setLength(kLength);
        led.setData(buffer);
        led.start();
        timer.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            updateAllianceColor();
        } else {
            updatePoliceLights();
        }
        led.setData(buffer);
    }

    // Left half red / right half blue, then swaps at kPoliceHz
    private void updatePoliceLights() {
        int half = kLength / 2;
        boolean phase = (timer.get() % (1.0 / kPoliceHz)) < (0.5 / kPoliceHz);

        for (int i = 0; i < kLength; i++) {
            boolean isLeftHalf = i < half;
            boolean showRed = isLeftHalf ^ phase; // XOR flips which side is red each phase
            if (showRed) {
                buffer.setRGB(i, 0, 255, 0); // GRB strip: swap R↔G to get red
            } else {
                buffer.setRGB(i, 0, 0, 255);
            }
        }
    }

    // Solid alliance color when disabled
    private void updateAllianceColor() {
        boolean isBlue = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        // GRB strip: first byte = green channel, second = red channel
        int r = isBlue ? 0   : 0;   // green channel: 0 for both
        int g = isBlue ? 0   : 255; // red channel:   255 for red alliance
        int b = isBlue ? 255 : 0;

        for (int i = 0; i < kLength; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
}
