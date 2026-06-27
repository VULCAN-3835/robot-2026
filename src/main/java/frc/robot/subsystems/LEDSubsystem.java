package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int    kPort          = 2;
    private static final int    kLength        = 12;
    private static final double kMinBrightness = 0.25; // no LED goes fully off

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
        double t = timer.get();

        // Hue drifts very slowly through a tight purple band (WPILib 0-180)
        // 130 = blue-purple, 135 = pure purple, 140 = red-purple/violet
        // Slow 12-second cycle, stays clearly purple the whole time
        double baseHue = 135 + 5 * Math.sin(t * (2 * Math.PI / 12.0));

        // Every 7 seconds: 0.8 s shift toward a richer violet, then back
        double accentPhase = t % 7.0;
        double accentBlend = accentPhase < 0.8
                ? Math.sin(accentPhase / 0.8 * Math.PI)
                : 0.0;

        for (int i = 0; i < kLength; i++) {
            // Single clean wave flowing along the strip
            double phase = i * (2 * Math.PI / kLength) * 1.5 - t * 2.5;
            double wave  = (Math.sin(phase) + 1.0) / 2.0; // [0, 1]

            // Floor so no LED ever goes dark
            double brightness = kMinBrightness + (1.0 - kMinBrightness) * wave;

            // Accent nudges hue toward deeper violet
            double hue = baseHue + accentBlend * 6.0;

            // Full saturation — pure vivid purple, slight breathe during accent
            int sat = (int) Math.min(255, 245 + accentBlend * 10);
            int val = (int) (brightness * 255);

            buffer.setHSV(i, (int) hue, sat, val);
        }

        led.setData(buffer);
    }
}
