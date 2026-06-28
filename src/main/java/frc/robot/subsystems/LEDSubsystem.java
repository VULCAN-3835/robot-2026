package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int    kPort          = 2;
    private static final int    kLength        = 12;
    private static final double kMinBrightness = 0.25; // no LED goes fully off

    private AddressableLED       led;
    private AddressableLEDBuffer buffer;
    private final Timer          timer   = new Timer();
    private boolean              enabled = true;

    public LEDSubsystem() {
        try {
            led    = new AddressableLED(kPort);
            buffer = new AddressableLEDBuffer(kLength);
            led.setLength(kLength);
            led.setData(buffer);
            led.start();
            timer.start();
        } catch (Exception e) {
            System.err.println("[LEDSubsystem] Init failed, LEDs disabled: " + e.getMessage());
            enabled = false;
        }
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        try {
            double t = timer.get();

            // WPILib HSV hue: 0-180. Pure blue = 120, cyan-blue = 110, deep blue = 125
            // Slow drift between electric blue and cyan-blue over 10 seconds
            double baseHue = 118 + 6 * Math.sin(t * (2 * Math.PI / 10.0));

            // Every 5 seconds: quick 0.6 s pulse toward bright cyan, then back
            double accentPhase = t % 5.0;
            double accentBlend = accentPhase < 0.6
                    ? Math.sin(accentPhase / 0.6 * Math.PI)
                    : 0.0;

            for (int i = 0; i < kLength; i++) {
                // Wave flows along the strip; 1.5 wave cycles visible, moderate speed
                double phase = i * (2 * Math.PI / kLength) * 1.5 - t * 2.0;
                double wave  = (Math.sin(phase) + 1.0) / 2.0; // [0, 1]

                // No LED goes fully off
                double brightness = kMinBrightness + (1.0 - kMinBrightness) * wave;

                // Accent pulls hue toward cyan
                double hue = baseHue - accentBlend * 8.0;

                int sat = (int) Math.min(255, 240 + accentBlend * 15);
                int val = (int) (brightness * 255);

                buffer.setHSV(i, (int) hue, sat, val);
            }

            led.setData(buffer);
        } catch (Exception e) {
            System.err.println("[LEDSubsystem] periodic error: " + e.getMessage());
        }
    }
}
