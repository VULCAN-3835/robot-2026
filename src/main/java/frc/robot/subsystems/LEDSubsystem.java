package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int kPort   = 2;
    private static final int kLength = 12;

    private final AddressableLED       led;
    private final AddressableLEDBuffer buffer;
    private final Timer                timer = new Timer();

    // Two scanners — each has a retained trail array that decays each frame
    private final double[] trail1 = new double[kLength];
    private final double[] trail2 = new double[kLength];

    private double pos1 = 0.0,  vel1 =  9.0;  // LEDs/sec
    private double pos2 = 6.0,  vel2 = -6.5;  // opposite direction, slightly different speed

    private double lastT = 0.0;

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
        double t  = timer.get();
        double dt = Math.min(t - lastT, 0.05); // cap to avoid big jump on first frame
        lastT = t;

        // Move scanners and bounce off ends
        pos1 += vel1 * dt;
        if (pos1 >= kLength - 1) { pos1 = 2 * (kLength - 1) - pos1; vel1 = -vel1; }
        if (pos1 <= 0)            { pos1 = -pos1;                     vel1 = -vel1; }

        pos2 += vel2 * dt;
        if (pos2 >= kLength - 1) { pos2 = 2 * (kLength - 1) - pos2; vel2 = -vel2; }
        if (pos2 <= 0)            { pos2 = -pos2;                     vel2 = -vel2; }

        // Decay trails — frame-rate independent
        double decay = Math.pow(0.82, dt * 60);
        for (int i = 0; i < kLength; i++) {
            trail1[i] *= decay;
            trail2[i] *= decay;
        }

        // Subpixel head paint — splits brightness across the two nearest LEDs
        paintHead(trail1, pos1);
        paintHead(trail2, pos2);

        // Render — scanner1 = warm white, scanner2 = ice blue, overlap = pure white
        for (int i = 0; i < kLength; i++) {
            double a = trail1[i] * trail1[i]; // gamma correct
            double b = trail2[i] * trail2[i];

            int r = (int) Math.min(255, a * 240);
            int g = (int) Math.min(255, a * 220 + b * 200);
            int bl = (int) Math.min(255, a * 200 + b * 255);

            buffer.setRGB(i, g, r, bl); // GRB strip
        }

        led.setData(buffer);
    }

    // Splits the head brightness across two adjacent LEDs for smooth subpixel motion
    private void paintHead(double[] trail, double pos) {
        int    lo   = (int) pos;
        double frac = pos - lo;
        if (lo     < kLength) trail[lo]     = Math.max(trail[lo],     1.0 - frac);
        if (lo + 1 < kLength) trail[lo + 1] = Math.max(trail[lo + 1], frac);
    }
}
