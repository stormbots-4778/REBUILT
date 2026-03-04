package frc.robot.subsystems.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.RobotConfiguration.LightsConfig;

// untested
public class Blinkin extends SubsystemBase {
    public enum Pattern {
        CONFETTI(-0.87),
        RAINBOW(-0.99),
        HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        RED_ORANGE(0.63),
        ORANGE(0.65),
        GOLD(0.67),
        YELLOW(0.69),
        LAWN_GREEN(0.71),
        LIME(0.73),
        DARK_GREEN(0.75),
        GREEN(0.77),
        BLUE_GREEN(0.79),
        AQUA(0.81),
        SKY_BLUE(0.83),
        DARK_BLUE(0.85),
        BLUE(0.87),
        BLUE_VIOLET(0.89),
        VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        OFF(0.99);

        private double value;

        Pattern(double value) {
            this.value = value;
        }
    }

    private final Spark blinkin = new Spark(LightsConfig.blinkinPort);

    public void setPattern(Pattern pattern) {
        blinkin.set(pattern.value);
    }
}
