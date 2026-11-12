
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {

    // =====================================
    // ALL AVAILABLE BLINKIN PATTERNS
    // =====================================
    //https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe

    public enum Pattern {

        RAINBOW(-0.99),
        RAINBOW_PARTY(-0.97),
        RAINBOW_OCEAN(-0.95),
        RAINBOW_LAVA(-0.93),
        RAINBOW_FOREST(-0.91),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),

        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),

        SINELON_RAINBOW(-0.79),
        SINELON_PARTY(-0.77),
        SINELON_OCEAN(-0.75),
        SINELON_LAVA(-0.73),
        SINELON_FOREST(-0.71),

        BPM_RAINBOW(-0.69),
        BPM_PARTY(-0.67),
        BPM_OCEAN(-0.65),
        BPM_LAVA(-0.63),
        BPM_FOREST(-0.61),

        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),

        TWINKLES_RAINBOW(-0.55),
        TWINKLES_PARTY(-0.53),
        TWINKLES_OCEAN(-0.51),
        TWINKLES_LAVA(-0.49),
        TWINKLES_FOREST(-0.47),

        COLOR_WAVES_RAINBOW(-0.45),
        COLOR_WAVES_PARTY(-0.43),
        COLOR_WAVES_OCEAN(-0.41),
        COLOR_WAVES_LAVA(-0.39),
        COLOR_WAVES_FOREST(-0.37),

        LARSON_SCANNER_RED(-0.35),
        LARSON_SCANNER_GRAY(-0.33),

        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),

        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),

        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),

        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),

        COLOR1_SOLID(-0.03),
        COLOR1_LARSON(-0.01),
        COLOR1_LIGHT_CHASE(0.01),
        COLOR1_HEARTBEAT_SLOW(0.03),
        COLOR1_HEARTBEAT_MEDIUM(0.05),
        COLOR1_HEARTBEAT_FAST(0.07),
        COLOR1_BREATH_SLOW(0.09),
        COLOR1_BREATH_FAST(0.11),
        COLOR1_SHOT(0.13),
        COLOR1_STROBE(0.15),

        COLOR2_SOLID(0.17),
        COLOR2_LARSON(0.19),
        COLOR2_LIGHT_CHASE(0.21),
        COLOR2_HEARTBEAT_SLOW(0.23),
        COLOR2_HEARTBEAT_MEDIUM(0.25),
        COLOR2_HEARTBEAT_FAST(0.27),
        COLOR2_BREATH_SLOW(0.29),
        COLOR2_BREATH_FAST(0.31),
        COLOR2_SHOT(0.33),
        COLOR2_STROBE(0.35),

        COLOR1_COLOR2_SPARKLE(0.37),
        COLOR2_COLOR1_SPARKLE(0.39),
        COLOR1_COLOR2_GRADIENT(0.41),
        COLOR1_COLOR2_BPM(0.43),
        COLOR1_TO_COLOR2_BLEND(0.45),
        COLOR1_COLOR2_BLEND(0.47),
        COLOR1_COLOR2_NO_BLEND(0.49),
        COLOR1_COLOR2_TWINKLES(0.51),
        COLOR1_COLOR2_WAVES(0.53),
        COLOR1_COLOR2_SINELON(0.55),

        SOLID_HOT_PINK(0.57),
        SOLID_DARK_RED(0.59),
        SOLID_RED(0.61),
        SOLID_RED_ORANGE(0.63),
        SOLID_ORANGE(0.65),
        SOLID_GOLD(0.67),
        SOLID_YELLOW(0.69),
        SOLID_LAWN_GREEN(0.71),
        SOLID_LIME(0.73),
        SOLID_DARK_GREEN(0.75),
        SOLID_GREEN(0.77),
        SOLID_BLUE_GREEN(0.79),
        SOLID_AQUA(0.81),
        SOLID_SKY_BLUE(0.83),
        SOLID_DARK_BLUE(0.85),
        SOLID_BLUE(0.87),
        SOLID_BLUE_VIOLET(0.89),
        SOLID_VIOLET(0.91),
        SOLID_WHITE(0.93),
        SOLID_GRAY(0.95),
        SOLID_DARK_GRAY(0.97),
        SOLID_BLACK(0.99);

        // Stores the PWM value for each pattern
        private final double pwmValue;

        // Constructor
        Pattern(double pwmValue) {
            this.pwmValue = pwmValue;
        }

        // Returns the PWM value
        public double getPwmValue() {
            return pwmValue;
        }
    }

    // =====================================
    // BLINKIN LED DRIVER
    // =====================================

    private final Spark blinkin = new Spark(0);

    public LEDS() {

        // Set starting LED pattern
        setPattern(Pattern.SHOT_BLUE);
    }

    // =====================================
    // SET LED PATTERN
    // =====================================

    // Immediately changes the LED pattern
    public void setPattern(Pattern pattern) {
        blinkin.set(pattern.getPwmValue());
    }

    // =====================================
    // LED COMMANDS
    // =====================================

    // Creates a command to change the LED pattern
    public Command setPatternCommand(Pattern pattern) {
        return runOnce(() -> setPattern(pattern));
    }

    // =====================================
    // TEST COMMANDS
    // =====================================

    public Command test() {
        return setPatternCommand(Pattern.SOLID_HOT_PINK);
    }

    // public Command w() {
    //     return setPatternCommand(Pattern.SOLID_GREEN);
    // }

    // public Command e() {
    //     return setPatternCommand(Pattern.SOLID_WHITE);
    // }

    // public Command r() {
    //     return setPatternCommand(Pattern.SOLID_VIOLET);
    // }

    // public Command t() {
    //     return setPatternCommand(Pattern.SOLID_AQUA);
    // }
}
