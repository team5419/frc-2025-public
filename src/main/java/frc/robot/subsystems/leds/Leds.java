package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;
import frc.robot.constants.Ports;
import frc.robot.lib.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds extends VirtualSubsystem {

    private static final int kLength = 18;
    private static final int kDefaultPercent = 75;

    private static Leds instance;

    public static Leds getInstance() {
        if (instance == null) instance = new Leds();
        return instance;
    }

    private enum LEDState {
        OFF,
        DS_DISCONNECTED,
        DISABLED,
        TELEOP,
        AUTO,
        AUTO_ALIGN,
        E_STOPPED,
        HAS_CORAL,
        MOVING_ELE,
        CLIMBING;
    }

    private final Timer coralTimer;

    @AutoLogOutput(key = "Leds/Current state")
    private LEDState currentState;

    private RobotState state;
    private LEDPattern currentPattern;
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView viewOne;
    private final AddressableLEDBufferView viewTwo;

    private Leds() {
        leds = new AddressableLED(Ports.kLedPort);
        buffer = new AddressableLEDBuffer(kLength);
        viewOne = buffer.createView(0, kLength / 2 - 1);
        viewTwo = buffer.createView(kLength / 2, kLength - 1);
        leds.setLength(kLength);
        leds.setData(buffer);
        leds.start();
        coralTimer = new Timer();
        state = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        if (!DriverStation.isDSAttached()) {
            currentPattern = modifierScrollRelative(conGradient(Color.kBlue, Color.kRed), 140);
            currentState = LEDState.DS_DISCONNECTED;
        } // scroll red & blue
        else if (DriverStation.isEStopped()) {
            currentPattern = modifierBreathe(solidColor(Color.kWhite), 0.5);
            currentState = LEDState.E_STOPPED;
        } // blink white
        else if (DriverStation.isDisabled()) {
            currentPattern = modifierBreathe(
                    solidColor(DriverStation.getAlliance().get() == Alliance.Blue ? Color.kBlue : Color.kRed), 2);
            currentState = LEDState.DISABLED;
        } // breathe red or blue depending on alliance
        else if (state.isClimbing()) {
            // currentPattern = modifierScrollRelative(rainbow(), 80);
            currentPattern = modifierScrollRelative(conGradient(Color.kAquamarine, Color.kNavy), 140);
            // currentPattern = modifierBreathe(solidColor(new Color(255, 71, 103)), 0.6);
            currentState = LEDState.CLIMBING;
        } // scroll rainbow
        else if (state.isAligning()) {
            currentPattern = solidColor(Color.kRoyalBlue);
            currentState = LEDState.AUTO_ALIGN;
        } // solid blue
        else if (state.isEleDesiring() || (coralTimer.hasElapsed(1) && state.isDisplacing())) {
            currentState = LEDState.MOVING_ELE;
            currentPattern = modifierMask(
                    solidColor(state.isDisplacing() ? new Color(113, 212, 28) : new Color(168, 0, 168)),
                    LEDPattern.progressMaskLayer(this::getEleProgress));
        } // pink progress bar based on desired level
        else if (state.isHasCoral()) {
            currentState = LEDState.HAS_CORAL;
            currentPattern = solidColor(new Color(161, 43, 0));
        } // solid yellow
        else if (DriverStation.isTeleop()) {
            currentState = LEDState.TELEOP;
            currentPattern = solidColor(Color.kRed);
        } // solid red
        else if (DriverStation.isAutonomous()) {
            currentState = LEDState.AUTO;
            currentPattern = solidColor(Color.kLimeGreen);
        } // solid green
        else {
            currentState = LEDState.OFF;
            currentPattern = solidColor(Color.kBlack);
        }

        if (!state.isVisionConnected()) modifierBlinkSymmetrical(currentPattern, 0.5); // blink current color
        if (currentState == LEDState.MOVING_ELE) {
            currentPattern.applyTo(viewOne);
            currentPattern.applyTo(viewTwo);
            for (int i = 0; i < viewOne.getLength(); i++) {
                buffer.setLED(i, viewOne.getLED(i));
            }
            for (int i = viewOne.getLength(); i < viewTwo.getLength(); i++) {
                buffer.setLED(i, viewTwo.getLED(i));
            }

        } else currentPattern.applyTo(buffer);
        leds.setData(buffer);
        if (!state.isHasCoral()) coralTimer.restart();
    }

    private double getEleProgress() {
        return switch (state.getElevatorLevel()) {
            case L1 -> 0.25;
            case L2 -> 0.5;
            case L3 -> 0.75;
            default -> 1;
        };
    }

    // --------------- base functions that return an LEDPattern ---------------

    private LEDPattern rainbow() {
        return LEDPattern.rainbow(255, 128).atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern solidColor(Color color) {
        return LEDPattern.solid(color(color)).atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern conGradient(Color color1, Color color2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color(color1), color(color2))
                .atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern conGradientSeperated(Color color1, Color color2) {
        return LEDPattern.gradient(
                        LEDPattern.GradientType.kContinuous, color(color1), Color.kBlack, color(color2), Color.kBlack)
                .atBrightness(Percent.of(kDefaultPercent));
    }

    private LEDPattern disconGradient(Color color1, Color color2) {
        return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color(color1), color(color2))
                .atBrightness(Percent.of(kDefaultPercent));
    }

    // --------------- modifiers that take an LEDPattern and change it ---------------

    private LEDPattern modifierOffset(LEDPattern base, int dist) {
        return base.offsetBy(dist);
    }

    private LEDPattern modifierScrollAbsolute(LEDPattern base, double InchesPerSecond, int density) {
        Distance ledSpacing = Foot.of(1 / density);
        return base.scrollAtAbsoluteSpeed(Inches.per(Second).of(InchesPerSecond), ledSpacing);
    }

    private LEDPattern modifierScrollRelative(LEDPattern base, double percentPerSecond) {
        return base.scrollAtRelativeSpeed(Percent.per(Second).of(percentPerSecond));
    }

    private LEDPattern modifierBreathe(LEDPattern base, double time) {
        return base.breathe(Seconds.of(time));
    }

    private LEDPattern modifierBlinkSymmetrical(LEDPattern base, double time) {
        return base.blink(Seconds.of(time));
    }

    private LEDPattern modifierBlinkAsymmetrical(LEDPattern base, double timeon, double timeoff) {
        return base.blink(Seconds.of(timeon), Seconds.of(timeoff));
    }

    private LEDPattern modifierMask(LEDPattern base, LEDPattern mask) {
        return base.mask(mask);
    }

    private LEDPattern modifierOverlay(LEDPattern base, LEDPattern mask) {
        return mask.overlayOn(base);
    }

    private LEDPattern modifierBlend(LEDPattern base, LEDPattern mask) {
        return base.blend(mask);
    }

    /** Converts a color to display the correct color (swaps green and red) */
    private Color color(Color color) {
        return new Color(color.green, color.red, color.blue);
    }
}
