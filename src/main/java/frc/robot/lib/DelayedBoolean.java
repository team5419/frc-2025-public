package frc.robot.lib;

/** An iterative boolean latch that delays the transition from false to true. */
public class DelayedBoolean {

    private boolean lastValue;
    private double transitionTimestamp;
    private final double delay;

    public DelayedBoolean(double timestamp, double delay) {
        transitionTimestamp = timestamp;
        lastValue = false;
        this.delay = delay;
    }

    public boolean update(double timestamp, boolean value) {
        boolean result = false;

        if (value && !lastValue) transitionTimestamp = timestamp;

        // If we are still true and we have transitioned.
        if (value && (timestamp - transitionTimestamp > delay)) result = true;

        lastValue = value;
        return result;
    }
}
