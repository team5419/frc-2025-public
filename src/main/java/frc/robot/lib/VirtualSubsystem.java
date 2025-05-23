package frc.robot.lib;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
    private static List<VirtualSubsystem> registered = new ArrayList<>();

    public VirtualSubsystem() {
        registered.add(this);
    }

    public abstract void periodic();

    public static void periodicAll() {
        registered.forEach(s -> s.periodic());
    }
}
