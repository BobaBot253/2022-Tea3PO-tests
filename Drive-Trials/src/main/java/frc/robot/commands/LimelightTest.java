package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LimelightTest implements Command {
    private Subsystem[] requirements = {};

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
