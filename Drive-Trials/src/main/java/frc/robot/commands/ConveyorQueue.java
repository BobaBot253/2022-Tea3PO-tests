package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Conveyor;

public class ConveyorQueue implements Command {

    private Subsystem[] requirements = {Conveyor.getInstance()};
    private PowerDistribution pdp = new PowerDistribution();

    public enum State {
        OneSensor, TwoSensors, None
    }

    private State state;

    Timer photoelectric_delay = new Timer();
    Timer current_spike = new Timer();

    public ConveyorQueue(State state) {
        this.state = state;
        photoelectric_delay.start();
        current_spike.start();
    }

    public void execute() { 
        if (Conveyor.getInstance().getHorizontalSensor()){
            if (Conveyor.getInstance().getVerticalSensor()){
                Conveyor.getInstance().setOpenLoop(0);
                
            } else {
                Conveyor.getInstance().setOpenLoop(0.45);
            } 
        } else {
            Conveyor.getInstance().setOpenLoop(0);
        }
    }

    public void end() {
        Conveyor.getInstance().stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }    
}
