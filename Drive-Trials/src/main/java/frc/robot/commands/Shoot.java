package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor;

public class Shoot implements Command {
    
    private Subsystem[] requirements = {RobotContainer.shooter, RobotContainer.conveyor, RobotContainer.intake};
    
    Timer encoderFallback;

    public Shoot(){
        encoderFallback = new Timer();

        
    }

    @Override
    public void initialize() {
        encoderFallback.start();
        encoderFallback.reset();
        
    }
    @Override
    public void execute(){

        Shooter.getInstance().setOpenLoop(0.65);

        if(Shooter.getInstance().getShooterVelocity() > 3400){
            Conveyor.getInstance().setOpenLoop(0.55);
            Intake.getInstance().setConveyor(0.50);     

            encoderFallback.reset();
        } else if(encoderFallback.hasElapsed(1.5)){
            Conveyor.getInstance().setOpenLoop(0.55);
            Intake.getInstance().setConveyor(0.50); 

            
        }
        
        
    }
    
    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stop();
        Conveyor.getInstance().stop();
        Intake.getInstance().stopConveyor();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}