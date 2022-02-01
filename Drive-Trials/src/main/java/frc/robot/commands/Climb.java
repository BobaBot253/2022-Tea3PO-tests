package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends WaitCommand {
    public enum Side {
        LEFT, RIGHT, BOTH
    }
    
    Side side;
    double speed;
    private Climber climber = RobotContainer.climber;
    
    public Climb(Side side, double speed) {
        super(0.2);
        this.side = side;
        this.speed = speed;
        
        m_requirements.add(climber);
    }
    
    @Override
    public void initialize() {
        switch (side) {
            case LEFT:
                climber.leftServoDown();
                break;
            case BOTH:
                climber.leftServoDown();
            case RIGHT:
                climber.rightServoDown();
                break;
        }
        
        super.initialize();
    }
    
    @Override
    public void execute() {
        // only runs if the time has elapsed
        if (super.isFinished()) {
            switch (side) {
                case LEFT:
                    climber.climbLeft(speed);
                    break;
                case BOTH:
                    climber.climbLeft(speed);
                case RIGHT:
                    climber.climbRight(speed);
                    break;
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            switch (side) {
                case LEFT:
                    climber.stopLeftMotor();
                    break;
                case BOTH:
                    climber.stopLeftMotor();
                case RIGHT:
                    climber.stopRightMotor();
                    break;
            }
        }
        super.end(interrupted);
    }
}