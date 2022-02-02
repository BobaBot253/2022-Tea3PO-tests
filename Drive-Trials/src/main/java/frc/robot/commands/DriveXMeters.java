package frc.robot.commands;
import java.util.Set;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class DriveXMeters implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};
    private TrapezoidProfile.State goal;
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints;
    private double startTime;
    

    /*
        @param distance     desired distance
        @param maxSpeedMPS  max speed during motion
        @param maxAccelMPSS max acceleration during motion
    */
    public DriveXMeters(double distance, double maxSpeedMPS, double maxAccelMPSS) {
        constraints = new TrapezoidProfile.Constraints(maxSpeedMPS, maxAccelMPSS);
        goal = new TrapezoidProfile.State(distance, 0);
        profile = new TrapezoidProfile(constraints, goal);
    }


    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        //TrapezoidProfile.State profileCalc = profile.calculate(Constants.dt);
        TrapezoidProfile.State profileCalc = profile.calculate(Timer.getFPGATimestamp() - startTime);
        double left, right;
        left = Drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        right = Drivetrain.FEEDFORWARD.calculate(profileCalc.velocity);
        left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncMeters(), profileCalc.velocity);
        right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncMeters(), profileCalc.velocity);
        left /= Constants.kMaxVoltage;
        right /= Constants.kMaxVoltage;
        drivetrain.setOpenLoop(left, right);
        SmartDashboard.putString("DriveXFinished", "No");
        //profile = new TrapezoidProfile(constraints, goal, profileCalc);
    }

    @Override 
    public boolean isFinished() {
        return profile.isFinished(Timer.getFPGATimestamp() - startTime);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("DriveXFinished", "Yes");
        drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}