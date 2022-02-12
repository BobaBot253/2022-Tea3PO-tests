package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer.CargoPipeline;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class VisionTrack implements Command {

    private static final PIDController TURN_PID_CONTROLLER = new PIDController(VisionConstants.kPTurn,
            VisionConstants.kITurn, VisionConstants.kDTurn);
    private static final PIDController DIST_PID_CONTROLLER = new PIDController(VisionConstants.kPDist,
            VisionConstants.kIDist, VisionConstants.kDDist);

    private Subsystem[] requirements = { RobotContainer.drivetrain };
    
    public VisionTrack(CargoPipeline color) {
        RobotContainer.limelight.getEntry("pipeline").setNumber(color.val);
    }
/* For on-the-go changing of cargo targets: not to be used in competition
    public void changePipeline(CargoPipeline color) {
        RobotContainer.limelight.getEntry("pipeline").setNumber(color.val);
    }
*/
    @Override
    public void initialize() {
        //RobotContainer.getInstance().setLEDMode(LEDMode.ON);
        //RobotContainer.getInstance().setPipeline(0);

    }
    /*
    * @param measured the measured ratio
    * @param target the desired ratio
    * @param tolerance the leniency you will accept
    */
    public boolean withinTarget(double measured, double target, double tolerance) {
        return (Math.abs(measured - target) <= tolerance);
    }
    @Override
    public void execute() {

        double left, right;

        double turnError = RobotContainer.limelight.getEntry("tx").getDouble(0);
        double distError = RobotContainer.limelight.getEntry("ty").getDouble(0);

        if (turnError < VisionConstants.kTurnTolerance) turnError = 0;
        if (distError < VisionConstants.kDistTolerance) distError = 0;

        double throttle = DIST_PID_CONTROLLER.calculate(distError, 0);
        double turn = TURN_PID_CONTROLLER.calculate(turnError, 0);
        double thor = RobotContainer.limelight.getEntry("thor").getDouble(0);
        double tvert = RobotContainer.limelight.getEntry("tvert").getDouble(0);
        double ratio = thor/tvert;
        SmartDashboard.putNumber("h-v ratio", ratio);
        if (throttle != 0) {
            //SmartDashboard.putBoolean("Vision ratio is acceptable?", true);
            left = right = 0;
        } else {
            // Turns in place when there is no throttle input
            left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
            right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;

            left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
            right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
            //SmartDashboard.putBoolean("Vision ratio is acceptable?", false);
        }
        
       
        //Drivetrain.setOpenLoop(left, right);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //RobotContainer.getInstance().setLEDMode(LEDMode.OFF);
        Drivetrain.setOpenLoop(0.0, 0.0);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}