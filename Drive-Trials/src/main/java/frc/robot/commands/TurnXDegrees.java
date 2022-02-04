package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnXDegrees implements Command {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private Subsystem[] requirements = {drivetrain};
    private PIDController turnController;
    private double degrees;
    private double startAngle;

    public TurnXDegrees(double degrees) {
        turnController = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
        this.degrees = degrees;
        turnController.setTolerance(0.5);
        startAngle = RobotContainer.navX.getAngle();
        turnController.setSetpoint(degrees + startAngle);
    }

    

    @Override
    public void execute() {

        double turn = turnController.calculate(RobotContainer.navX.getAngle());
        double left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens
        *DrivetrainConstants.buffer;
        double right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens
        *DrivetrainConstants.buffer;
        left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
        right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
        
        SmartDashboard.putString("TurnXFinished?", "No");
        Drivetrain.setOpenLoop(left, right);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("TurnXFinished?", "Yes");
        Drivetrain.stop();
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
