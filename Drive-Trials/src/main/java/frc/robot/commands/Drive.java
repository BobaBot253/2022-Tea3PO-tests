package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class Drive implements Command {
    public enum State {
        CurvatureDrive2019, CheesyDriveOpenLoop, CheesyDriveClosedLoop
    }
    
    private State state;
    Subsystem[] requirements = { RobotContainer.drivetrain };
    
    public Drive(State state){
        this.state = state;
    }
    
    @Override
    public void execute(){
        // Retrieving the deadbanded throttle and turn values (the controller joystick values)
        double throttle = RobotContainer.getThrottleValue();
        double turn = RobotContainer.getTurnValue();
        double left, right;
        
        switch (state) {
            
            case CurvatureDrive2019:
                // Differential drive as long as throttle is greater than zero (deadbanded).
                if (throttle != 0) {
                    left = (throttle + throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;
                    right = (throttle - throttle * turn * DriverConstants.kTurnSens) * DriverConstants.kDriveSens;

                    // Normalize
                    double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
                    
                    if(maxMagnitude > DriverConstants.kDriveSens) {
                        left = left / maxMagnitude * DriverConstants.kDriveSens;
                        right = right / maxMagnitude * DriverConstants.kDriveSens;
                    } 

                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DriverConstants.kTurnInPlaceSens;
                }

                break;

            case CheesyDriveOpenLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens;
                    turn *= DrivetrainConstants.kMaxCurvature * DriverConstants.kTurnSens * throttle;

                    DifferentialDriveWheelSpeeds wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    wSpeeds.desaturate(DrivetrainConstants.kMaxSpeedMPS);

                    left = Drivetrain.FEEDFORWARD.calculate(wSpeeds.leftMetersPerSecond) / Constants.kMaxVoltage;
                    right = Drivetrain.FEEDFORWARD.calculate(wSpeeds.rightMetersPerSecond) / Constants.kMaxVoltage;

                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;

                    left = Drivetrain.FEEDFORWARD.calculate(left) / Constants.kMaxVoltage;
                    right = Drivetrain.FEEDFORWARD.calculate(right) / Constants.kMaxVoltage;
                }

                break;

            case CheesyDriveClosedLoop:
                if (throttle != 0) {
                    throttle *= DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kDriveSens;
                    turn *= DrivetrainConstants.kMaxCurvature * DriverConstants.kTurnSens * throttle;

                    DifferentialDriveWheelSpeeds _wSpeeds = Drivetrain.KINEMATICS.toWheelSpeeds(new ChassisSpeeds(throttle, 0, turn));
                    _wSpeeds.desaturate(DrivetrainConstants.kMaxSpeedMPS);

                    left = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.leftMetersPerSecond);
                    right = Drivetrain.FEEDFORWARD.calculate(_wSpeeds.rightMetersPerSecond);

                    left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncVelocityMeters(), _wSpeeds.leftMetersPerSecond);
                    right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncVelocityMeters(), _wSpeeds.rightMetersPerSecond);
                    
                    // Convert voltages to percent voltages
                    left /= Constants.kMaxVoltage;
                    right /= Constants.kMaxVoltage;
                } else {
                    // Turns in place when there is no throttle input
                    left = turn * DrivetrainConstants.kMaxSpeedMPS * DriverConstants.kTurnInPlaceSens;
                    right = -turn * DrivetrainConstants.kMaxSpeedMPS* DriverConstants.kTurnInPlaceSens;

                    left = Drivetrain.FEEDFORWARD.calculate(left);
                    right = Drivetrain.FEEDFORWARD.calculate(right);

                    left += Drivetrain.LEFT_PID_CONTROLLER.calculate(Drivetrain.getLeftEncVelocityMeters(), left);
                    right += Drivetrain.RIGHT_PID_CONTROLLER.calculate(Drivetrain.getRightEncVelocityMeters(), right);
                }
                
                break;

            default:
                left = 0;
                right = 0;
                break;
        }

        Drivetrain.setOpenLoop(left, right);
    }
    // When this command ends, it stops the drivetrain to guarantee safety
    @Override
    public void end(boolean interrupted) {
        Drivetrain.stop();
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
