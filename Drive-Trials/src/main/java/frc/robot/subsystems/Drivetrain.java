package frc.robot.subsystems;

import java.util.List;

import frc.robot.Units;
import frc.robot.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;


public class Drivetrain implements Subsystem {
    private static final TalonFX
        leftMaster = Util.createTalonFX(DrivetrainConstants.leftMaster),
        leftSlave = Util.createTalonFX(DrivetrainConstants.leftSlave),
        rightMaster = Util.createTalonFX(DrivetrainConstants.rightMaster),
        rightSlave = Util.createTalonFX(DrivetrainConstants.rightSlave);
    
    public static final List<TalonFX> motors = List.of(leftMaster, leftSlave, rightMaster, rightSlave);

    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);
    public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA);
    public static final PIDController LEFT_PID_CONTROLLER = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
    public static final PIDController RIGHT_PID_CONTROLLER = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
    private static Drivetrain instance;
    private Drivetrain() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        // Inverting opposite sides of the drivetrain
        List.of(leftMaster, leftSlave).forEach(motor -> motor.setInverted(false));
        List.of(rightMaster, rightSlave).forEach(motor -> motor.setInverted(true));
        
        register();
    }
    public static Drivetrain getInstance() {
        if(instance == null) instance = new Drivetrain();
        return instance;
    }
    /**
     * Sets drivetrain speeds in open loop (% of max voltage)
     *
     * @param left   Percent output of motors on left side of drivetrain
     * @param right  Percent output of motors on right side of drivetrain
     */
    public static void setOpenLoop(Double left, Double right){
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);

    }
    /**
     * Stops the drivetrain motors by setting their speed to 0
     */
    public static void stop(){
        setOpenLoop(0.0, 0.0);
    }

    public void resetEncoders() {
        resetEncoders(0, 0);
    }
    
    /**
     * Sets encoders to a specific value
     * @param left  left wheel value
     * @param right right wheel value
     */
    public void resetEncoders(int left, int right) {
        rightMaster.setSelectedSensorPosition(right);
        leftMaster.setSelectedSensorPosition(left);
    }

    /**
     * @return the left and right drivetrain velocities (in meters/sec) as a DifferentialDriveWheelSpeeds object
     */
    public static DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncVelocityMeters(), getRightEncVelocityMeters());
    }
    
    /**
     * @return the current position measurement of the left drivetrain encoder in talon native units (ticks)
     */
    public static double getLeftEnc() {
        return leftMaster.getSelectedSensorPosition();
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in talon native units (ticks/)
     */
    public static double getRightEnc() {
        return rightMaster.getSelectedSensorPosition();
    }

    /**
     * @return the current position measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getLeftEnc());
    }
    
    /**
     * @return the current position measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncMeters() {
        return Units.DrivetrainUnits.TicksToMeters(getRightEnc());
    }


    
    /**
     * @return the current velocity measurement of the left drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getLeftEncVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }
    
    /**
     * @return the current velocity measurement of the right drivetrain encoder in talon native units (ticks/ds)
     */
    public static double getRightEncVelocity() {
        return rightMaster.getSelectedSensorVelocity();
    }

    /**
     * @return the current velocity measurement of the left drivetrain encoder in meters
     */
    public static double getLeftEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getLeftEncVelocity());
    }

    /**
     * @return the current velocity measurement of the right drivetrain encoder in meters
     */
    public static double getRightEncVelocityMeters() {
        return Units.DrivetrainUnits.TicksPerDecisecondToMPS(getRightEncVelocity());
    }
    
    /* Static class to contain the speeds of each side of the drivetrain */
    public static class WheelState {
        public double left, right;
        
        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }
}
