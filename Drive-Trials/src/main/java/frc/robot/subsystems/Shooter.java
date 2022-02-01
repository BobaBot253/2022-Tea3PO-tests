package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem {
    private static final CANSparkMax master = new CANSparkMax(ShooterConstants.master, MotorType.kBrushless);
    private static final CANSparkMax slave = new CANSparkMax(ShooterConstants.slave, MotorType.kBrushless);

    private static CANPIDController pidController;
    private static CANEncoder encoder = master.getEncoder();

    private static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
    
    private static Shooter instance;
    public static Shooter getInstance(){
        if (instance == null) instance = new Shooter();
        return instance;
    }
    
    private Shooter(){

        master.restoreFactoryDefaults();
        slave.restoreFactoryDefaults();

        master.enableVoltageCompensation(12);
        slave.enableVoltageCompensation(12);

        master.setIdleMode(IdleMode.kCoast);
        slave.setIdleMode(IdleMode.kCoast);

        master.setInverted(false);
        slave.follow(master, true);

        pidController = master.getPIDController();
        pidController.setP(ShooterConstants.kP, ShooterConstants.kSlotID);
        pidController.setI(ShooterConstants.kI, ShooterConstants.kSlotID);
        pidController.setD(ShooterConstants.kD, ShooterConstants.kSlotID);
        pidController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax, ShooterConstants.kSlotID);

        master.burnFlash();
        slave.burnFlash();

        register();
    }

    @Override
    public void periodic(){
    }

    /**
     * Sets the speed of the shooter in percent of max voltage (overriding the closed loop velocity control)
     * @param value percent of voltage [-1, 1]
     */
    public void setOpenLoop(final double value){
        master.set(value);
       
    }

    public double getShooterVelocity(){
        return encoder.getVelocity();
    }
    
    /**
     * Stops the motor
     */
    public void stop(){
        master.stopMotor();
        
    }
    
   
    /**
     * Set the setpoint of the flywheel based on an RPM target
     * @param RPM the RPM for the flywheel to spin at
     */
    public void setGoal(final double RPM){
        pidController.setReference(RPM, ControlType.kVelocity, ShooterConstants.kSlotID, FEEDFORWARD.calculate(RPM), CANPIDController.ArbFFUnits.kVoltage);
    }
}