package frc.robot.subsystems;

import frc.robot.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeConstants;

public class Intake implements Subsystem {
    
    private static final TalonSRX spinMotor = Util.createTalonSRX(IntakeConstants.spinMotor, false);
    private static final CANSparkMax conveyorMotor = Util.createSparkMAX(IntakeConstants.conveyorMotor, MotorType.kBrushless);
    
    private static Intake instance;
    public static Intake getInstance(){
        if (instance == null) instance = new Intake();
        return instance;
    }

    private Intake(){
        conveyorMotor.setInverted(false);
        conveyorMotor.burnFlash();
        
    }

    /**
     * Sets the conveyor to spin at a percent of max speed
     * @param value Percent speed
     */
    public void setConveyor(double value) {
        conveyorMotor.set(value);
    }

    /**
     * Sets the intake to spin at a given voltage
     * 
     * @param value Percent of maximum voltage to send to motor
     */
    public void intake(double value) {
        spinMotor.set(ControlMode.PercentOutput, value);
    }
    
    /**
     * Stops the intake
     */
    public void stopIntake() {
        spinMotor.set(ControlMode.PercentOutput, 0);
        conveyorMotor.set(0);
    }

    public void stopConveyor(){
        conveyorMotor.set(0);
    }
}