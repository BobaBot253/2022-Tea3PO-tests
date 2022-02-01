package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Util;
import frc.robot.Constants.ClimberConstants;

public class Climber implements Subsystem {
    Servo leftServo, rightServo;

    private static Climber instance;
    public static TalonSRX
         leftMotor = Util.createTalonSRX(ClimberConstants.leftMotorID, true),
         rightMotor = Util.createTalonSRX(ClimberConstants.rightMotorID, true); 

    private Climber(){
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        leftServo = new Servo(ClimberConstants.leftServoID);
        rightServo = new Servo(ClimberConstants.rightServoID);
    }

    public void leftServoUp() {
        leftServo.set(0.025);
    }

    public void rightServoUp() {
        rightServo.set(0.975);
    }

    public void leftServoDown() {
        leftServo.set(0);
    }

    public void rightServoDown() {
        rightServo.set(1);
    }
    
    public void climbRight(double rightSpeed){
        rightMotor.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void climbLeft(double leftSpeed){
        leftMotor.set(ControlMode.PercentOutput, leftSpeed);
    }

    public static Climber getInstance(){
        if(instance == null) {instance = new Climber();}
        return instance;
    }
    
    public void stopLeftMotor(){
        leftServoUp();
        leftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    public void stopRightMotor(){
        rightServoUp();
        rightMotor.set(ControlMode.PercentOutput, 0.0);
    }
}