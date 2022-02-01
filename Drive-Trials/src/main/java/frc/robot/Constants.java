package frc.robot;

public class Constants {
    
    public static final double dt = 0.02;
    public static final double kMaxVoltage = 12.0;

    /* Controllers Input */
    public static class InputPorts {
        public static final int driver_Controller = 0;
        public static final int operator_Controller = 1;
    }

    /* Differential Drive Settings */
    public static class DriverConstants {
        /* Common drive mode settings */
        public static final double kJoystickDeadband = 0.07; // How much of joystick is "dead" zone [0,1]
        public static final double kDriveSens = 1.0; // Overall speed setting (turn down for demos) [0,1]
        public static final double kTurnInPlaceSens = 0.5; // Maximum turn-in-place rate (in percent of max) to allow
                                                            // robot to turn to [0,1]
        public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to
                                                  // turn to [0,1]

    }
    // TODO: Tune
    public static class DrivetrainConstants {
        public static final int
        /* Drivetrain motor IDs */
            leftMaster = 0, // TalonFX
            leftSlave = 1, // TalonFX
            rightMaster = 2, // TalonFX
            rightSlave = 3; // TalonFX

        /* feedforward constants */
        public static final double kS = 0.364; // voltage required to overcome friction (V)
        public static final double kV = 2.34; // voltage over velocity (V/(meters/second))
        public static final double kA = 0.0824; // voltage over acceleration (V(meters/second/second))

        /* PID constants */
        public static final double kP = 2.99;
        public static final double kI = 0;
        public static final double kD = 0;

        /* Wheels Constants */
        public static final double kTicksPerRotation = 2048 * 10.42; // Falcon 500 integrated encoder (2048 CPR)
                                                                     // multiplied by gear ratio (10.42:1)
        public static final double kWheelDiameter = Units.InchesToMeters(6);

        public static final double kMaxSpeedMPS = 4.972; // max speed in meters per second
        public static final double kTrackWidth = 0.7051868402911773; // distance between wheels
        public static final double kMaxCurvature = Math.toRadians(-162); // Maximum turn rate in radians per meter

        public static final double kRamseteBeta = 2.0;
        public static final double kRamseteZeta = 0.7;

        public static final boolean kStatorLimitEnable = false;
        public static final double kStatorCurrentLimit = 100;
        public static final double kStatorTriggerThreshold = 100;
        public static final double kStatorTriggerDuration = 0;

        public static final boolean kSupplyLimitEnable = false;
        public static final double kSupplyCurrentLimit = 70;
        public static final double kSupplyTriggerThreshold = 70;
        public static final double kSupplyTriggerDuration = 0.7;
    }

    public static class IntakeConstants {
        /* Motors */
        public static final int spinMotor = 6;
        public static final int conveyorMotor = 11;

    }

    //TODO: Tune
    public static class ArmConstants {
        public static final int armMotor = 5;

        /* PID Constants */
        public static double kP = 2.9;
        public static double kI = 0;
        public static double kD = 0;

        /* Feedforward Constants */
        public static double kS = 0.402;
        public static double kCos = 0.771;
        public static double kV = 0.758;
        public static double kA = 0.00717;

        /* Intake constants */
        public static double kMaxVelocity = 0.25; // Maximum velocity to turn arm at, radians per second
        public static double kMaxAcceleration = 2; // Maximum acceleration to turn arm at, radians per second per second
        public static double kArmOffset = 4.22; // Initial position of the intake arm

    }

    // TODO: Tune
    public static class ConveyorConstants {
        public static final int motor = 10;
        public static final double kQueueSpeed = 0.2;
    }
    // TODO: Tune
    public static class ShooterConstants {
        public static final int master = 8;
        public static final int slave = 9;

        /* PIDController Constants, Slot 0, RPM Velocity Control */ 
        public static int kSlotID = 0;
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kMin = -0.7;
        public static double kMax = 0.7;

        /* Feedforward Constants */ 
        public static double kS = 0;
        public static double kV = 0;
        public static double kA = 0;

        /* Shooter Constants */ 
        public static double kTolerance = 0;
    }
    // TODO: Tune
    public static class VisionConstants {
        /* Turn PID Constants */
        public static double kPTurn = 0;
        public static double kITurn = 0;
        public static double kDTurn = 0;
        public static double kTurnTolerance = 0;

        /* Distance PID Constants */
        public static double kPDist = 0;
        public static double kIDist = 0;
        public static double kDDist = 0;
        public static double kDistTolerance = 0;
    }

    public static class ClimberConstants{
        public static final int leftMotorID = 4;
        public static final int rightMotorID = 7;
        public static final int leftServoID = 8;
        public static final int rightServoID = 9;
    }
}
