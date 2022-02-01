package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.Climb;
import frc.robot.commands.ConveyorQueue;
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public class RobotContainer {
    private static RobotContainer instance;
    public static Drivetrain drivetrain;
    public static Intake intake;
    public static Conveyor conveyor;
    public static Shooter shooter;
    public static Climber climber;
    public static Arm arm;
    public boolean goShooter = false;
    private static final XboxController driver = new XboxController(Constants.InputPorts.driver_Controller);

    private static final XboxController operator = new XboxController(Constants.InputPorts.operator_Controller);

    private static final JoystickButton driver_A = new JoystickButton(driver, 1),
            driver_B = new JoystickButton(driver, 2), driver_X = new JoystickButton(driver, 3),
            driver_Y = new JoystickButton(driver, 4), driver_LB = new JoystickButton(driver, 5),
            driver_RB = new JoystickButton(driver, 6), driver_VIEW = new JoystickButton(driver, 7),
            driver_MENU = new JoystickButton(driver, 8);
    private static final JoystickButton operator_A = new JoystickButton(operator, 1),
            operator_B = new JoystickButton(operator, 2), operator_X = new JoystickButton(operator, 3),
            operator_Y = new JoystickButton(operator, 4), operator_LB = new JoystickButton(operator, 5),
            operator_RB = new JoystickButton(operator, 6), operator_VIEW = new JoystickButton(operator, 7),
            operator_MENU = new JoystickButton(operator, 8);
        
    private static final POVButton driver_DPAD_UP = new POVButton(driver, 0),
            driver_DPAD_RIGHT = new POVButton(driver, 90), driver_DPAD_DOWN = new POVButton(driver, 180),
            driver_DPAD_LEFT = new POVButton(driver, 270);

    private static final POVButton operator_DPAD_UP = new POVButton(operator, 0),
            operator_DPAD_RIGHT = new POVButton(driver, 90), operator_DPAD_DOWN = new POVButton(operator, 180),
            operator_DPAD_LEFT = new POVButton(driver, 270);
    private RobotContainer() {
        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDriveOpenLoop));
        intake = Intake.getInstance();

        conveyor = Conveyor.getInstance();
        conveyor.setDefaultCommand(new ConveyorQueue(ConveyorQueue.State.None));

        climber = Climber.getInstance();

        arm = Arm.getInstance();

        shooter = Shooter.getInstance();

        bindOI();
    }

    /**
     * Binds operator input to Commands 
     */
    private void bindOI() {
        
        // Flip down intake arm and spin when RB is held, flip back up and stop spinning when released
         driver_RB.whileHeld(new RunCommand(()->arm.rotate(-0.4), arm)
                     .alongWith(new RunCommand( ()->intake.intake(0.5)))
                     .alongWith(new RunCommand( ()->intake.setConveyor(0.5))))
                 .whenReleased(new RunCommand( ()->arm.rotate(0.35), arm)
                     .alongWith(new InstantCommand(intake::stopIntake)));
     
         // Spin up shooter when LB is held, stop when released
         driver_LB.whileHeld(new Shoot());
 
         // Flip intake down and spin outwards to sweep balls out of the way when A is held, flip up and stop when released
         driver_A.whileHeld(new RunCommand(()->arm.rotate(-0.4), arm)
                     .alongWith(new RunCommand( ()->intake.intake(-0.5))))
                 .whenReleased(new RunCommand( ()->arm.rotate(0.35), arm)
                     .alongWith(new InstantCommand(intake::stopIntake)));   
 
         // Run both climbers when DPAD up is held
         operator_DPAD_UP.whileHeld(new Climb(Climb.Side.BOTH, 0.5));
         operator_DPAD_DOWN.whileHeld(new Climb(Climb.Side.BOTH, -0.5));
 
         // Run the right and left climbers when view and menu are held, respectively
         operator_VIEW.whileHeld(new Climb(Climb.Side.LEFT, 0.5));
         operator_MENU.whileHeld(new Climb(Climb.Side.RIGHT, 0.5));
 
         // 2nd Controller vertical conveyor up and down respectively
         operator_Y.whileHeld(new RunCommand(()-> conveyor.setOpenLoop(0.55), conveyor))
                    .whenReleased(new InstantCommand(conveyor::stop, conveyor));
         operator_A.whileHeld(new RunCommand(()-> conveyor.setOpenLoop(-0.55), conveyor)) 
                    .whenReleased(new InstantCommand(conveyor::stop, conveyor));
 
         // 2nd controller horizontal conveyor in and out respectively
         operator_X.whileHeld(new RunCommand(()-> intake.setConveyor(0.5), intake))
                      .whenReleased(new InstantCommand(intake::stopIntake, intake));
         operator_B.whileHeld(new RunCommand(()-> intake.setConveyor(-0.5), intake))
         .whenReleased(new InstantCommand(intake::stopIntake, intake));
         
     }
 

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    /**
     * Returns the deadbanded throttle input from the main driver controller
     * 
     * @return the deadbanded throttle input from the main driver controller
     */
    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. This method
        // corrects that by returning the opposite of the y-value
        return -deadbandX(driver.getLeftY(), DriverConstants.kJoystickDeadband);
    }

    /**
     * Returns the deadbanded turn input from the main driver controller
     * 
     * @return the deadbanded turn input from the main driver controller
     */
    public static double getTurnValue() {
        return deadbandX(driver.getRightX(), DriverConstants.kJoystickDeadband);
    }

    /**
     * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
     * linear from (deadband, 0) to (1,1)
     * 
     * @param input    The input value to rescale
     * @param deadband The deadband
     * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
     */
    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

}
