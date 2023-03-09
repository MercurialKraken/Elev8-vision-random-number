package frc.robot;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /*private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;*/

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton getToPos = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton boostButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton precisionButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d2pButton = new JoystickButton(driver, XboxController.Button.kA.value);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight limelight = new Limelight(s_Swerve, new Pose2d());

    public static double maxSpeed = Constants.driveSpeed;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                ()->maxSpeed
            )
        );
        //SmartDashboard.putNumber("speed",speed);


        // Configure the button bindings
        configureButtonBindings();
    }

    //y = 1.05, x=14.21, rad = 0

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        getToPos.onTrue(new InstantCommand(()->s_Swerve.drive(new Translation2d(14.21, 4.05), 0, true, false)));
        boostButton.whileTrue(new ChangeMaxSpeed(Constants.boostSpeed));
        precisionButton.whileTrue(new ChangeMaxSpeed(Constants.precisionSpeed));
        //d2pButton.onTrue(new Drive2Pos(s_Swerve, vision, limelight, new Pose2d(new Translation2d(13, 1.34), new Rotation2d(Math.toRadians(0)))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        //return new InstantCommand(()->vision.dashboard());
        return new Auto3(s_Swerve, limelight);
    }
}
