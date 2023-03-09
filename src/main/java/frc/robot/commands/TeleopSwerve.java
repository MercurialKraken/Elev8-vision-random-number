package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier maxSpeed;
    private BooleanSupplier robotCentricSup;
    private final SlewRateLimiter transLimiter, strafeLimiter, turnLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier maxSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.maxSpeed = maxSpeed;
        this.transLimiter= new SlewRateLimiter(1.8);
        this.strafeLimiter= new SlewRateLimiter(1.8);
        this.turnLimiter=new SlewRateLimiter(1.5);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        /*double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);*/

        double translationVal = modifyAxis(translationSup.getAsDouble(), maxSpeed.getAsDouble(), transLimiter);
        double strafeVal = modifyAxis(strafeSup.getAsDouble(), maxSpeed.getAsDouble(), strafeLimiter);
        double rotationVal = modifyAxis(rotationSup.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    public double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter) {
        value = MathUtil.applyDeadband(value, Constants.stickDeadband);
        value = Math.copySign(value * value, value);
        value = value*speedModifyer;
        value = limiter.calculate(value);
        if(Math.abs(value)*Constants.Swerve.maxSpeed <= Constants.Swerve.maxSpeed*0.01){
          value = 0.0;
        }
    
        return value;
    }
}