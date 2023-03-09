package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto3 extends SequentialCommandGroup {
//Limelight limelight;


    public Auto3(Swerve s_Swerve, Limelight limelight) {

        // List<PathPlannerTrajectory> Auto3 = PathPlanner.loadPathGroup("New Path", new PathConstraints(2, 2));
        List<PathPlannerTrajectory> Auto3 = PathPlanner.loadPathGroup("New Path", new PathConstraints(5, 5));
        //limelight = new Limelight(s_Swerve, Auto3.get(0).getInitialHolonomicPose());
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("Event1", null);

        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //         limelight::getPose,//s_Swerve::getPose,
        //         limelight::setPose, 
        //         Constants.Swerve.swerveKinematics,
        //         //new PIDConstants(2.8, 0.0, 0.0),
        //         new PIDConstants(2.5, 0.0, 0.0),
        //         new PIDConstants(2.5, 0.0, 0.0),
        //         s_Swerve::setModuleStates,
        //         eventMap,
        //         false,
        //         s_Swerve,limelight
        // );

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                limelight::getPose,//s_Swerve::getPose,
                limelight::setPose, 
                Constants.Swerve.swerveKinematics,
                //new PIDConstants(2.8, 0.0, 0.0),
                new PIDConstants(30, 0.0, 0.0),
                new PIDConstants(38, 0.0, 0.0),
                s_Swerve::setModuleStates,
                eventMap,
                false,
                s_Swerve,limelight
        );

        Command part1 = autoBuilder.fullAuto(Auto3);
        //Command part2 = autoBuilder.fullAuto(Auto3.get(1));
        SmartDashboard.putNumber("INIT_ROTATION", Auto3.get(0).getInitialHolonomicPose().getRotation().getDegrees());

        addCommands(
                new InstantCommand(() -> limelight.setPose(Auto3.get(0).getInitialHolonomicPose())),
                //new InstantCommand(() -> s_Swerve.gyro.setYaw(Auto3.get(0).getInitialHolonomicPose().getRotation().getDegrees())), 
                part1/*,
                new InstantCommand(() -> s_Swerve.resetOdometry(Auto3.get(1).getInitialPose())),
                part2*/
        );
}
}