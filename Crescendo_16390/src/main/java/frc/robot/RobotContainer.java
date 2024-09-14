package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightConfig;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.commands.Drive;
import frc.robot.commands.TurnToHead;

public class RobotContainer {

  public static LimeLight limeLight = new LimeLight();

  public static Drivetrain6390 driveTrain = new Drivetrain6390(limeLight);
  public static DebouncedController controller = new DebouncedController(0);
  public static Pose2d scoringPos = new Pose2d(1.24, 5.52, driveTrain.getRotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.19, 5.55, driveTrain.getRotation2d());

  public RobotContainer() 
  {
    driveTrain.init();
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));

    configureBindings();
  }
  

  private void configureBindings() 
  {
   controller.start.onTrue(new InstantCommand(driveTrain::zeroHeading));
   
   java.util.List<Translation2d> points = PathPlannerPath.bezierFromPoses(driveTrain.getVisionPose(), scoringPosR);
   PathPlannerPath path = new PathPlannerPath(points, new PathConstraints(1, 1,Units.degreesToRadians(540), Units.degreesToRadians(720)), new GoalEndState(0, new Rotation2d(180)));
  //  path.preventFlipping = true;
   controller.a.onTrue(AutoBuilder.followPath(path));

   
  //  Command pathFind; 
  //  scoringPos = new Pose2d(1.24, 5.52, new Rotation2d(0));
  //   scoringPosR = new Pose2d(15.19, 5.55, new Rotation2d(0));             
  //   if(!driveTrain.getSide())
  //   {
  //     pathFind = AutoBuilder.pathfindToPose(scoringPos, new PathConstraints(1, 1,Units.degreesToRadians(540), Units.degreesToRadians(90)));
  //   }
  //   else
  //   {
  //     pathFind = AutoBuilder.pathfindToPose(scoringPosR, new PathConstraints(1, 1,Units.degreesToRadians(540), Units.degreesToRadians(90)));
  //   }

    // controller.b.onTrue(pathFind);

    controller.y.onTrue
    (
      new ParallelCommandGroup(
        // new InstantCommand(driveTrain::resetHeading),
        new TurnToHead(driveTrain, 0),
        AutoBuilder.pathfindToPose
      (
        new Pose2d(15.19, 5.55, new Rotation2d(0)), 
        new PathConstraints(1, 1,Units.degreesToRadians(180), Units.degreesToRadians(0))
      )
      )
    );
  
  };
  
    
  



  public Command getAutonomousCommand(){
    // driveTrain.setHeading(180);
    return new PathPlannerAuto("New Auto");
  }

  
}