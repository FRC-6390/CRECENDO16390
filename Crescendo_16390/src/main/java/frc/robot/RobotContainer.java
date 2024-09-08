package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightConfig;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.commands.Drive;

public class RobotContainer {

  public static LimeLight limeLight = new LimeLight();
  public static LimeLight limeLight2 = new LimeLight(new LimelightConfig(null, 0, 0));

  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static DebouncedController controller = new DebouncedController(0);

  public static Pose2d scoringPos = new Pose2d(1.24, 5.52, new Rotation2d());
  public static Pose2d scoringPosR = new Pose2d(15.26, 5.52, new Rotation2d());


  public RobotContainer() 
  {
    driveTrain.init();
    driveTrain.setDefaultCommand(new Drive(driveTrain, controller.leftX, controller.leftY, controller.rightX));

    configureBindings();
  }
  

  private void configureBindings() 
  {
   controller.start.onTrue(new InstantCommand(driveTrain::zeroHeading));
   controller.a.onTrue(new PathfindHolonomic
                        (new Pose2d(1.24, 5.52, new Rotation2d(3.142)), 
                        new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(720)), 
                        driveTrain::getPose, 
                        driveTrain::getSpeeds, 
                        driveTrain::drive, 
                        new HolonomicPathFollowerConfig(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, 1, new ReplanningConfig()), 
                        driveTrain));
   Command pathFind; 
                        
    if(!driveTrain.getSide())
    {
      pathFind = AutoBuilder.pathfindToPose(scoringPos, new PathConstraints(4, 4,Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }
    else
    {
      pathFind = AutoBuilder.pathfindToPose(scoringPosR, new PathConstraints(4, 4,Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }

    controller.b.onTrue(pathFind);

    controller.y.onTrue
    (
      AutoBuilder.pathfindToPose
      (
        new Pose2d(1.24, 5.52, new Rotation2d(3.142)), 
        new PathConstraints(4, 4,Units.degreesToRadians(540), Units.degreesToRadians(720))
      )
    );
  }
  
    
   public Command getAutonomousCommand(){
    return new PathPlannerAuto("New Auto");
  }

  
}