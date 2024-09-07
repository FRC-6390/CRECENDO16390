package frc.robot;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightConfig;
import frc.robot.commands.Drive;

public class RobotContainer {

  public static LimeLight limeLight = new LimeLight();
  public static LimeLight limeLight2 = new LimeLight(new LimelightConfig(null, 0, 0));

  public static Drivetrain6390 driveTrain = new Drivetrain6390();
  public static DebouncedController controller = new DebouncedController(0);


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
                        (new Pose2d(0,0,new Rotation2d(0)), 
                        new PathConstraints(4, 4, 3, 3), 
                        driveTrain::getPose, 
                        driveTrain::getSpeeds, 
                        driveTrain::drive, 
                        new HolonomicPathFollowerConfig(10, 10, new ReplanningConfig()), 
                        driveTrain));
  }
  
    
  



  public Command getAutonomousCommand(){
    return null;
  }

  
}