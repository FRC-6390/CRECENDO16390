package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Drivetrain6390;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightConfig;
import frc.robot.commands.Drive;

public class RobotContainer {

  public static LimeLight limeLight = new LimeLight();

  public static Drivetrain6390 driveTrain = new Drivetrain6390(limeLight);
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
  }
  
    
  



  public Command getAutonomousCommand(){
    return null;
  }

  
}