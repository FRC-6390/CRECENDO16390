package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.Drivetrain6390;

public class Drive extends Command {

  //Creates a drivetrain subsystem
  private Drivetrain6390 driveTrain;
  //Double suppliers are outputted by the joystick
  private DoubleSupplier xInput, yInput, thetaInput;
  //These are limiters. The make sure the rate of change is never too abrupt and smooth out inputs from the joystick.
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  public Drive(Drivetrain6390 driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND);
    thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);

    //YOU MUST HAVE THIS - WONT WORK OTHERWISE
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.lockWheels();
  }

  @Override
  public void execute() {
  if(DriverStation.isTeleop()){
    //Take the inputs from the joystick, dampen them and then put it into variables
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

    //Store the individual speeds into a single class
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, -thetaSpeed, driveTrain.getRotation2d());

    //Feed that into the drive train subsystem

    driveTrain.drive(chassisSpeeds);}
    
  }

  @Override
  public void end(boolean interrupted) {
    //Shut it off
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}