package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModule;
 
public class Drivetrain6390 extends SubsystemBase{

  private static SwerveModule[] swerveModules;
  private static Boolean isRed = true;
  private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab, autoTab;
  private static Field2d gameField;
  private static double desiredHeading;
  // public static ReplanningConfig c;
  private static PIDConfig driftCorrectionPID = new PIDConfig(0.1, 0,0).setContinuous(-Math.PI, Math.PI);
  //old value for p was 0.09, 0.05 is not good
  private static PID pid;
  private static PIDController rotationPidController = new PIDController(0.3, 0, 0);

  public Drivetrain6390()
  {
    
    // AutoBuilder.configureHolonomic
    // (
    //   this::getPose,
    //   this::resetOdometry,
    //   this::getSpeeds,
    //   this::drive,
    //   new HolonomicPathFollowerConfig(new PIDConstants(0.85), new PIDConstants(3.125), Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, Constants.DRIVETRAIN.SWERVE_MODULE_LOCATIONS[0].getNorm(), new ReplanningConfig()),
    //   this::getSide,
    //   this
    // );
  }
//5 and 10
  static {

    gameField = new Field2d();
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new
SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG, tab);
    swerveModules[1] = new
SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new
SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new
SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);
    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);

    pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions =
{swerveModules[0].getPostion(), swerveModules[1].getPostion(),
swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
    odometry = new SwerveDriveOdometry(kinematics,
Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()), SwervePositions);
    pose = new Pose2d();

    pid = new PID(driftCorrectionPID).setMeasurement(() ->
pose.getRotation().getDegrees());
}


  public void shuffleboard(){
//     tab.addDouble("Front Left Encoder", () ->
// swerveModules[0].getEncoderRadians());
//     tab.addDouble("Front Right Encoder", () ->
// swerveModules[1].getEncoderRadians());
//     tab.addDouble("Back Left Encoder", () ->
// swerveModules[2].getEncoderRadians());
//     tab.addDouble("Back Right Encoder", () ->
// swerveModules[3].getEncoderRadians());

//     autoTab.addDouble("Desired Heading", () ->
// pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
//     autoTab.addDouble("PID Desired Heading", () ->
// pid.calculate(pose.getRotation().getDegrees())).withWidget(BuiltInWidgets.kTextView);

//     autoTab.add(gameField);
//     autoTab.addDouble("Odometry Heading", () ->
// pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
//     autoTab.addDouble("Odometry X", () ->
// pose.getX()).withWidget(BuiltInWidgets.kTextView);
//     autoTab.addDouble("Odometry Y", () ->
// pose.getY()).withWidget(BuiltInWidgets.kTextView);


  }

  public void init(){
    pdh.clearStickyFaults();
    zeroHeading();
    resetOdometry(new Pose2d(0,0,getRotation2d()));
    //shuffleboard();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
    resetOdometry(pose);
  }

  public  void resetHeading(){
    gyro.setYaw(0);
  }

  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll().refresh().getValueAsDouble(), 360);
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch().refresh().getValueAsDouble(),360);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void driftCorrection(ChassisSpeeds speeds){
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) desiredHeading =
pose.getRotation().getDegrees();
    else speeds.omegaRadiansPerSecond += pid.calculate(desiredHeading);
  }

  public Boolean getSide()
  {
    return isRed; 
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
    //System.out.println(speeds);
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states,
SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
      //SmartDashboard.putNumber("Module " + i+"",swerveModules[i].getEncoderRadians());
    }
    
  }

  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new
SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
     // System.out.println("/////////////////////////////////////|||||||||||||||||||||||||");
    }
  }

  public void lockWheels(){

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
      
    }
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

  private void updateOdometry(){
    // if(limeLight.hasBotPose()){
      // if(limeLight.getPipeline() == 0){
      //   APRILTAGS tag = APRILTAGS.getByID((int)limeLight.getAprilTagID());
      //   if(!tag.equals(APRILTAGS.INVALID)){
      //     Pose2d relativePose =limeLight.getBot2DPosition();
      //     Pose2d tagPose = tag.getPose2d();
      //     pose = new Pose2d(relativePose.getX() + tagPose.getX(),
      //relativePose.getY() + tagPose.getY(), getRotation2d());
      //   }
      // }
    // }else{
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();
    // }

    gameField.setRobotPose(pose);
  }

  public ChassisSpeeds getSpeeds()
  {
    return chassisSpeeds;
  }

  public double maxAccel = 0;
  
  @Override
  public void periodic() {
  // SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
  // SmartDashboard.putNumber("Desired Heading", desiredHeading);
    double xSpeed = chassisSpeeds.vxMetersPerSecond +
feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond +
feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond +
feedbackSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    if(DriverStation.isTeleop()){
   //driftCorrection(speed);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    setModuleStates(states);

    updateOdometry();
    // SmartDashboard.putNumber("Odometry Headin", pose.getRotation().getDegrees());
    // SmartDashboard.putNumber("Odometry X", pose.getX());
    // SmartDashboard.putNumber("Odometry Y", pose.getY());

    
    if(gyro.getAccelerationX().getValueAsDouble() > maxAccel)
    {
      maxAccel = gyro.getAccelerationX().getValueAsDouble();
    }
   // System.out.println(getPose());
  }

  @Override
  public void simulationPeriodic() {

  }
  
}