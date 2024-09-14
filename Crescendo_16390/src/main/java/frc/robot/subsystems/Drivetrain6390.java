package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
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
// import frc.robot.utilities.telemetry.SwerveTelemetry;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightHelpers;
 
public class Drivetrain6390 extends SubsystemBase{

  private static SwerveModule[] swerveModules;
  private static Boolean isRed =true;
  private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  // private static SwerveTelemetry tele;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
  private static Field2d gameField;
  private static Field2d gameFieldVision;
  private static Field2d gameFieldVision2;
  private static double desiredHeading;
  public static ReplanningConfig c;
  //0.1
  private static PIDConfig driftCorrectionPID = new PIDConfig(5, 0,0).setContinuous(-Math.PI, Math.PI);
  private static Pose2d visionPose;
  private static PID pid;
  public LimeLight limeLight;

  // public static Orchestra orchestra = new Orchestra();
  
  public SwerveDrivePoseEstimator estimator = 
  new SwerveDrivePoseEstimator(
    kinematics, 
    getRotation2d(), 
    getModulePostions(), 
    new Pose2d(), 
    VecBuilder.fill(0.1,0.1,Units.degreesToRadians(10)), 
    VecBuilder.fill(5,5,Units.degreesToRadians(500)));

  public Drivetrain6390(LimeLight limelight)
  {
    this.limeLight = limelight;
    AutoBuilder.configureHolonomic
    (
      this::getVisionPose,
      this::resetOdometryVision,
      this::getSpeeds,
      this::drive,
      //0.85 translation 3.125 rotation
      new HolonomicPathFollowerConfig(new PIDConstants(0.5), new PIDConstants(0.0000000000000001), Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, Constants.DRIVETRAIN.SWERVE_MODULE_LOCATIONS[0].getNorm(), new ReplanningConfig()),
      this::getSide,
      this
    );
    
  
  }

  static {
  
    gameField = new Field2d();
    gameFieldVision = new Field2d();
    gameFieldVision2 = new Field2d();
    
   
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
    visionPose = new Pose2d();
    

    pid = new PID(driftCorrectionPID).setMeasurement(() ->
    pose.getRotation().getDegrees());
    // tele = new SwerveTelemetry(swerveModules[0], swerveModules[1], swerveModules[2], swerveModules[3], pid, odometry, gameField, tab);
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
  public  void setHeading(double value){
    gyro.setYaw(value);
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

  public Pose2d getVisionPose(){
   return visionPose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void zeroOdometry()
  {
    resetHeading();
    odometry.resetPosition(getRotation2d(), getModulePostions(), new Pose2d(0,0, new Rotation2d(0)));
  }
  public void resetOdometryVision(Pose2d pose)
  {
    estimator.resetPosition(getRotation2d(), getModulePostions(), pose);
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
  LimeLight limelight = new LimeLight();
  private void updateOdometry(){
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();

      boolean doRejectUpdate = false;
      estimator.update(getRotation2d(), getModulePostions());
      
      //MY VERSION
      Pose2d roboPos = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
      int tagCount = LimelightHelpers.getTargetCount("limelight");

      Pose2d roboPos2 = limelight.getBot2DPositionM2();

      System.out.println(roboPos);
      System.out.println(roboPos2);
      if(Math.abs(gyro.getRate()) > 720) 
      {
        doRejectUpdate = true;
      }
      if(tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        estimator.addVisionMeasurement(
            roboPos,  edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
      }

      //MEGATAG LOCALIZATION W/ LIMELIGHT HELPERS
    //   boolean useMegaTag2 = true; //set to false to use MegaTag1
    //   boolean doRejectUpdate = false;
    //   if(useMegaTag2 == false)
    //   {
        // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
    //     if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
    //     {
    //       if(mt1.rawFiducials[0].ambiguity > .7)
    //       {
    //        doRejectUpdate = true;
    //       }
    //       if(mt1.rawFiducials[0].distToCamera > 3)
    //       {
    //         doRejectUpdate = true;
    //       }
    //     }
    //     if(mt1.tagCount == 0)
    //     {
    //       doRejectUpdate = true;
    //     }

    //     if(!doRejectUpdate)
    //     {
    //       estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    //       estimator.addVisionMeasurement(
    //           mt1.pose,
    //           mt1.timestampSeconds);
    //     }
    // }
    // else if (useMegaTag2 == true)
    // {
    //   LimelightHelpers.SetRobotOrientation("limelight", estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //   LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //   if(Math.abs(gyro.getRate()) > 720) 
    //   {
    //     doRejectUpdate = true;
    //   }
    //   if(mt2.tagCount == 0)
    //   {
    //     doRejectUpdate = true;
    //   }
    //   if(!doRejectUpdate)
    //   {
    //     estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //     estimator.addVisionMeasurement(
    //         mt2.pose,
    //         mt2.timestampSeconds);
    //   }
    // }
    gameField.setRobotPose(pose);
    gameFieldVision.setRobotPose(estimator.getEstimatedPosition());
    gameFieldVision2.setRobotPose(roboPos);
    visionPose = estimator.getEstimatedPosition();
    
  }

  public ChassisSpeeds getSpeeds()
  {
    return chassisSpeeds;
  }

  public double maxAccel = 0;
  
  @Override
  public void periodic() {
    double xSpeed = chassisSpeeds.vxMetersPerSecond +
feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond +
feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond +
feedbackSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    setModuleStates(states);

    updateOdometry();

    // if(DriverStation.isTeleop())
    // {
    // driftCorrection(speed);
    // }

    SmartDashboard.putNumber("Odometry Headin", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());

   SmartDashboard.putData("Vision Pose", gameFieldVision);
   SmartDashboard.putData("Vision 2 Pose", gameFieldVision2);
   SmartDashboard.putNumber("Rotor Position 1", swerveModules[0].getDriveMotorPosition());
   SmartDashboard.putNumber("Rotor Position 2", swerveModules[1].getDriveMotorPosition());
   SmartDashboard.putNumber("Rotor Position 3", swerveModules[2].getDriveMotorPosition());
   SmartDashboard.putNumber("Rotor Position 4", swerveModules[3].getDriveMotorPosition());


   SmartDashboard.putData("Pose", gameField);
    
    if(gyro.getAccelerationX().getValueAsDouble() > maxAccel)
    {
      maxAccel = gyro.getAccelerationX().getValueAsDouble();
    }
    // tele.updateShuffleboard();
  }

  @Override
  public void simulationPeriodic() {

  }
  
}