package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.controlloop.PIDConfig;

import frc.robot.utilities.swerve.SwerveModuleConfig;

public interface Constants {

    public interface ARM {
        int ARM_MOTOR = 18;
        int ARM_ENCODER = 19;
        int OUTPUT_ROLLER = 17;

        
        int SETPOINT_SCORE = 154;
        int SETPOINT_SHELF_1 = 135;
        int SETPOINT_SHELF_2 = 181;
        int SETPOINT_CUBE = 200;
        int SETPOINT_DOWN = 0;
    }

    public interface GRIPPER {
        int SIDE_GRIPPER = 21;
    }

    public interface WASHING_MACHINE {
        int MOTOR_ID = 15;
    }

    public interface ELEVATOR {
        int DRIVE_MOTOR = 16;
        int ENCODER = 20;
        double GEARBOX_RATIO = 0;
        int LIMIT_SWITCH = 1;

        //encoder setpoint values
        double SETPOINT_HIGH = 5150;
        double SETPOINT_SHELF = 3000;
        double SETPOINT_MID = 1590; //1790
        double SETPOINT_LOW = 0;
        double SETPOINT_DOWN = 0;
    }
    

    public interface INTAKE {
        int ARM = 14;
        int INTAKE_MOTOR = 23;
        int POSITION_ENCODER = 13;
        int LIMIT_SWITCH = 0;
        double ARM_GEARBOX_RATIO = 0;
        double GEARBOX_RATIO = 1d/10d;
        PIDConfig pidConfig = new PIDConfig(0, 0, 0);
 //       MotionProfileConfig config = new MotionProfileConfig(0.1, 0.04, pidConfig);

        int SETPOINT_DOWN = -90;
        double SETPOINT_UP = 0.5;
    }

    public interface DRIVETRAIN{

        String CANBUS = "can";

        int PIGEON = 0;

        int REV_PDH = 1;

        Translation2d[] SWERVE_MODULE_LOCATIONS = {ROBOT.FRONT_LEFT, ROBOT.FRONT_RIGHT, ROBOT.BACK_LEFT, ROBOT.BACK_RIGHT};

        int FRONT_LEFT_DRIVE = 1;
        int FRONT_LEFT_ROTATION = 5;
        int FRONT_LEFT_ENCODER = 9;
        int FRONT_RIGHT_DRIVE = 2;
        int FRONT_RIGHT_ROTATION = 6;
        int FRONT_RIGHT_ENCODER = 10;
        int BACK_LEFT_DRIVE = 3;
        int BACK_LEFT_ROTATION = 7;
        int BACK_LEFT_ENCODER = 11;
        int BACK_RIGHT_DRIVE = 4;
        int BACK_RIGHT_ROTATION = 8;
        int BACK_RIGHT_ENCODER = 12;

        //original offsets
        // double FRONT_LEFT_OFFSET = 0.013805+Math.PI;
        // double FRONT_RIGHT_OFFSET = 0.010737+Math.PI;
        // double BACK_LEFT_OFFSET = 0.030679+Math.PI;
        // double BACK_RIGHT_OFFSET = -0.042951+Math.PI;

        double FRONT_LEFT_OFFSET =  -0.111328125;
        double FRONT_RIGHT_OFFSET = -0.494873046875;
        double BACK_LEFT_OFFSET =  -0.389892578125;
        double BACK_RIGHT_OFFSET = 0.138671875;

        SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_LEFT_DRIVE, false, FRONT_LEFT_ROTATION, false, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_RIGHT_DRIVE, false, FRONT_RIGHT_ROTATION, false, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(BACK_LEFT_DRIVE, false, BACK_LEFT_ROTATION, false, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(BACK_RIGHT_DRIVE, false, BACK_RIGHT_ROTATION, false, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET, CANBUS);

    }

    public interface SWERVEMODULE {
        double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_SPEED_METERS_PER_SECOND_SQUARED = Units.feetToMeters(13.5) * Units.feetToMeters(13.5);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_ACCELERATION_METERS_PER_SECOND = 2.75;
        double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND = 3.85;
        double ROTATION_GEAR_RATIO = 1d/(150d/7d);
        double DRIVE_GEAR_RATIO = 1d/(8.14);
        double ROTATION_ENCODER_CONVERSION_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;
        double ROTATION_ENCODER_CONVERSION_RADIANS_PER_SECOND = ROTATION_ENCODER_CONVERSION_RADIANS / 60;
        double DRIVE_ENCODER_CONVERSION_METERS = (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS) * 0.59; //the 0.625 is a quick fix to correct the odometry
        double DRIVE_ENCODER_CONVERSION_METERS_PER_SECOND = DRIVE_ENCODER_CONVERSION_METERS / 60;
        PIDConfig ROTATION_PID = new PIDConfig(0.5, 0, 0).setContinuous(-Math.PI, Math.PI);
    }

    public interface AUTO{
        
        PIDConfig XY_PID_CONFIG = new PIDConfig(0.5, 0, 0);
        PIDConfig THETA_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ROLL_PITCH_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);
        PIDConfig ALIGN_XY_PID_CONFIG = new PIDConfig(1, 0, 0);
        PIDConfig ALIGN_THETA_PID_CONFIG = new PIDConfig(0.1, 0, 0).setContinuous(-Math.PI, Math.PI);

      

        TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    }
    
    public interface ROBOT {
        double TRACKWIDTH_METERS = 0.61;
        double WHEELBASE_METERS = 0.61;

        int CANDLE_ID = 22; //unkown tbd

        Translation2d FRONT_LEFT = new Translation2d(TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d FRONT_RIGHT = new Translation2d(TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        Translation2d BACK_LEFT = new Translation2d(-TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d BACK_RIGHT = new Translation2d(-TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        int BLINKIN_PORT = 1;
        
 //       LimelightConfig LIMELIGHT_CONFIG = new LimelightConfig(0, 0);
        
    }
    
    public enum APRILTAGS {
    
        INVALID(0, new Pose3d(0,0,0, new Rotation3d())),
        //Z is to the carpet to center of the tag
        RED_ALLIANCE_RIGHT(1, new Pose3d(15.57,0.985,0.436, new Rotation3d())),
        RED_ALLIANCE_MIDDLE(2, new Pose3d(15.57,2.69,0.436, new Rotation3d())),
        RED_ALLIANCE_LEFT(3, new Pose3d(15.57,4.37,0.436, new Rotation3d())),

        BLUE_ALLIANCE_LOADING(4, new Pose3d(0.36,6.67,0.666, new Rotation3d())),
        RED_ALLIANCE_LOADING(5, new Pose3d(16.18,6.67,0.666, new Rotation3d())),

        BLUE_ALLIANCE_LEFT(6, new Pose3d(0.97,4.37,0.436, new Rotation3d())),
        BLUE_ALLIANCE_MIDDLE(7, new Pose3d(0.97,2.69,0.436, new Rotation3d())),
        BLUE_ALLIANCE_RIGHT(8, new Pose3d(0.97,0.985,0.436, new Rotation3d()));    

        // Blue right X: 138cm - 41cm = 0.97m Y: 65cm+23.5cm + 5cm + 5cm = 0.985m
        // Blue middle X: 138cm - 41cm = 0.97m Y: 191cm + 2.5cm+ 47cm+ + 5cm + 23.5cm = 2.69m
        // Blue left X: 138cm - 41cm = 0.97m Y: 191cm + 168cm + 2.5cm + 47cm + 5cm + 23.5cm = 4.37m
        
        // Red loading X: 36cm = 0.36m Y: 550cm + 117cm = 6.67m
        // Blue loading X: 1654cm - 36cm = 16.18m Y: 550cm + 117cm = 6.67m
        
        // Red right X: 1654cm - 138cm + 41cm = 15.57m Y: 65cm+23.5cm + 5cm + 5cm = 0.985m
        // Red middle X: 1654cm - 138cm + 41cm = 15.57m Y: 191cm + 2.5cm+ 47cm+ + 5cm + 23.5cm = 2.69m
        // Red left X: 1654cm - 138cm + 41cm = 15.57m Y: 191cm + 168cm + 2.5cm + 47cm + 5cm + 23.5cm = 4.37m

        int id;
        Pose3d pose3d;
        private APRILTAGS(int id, Pose3d pose3d){
            this.id = id;
            this.pose3d = pose3d;
            
        }
       
        public static APRILTAGS getByID(int id){
            for (APRILTAGS tag : values()) {
                if(tag.getID() == id) return tag;
            }
            return INVALID;
        }

        public int getID(){
            return id;
        }

        public double getHeight(){
            return pose3d.getZ();
        }
        
        public Pose2d getPose2d(){
            return pose3d.toPose2d();
        }

    }
}
