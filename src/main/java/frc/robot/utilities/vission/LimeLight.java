package frc.robot.utilities.vission;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    public LimelightConfig config;
    private NetworkTable limelightTable;
    public static NetworkTableEntry tv;
    public static NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    public NetworkTableEntry ts;
    public NetworkTableEntry tl;
    public NetworkTableEntry tshort;
    public NetworkTableEntry tlong;
    public NetworkTableEntry thor;
    public NetworkTableEntry getpipe;
    public NetworkTableEntry camtran;
    public NetworkTableEntry tid;
    public NetworkTableEntry json;
    public NetworkTableEntry botpose;
    public NetworkTableEntry tclass;
    public NetworkTableEntry tc;
    public static NetworkTableEntry ledMode;
    public NetworkTableEntry camMode;
    public NetworkTableEntry pipeline;
    public NetworkTableEntry stream;
    public NetworkTableEntry snapshot;
    public NetworkTableEntry crop;
    public NetworkTableEntry tx0;
    public NetworkTableEntry ty0;
    public NetworkTableEntry ta0;
    public NetworkTableEntry ts0;
    public NetworkTableEntry tx1;
    public NetworkTableEntry ty1;
    public NetworkTableEntry ta1;
    public NetworkTableEntry ts1;
    public NetworkTableEntry tx2;
    public NetworkTableEntry ty2;
    public NetworkTableEntry ta2;
    public NetworkTableEntry ts2;
    public NetworkTableEntry cx0;
    public NetworkTableEntry cy0;
    public NetworkTableEntry cx1;
    public NetworkTableEntry cy1;

    public enum LedMode{
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);
        private int id;
        private LedMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum CameraMode{
        VISION_PROCESSOR(0),
        DRIVER(1);
        private int id;
        private CameraMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum StreamMode{
        STANDARD(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);
        private int id;
        private StreamMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum SnapshotMode{
        RESET(0),
        ONCE(1);
        private int id;
        private SnapshotMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public LimeLight(){
        this(LimelightConfig.defualt());
    }

    public LimeLight(LimelightConfig config){
        this.config = config;
        limelightTable = NetworkTableInstance.getDefault().getTable(config.table());
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        tl = limelightTable.getEntry("tl");
        tshort = limelightTable.getEntry("tshort");
        tlong = limelightTable.getEntry("tlong");
        thor = limelightTable.getEntry("thor");
        getpipe = limelightTable.getEntry("getpipe");
        camtran = limelightTable.getEntry("camtran");
        tid = limelightTable.getEntry("tid");
        json = limelightTable.getEntry("json");
        botpose = limelightTable.getEntry("botpose");
        tclass = limelightTable.getEntry("tclass");
        tc = limelightTable.getEntry("tc");
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        stream = limelightTable.getEntry("stream");
        snapshot = limelightTable.getEntry("snapshot");
        crop = limelightTable.getEntry("crop");
        tx0 = limelightTable.getEntry("tx0");
        ty0 = limelightTable.getEntry("ty0");
        ta0 = limelightTable.getEntry("ta0");
        ts0 = limelightTable.getEntry("ts0");
        tx1 = limelightTable.getEntry("tx1");
        ty1 = limelightTable.getEntry("ty1");
        ta1 = limelightTable.getEntry("ta1");
        ts1 = limelightTable.getEntry("ts1");
        tx2 = limelightTable.getEntry("tx2");
        ty2 = limelightTable.getEntry("ty2");
        ta2 = limelightTable.getEntry("ta2");
        ts2 = limelightTable.getEntry("ts2");
        cx0 = limelightTable.getEntry("cx0");
        cy0 = limelightTable.getEntry("cy0");
        cx1 = limelightTable.getEntry("cx1");
        cy1 = limelightTable.getEntry("cy1");
    }
    
    
    /**
     * Whether the limelight has any valid targets
     */
    public boolean hasValidTarget(){
        return tv.getDouble(0) == 1;
    }

    public boolean hasBotPose(){
        // return botpose.getNumberArray(null) != null;
        return false;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getTargetHorizontalOffset(){
        return tx.getDouble(0);
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getTargetVerticalOffset(){
        return ty.getDouble(0);
    }

     /**
     * Target Area (0% of image to 100% of image)
     */
    public double getTargetArea(){
        return ta.getDouble(0);
    }

     /**
     * Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew(){
        return ts.getDouble(0);
    }

     /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getLatency(){
        return tl.getDouble(0);
    }

     /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     */
    public double getTargetShortestSide(){
        return tshort.getDouble(0);
    }

     /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     */
    public double getTargetLongestSide(){
        return tlong.getDouble(0);
    }

     /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetHorizontalSide(){
        return thor.getDouble(0);
    }

     /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetVerticalSide(){
        return thor.getDouble(0);
    }

     /**
     * True active pipeline index of the camera (0 .. 9)
     */
    public double getPipeline(){
        return getpipe.getDouble(0);
    }

     /**
     * Camera transform in target space of primary apriltag or solvepnp target, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     */
    public Number[] getCameraTransform(){
        return camtran.getNumberArray(null);
    }

    /**
     * ID of primary AprilTag
     */
    public long getAprilTagID(){
        return tid.getInteger(0);
    }

     /**
     * Full JSON dump of targeting results
     */
    public String getJSON(){
        return json.getString("");
    }

     /**
     * Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
     */
    public Double[] getBotPositionRaw(){
        Double[] dub = {0.0,0.0,0.0,0.0,0.0,0.0};
        Double[] poseReal = botpose.getDoubleArray(dub);
        return poseReal;
    }

    /**
     * Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
     */
    public Pose3d getBot3DPosition(){
        Number[] pose = getBotPositionRaw();
        if(pose == null) return new Pose3d();
        Translation3d translation3d = new Translation3d(pose[0].doubleValue(), pose[1].doubleValue(), pose[2].doubleValue());
        Rotation3d rotation3d = new Rotation3d(pose[3].doubleValue(), pose[4].doubleValue(), pose[5].doubleValue());
        return new Pose3d(translation3d, rotation3d);
    }

    public Pose2d getBot2DPosition(){
        Double[] pose = getBotPositionRaw();
        if(pose == null) return new Pose2d();
        Translation2d translation = new Translation2d(pose[1], pose[0]);
        Rotation2d rotation = new Rotation2d(pose[5]);
        return new Pose2d(translation, rotation);
    }

    /**
     * Class ID of primary neural detector result or neural classifier result
     */
    public long getNeuralClassID(){
        return tid.getInteger(0);
    }

    /**
     *  Get the average HSV color underneath the crosshair region as a NumberArray
     */
    public Number[] getAverageHSV(){
        return tc.getNumberArray(null);
    }

    /**
     *  Sets limelight’s LED state
     */
    public static void setLedMode(LedMode mode){
        ledMode.setNumber(mode.get());
    }

    /**
     *  Sets limelight’s operation mode
     */
    public void setCameraMode(CameraMode mode){
        camMode.setNumber(mode.get());
    }

    /**
     *  Sets limelight’s streaming mode
     */
    public void setStream(StreamMode mode){
        stream.setNumber(mode.get());
    }

     /**
     *  Sets limelight’s current pipeline (0-9)
     */
    public void setStream(int mode){
        pipeline.setNumber(mode);
    }

    /**
     * Allows users to take snapshots during a match
     */
    public void setSnapshots(int mode){
        snapshot.setNumber(mode);
    }

    public double getDistanceFromTarget(double targetHeightMeters){
        return getDistanceFromTarget(config.mountingAngle(), config.mountingHeightMeters(), targetHeightMeters);
    }

    //Change back to vertical on good robot
    public double getDistanceFromTarget(double mountingAngle, double mountingHeightMeters, double targetHeightMeters){
        double angleToTargetRadains =  (mountingAngle + getTargetHorizontalOffset()) * (Math.PI/180d); 
        double heightDiff = targetHeightMeters - mountingHeightMeters;
        
        return heightDiff == 0 ? Math.tan(angleToTargetRadains) : (heightDiff)/Math.tan(angleToTargetRadains);
    }

}
