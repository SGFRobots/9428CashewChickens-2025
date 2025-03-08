package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.cameraserver.CameraServer;

public class Limelight extends SubsystemBase{
    // Data values
    private int id;
    private double x;
    private double yaw;
    private double dist;

    // Input
    private NetworkTable table;
    private Rev2mDistanceSensor sensor;

    public Limelight() {
        // Set up limelight and sensor
        table = NetworkTableInstance.getDefault().getTable("limelight-scoring");  
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setAutomaticMode(true);
        sensor.setEnabled(true);
        id = 0;
        x = 0;
        yaw = 0;
        dist = -1;
    }
    
    public void update(){
        // Get data
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry tid = table.getEntry("tid");
        double[] targatePose_cameraSpace = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        x = tx.getDouble(0.0);
        id = (int) tid.getInteger(0);
        yaw = targatePose_cameraSpace[4];
        dist = sensor.getRange(Unit.kMillimeters) / 1000;
    }

    public void displayData() {
        // Post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightID", id);
        SmartDashboard.putNumber("LimelightYaw", yaw);
        SmartDashboard.putNumber("Distance", dist);
        SmartDashboard.putBoolean("distanceEnabled", sensor.isEnabled());
        SmartDashboard.putBoolean("isAligned", isAligned());

        // VideoSource camera = new VideoSource(5);
                
        
    }

    // Get ID value from Limelight
    public int getID(){
        return id;
    }

    // get X value from Limelight
    public double getX(){
        return x;
    }
    
    // Get Yaw value from Limelight
    public double getYaw() {
        return yaw;
    }

    // Get distance from distance sensor
    public double getDistance() {
        return dist;
    }

    // check whether the robot is aligned to either scoring side
    public boolean isAligned() {
        double xError = Constants.AprilTags.xErrorAllowed;
        double distError = Constants.AprilTags.distanceErrorAllowed;
        double yawError = Constants.AprilTags.yawErrorAllowed;
        
        boolean alignedLeft = (Math.abs(getX() - Constants.AprilTags.leftCoral[0]) < xError) && (Math.abs(Constants.AprilTags.leftCoral[2] - getYaw()) < yawError) && (Math.abs(Constants.AprilTags.leftCoral[1] - getDistance()) < distError);
        boolean alignedRight = (Math.abs(getX() - Constants.AprilTags.rightCoral[0]) < xError) && (Math.abs(Constants.AprilTags.rightCoral[2] - getYaw()) < yawError) && (Math.abs(Constants.AprilTags.rightCoral[1] - getDistance()) < distError);
        
        return alignedLeft || alignedRight;
    }
}
