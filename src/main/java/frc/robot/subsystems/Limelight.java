package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class Limelight extends SubsystemBase{
    private int id;
    private double x;
    private double y;
    private double area;
    private double yaw;
    private double hi;
    private NetworkTable table;
    private Rev2mDistanceSensor sensor;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight-scoring");  
        id = 0;
        x = 0;
        y = 0;
        yaw = 0;
        area = 0;  
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setDistanceUnits(Unit.kMillimeters);
        sensor.setAutomaticMode(true);
        sensor.setEnabled(true);
    }
    
    public void displayData(){
        // System.out.println("Network Table Initialized: " + (table != null));
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");
        double[] targatePose_cameraSpace = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        id = (int) tid.getInteger(0);
        yaw = targatePose_cameraSpace[4];
        // System.out.println(yaw);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightID", id);
        SmartDashboard.putNumber("LimelightYaw", yaw);
        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putBoolean("distanceenable", sensor.isEnabled());
        // SmartDashboard.putData("hi", table.getEntry("targetpose_cameraspace"));


    }

    public int getID(){
        return id;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getA(){
        return area;
    }
    
    public double getYaw() {
        return yaw;
    }

    public double getDistance() {
        return sensor.getRange(Unit.kMillimeters)/1000;
    }
}
