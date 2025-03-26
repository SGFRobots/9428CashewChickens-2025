package frc.robot.commands.Limelight;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;

public class AprilTagAlign extends Command {
    private final SwerveSubsystem mSubsystem;
    private final Limelight mLimelight;
    private double x;
    private double yaw;
    private double distance;
    private double targetX;
    private double targetDistance;
    private double targetYaw;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private double xErrorAllowed;
    private double distanceErrorAllowed;
    private double yawErrorAllowed;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController turnPID;

    public AprilTagAlign(SwerveSubsystem pSubsystem, Limelight pLimelight, double pTargetX, double pTargetDistance, double pTargetYaw) {
        // Set up subsystems
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;

        // Set up speed and deadzone
        xErrorAllowed = Constants.AprilTags.xErrorAllowed;
        distanceErrorAllowed = Constants.AprilTags.distanceErrorAllowed;
        yawErrorAllowed = Constants.AprilTags.yawErrorAllowed;

        // Target position
        targetX = pTargetX;
        targetDistance = pTargetDistance;
        targetYaw = pTargetYaw;
    }

    @Override
    public void initialize() {
        mSubsystem.toggleFindingPos();
        // if (Robot.stage.equals("teleOp")){
            xPID = new PIDController(0.007, 0, 0.0001);
            yPID = new PIDController(0.1005, 0, 0.0001);
            turnPID = new PIDController(0.008, 0, 0);
            // System.out.println("teleOp");
        // }
        // else{
        //     xPID = new PIDController(0.026, 0, 0.00015);
        //     yPID = new PIDController(0.3015, 0, 0.0001);
        //     turnPID = new PIDController(0.015, 0, 0);
        //     System.out.println("auto");
        // }
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        // Get data from limelight
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        distance = mLimelight.getDistance();

        // Calculate Speeds
        xSpeed = xPID.calculate(x, targetX);
        ySpeed = yPID.calculate(distance, targetDistance);
        turningSpeed = turnPID.calculate(yaw, targetYaw);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("xPID", xSpeed);
        SmartDashboard.putNumber("yawPID", turningSpeed);
        SmartDashboard.putNumber("distPID",ySpeed);
        

        // Drive the robot
        chassisSpeeds = new ChassisSpeeds(-ySpeed,xSpeed,turningSpeed);
        mSubsystem.drive(chassisSpeeds);
        
        // Update SmartDashboard
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turningSpeed", turningSpeed);
    }

    @Override
    public void end(boolean isFinished) {
        mSubsystem.toggleFindingPos();
        mSubsystem.stopModules();
        System.out.println("Done Aligning");
    }

    @Override
    public boolean isFinished() {
        // End method when aligned
        if (((Math.abs(mLimelight.getX() - targetX) < xErrorAllowed) && (Math.abs(targetYaw - mLimelight.getYaw()) < yawErrorAllowed) && (Math.abs(targetDistance - mLimelight.getDistance()) < distanceErrorAllowed)) || (Math.abs(xSpeed) < 0.02 && Math.abs(ySpeed) < 0.02) && Math.abs(turningSpeed) < 0.02) {
            return true;
        }
        return false;
    }

    public double[] getTargets() {
        return new double[]{targetX, targetDistance, targetYaw};
    }
}
