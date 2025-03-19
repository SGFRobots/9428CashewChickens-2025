package frc.robot.commands.Limelight;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;

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
    private double driveSpeed;
    private double xErrorAllowed;
    private double distanceErrorAllowed;
    private double yawErrorAllowed;
    private double speedDeadzone;
    private double smallSpeed;
    private PIDController speedPID;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController turnPID;

    public AprilTagAlign(SwerveSubsystem pSubsystem, Limelight pLimelight, double pTargetX, double pTargetDistance, double pTargetYaw) {
        // Set up subsystems
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;

        // Set up speed and deadzone
        smallSpeed = 0.75 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        xErrorAllowed = Constants.AprilTags.xErrorAllowed;
        distanceErrorAllowed = Constants.AprilTags.distanceErrorAllowed;
        yawErrorAllowed = Constants.AprilTags.yawErrorAllowed;
        speedDeadzone = 0.05;

        // Target position
        targetX = pTargetX;
        targetDistance = pTargetDistance;
        targetYaw = pTargetYaw;

        // Speed PID
        // speedPID = new PIDController(0.005, 0.0002, 0.0000005);
        speedPID = new PIDController(0.0085, 0, 0);
        xPID = new PIDController(0.005, 0, 0.000075);
        yPID = new PIDController(0.04, 0, 0);
        turnPID = new PIDController(0.008, 0, 0);
    }

    @Override
    public void initialize() {
        mSubsystem.toggleFindingPos();
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        // Get data from limelight
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        distance = mLimelight.getDistance();
        
        // xSpeed = speedPID.calculate(x, targetX);
        // ySpeed = speedPID.calculate(distance, targetDistance);
        // turningSpeed = speedPID.calculate(yaw, targetYaw);

        
        // if (Math.abs(xSpeed) < speedDeadzone){
        //     xSpeed = (xSpeed > 0) ? smallSpeed : -smallSpeed;
        // }
        
        // if (Math.abs(ySpeed) < speedDeadzone){
        //     ySpeed = (ySpeed > 0) ? smallSpeed : -smallSpeed;
        // }
        
        // if (Math.abs(turningSpeed) < speedDeadzone){
        //     turningSpeed = (turningSpeed > 0) ? smallSpeed : -smallSpeed;
        // }
                    
        //             if (Math.abs(mLimelight.getX() - targetX) < xErrorAllowed) {
        //                 xSpeed = 0;
        //             }
            
        //             if (Math.abs(targetDistance - mLimelight.getDistance()) < distanceErrorAllowed) {
        //                 ySpeed = 0;
        //             }
            
        //             if (Math.abs(targetYaw - mLimelight.getYaw()) < yawErrorAllowed) {
        //                 turningSpeed = 0;
        //             }




        
        xSpeed = xPID.calculate(x, targetX);
        ySpeed = yPID.calculate(distance, targetDistance);
        turningSpeed = turnPID.calculate(yaw, targetYaw);
        
        SmartDashboard.putNumber("xPID", xSpeed);
        SmartDashboard.putNumber("yawPID", turningSpeed);
        SmartDashboard.putNumber("distPID",ySpeed);
        
        yaw = (Math.abs(targetYaw - yaw) > yawErrorAllowed) ? yaw : 0;
        x = (Math.abs(x - targetX) > xErrorAllowed) ? x : 0;
        distance = (Math.abs(targetDistance - distance) > distanceErrorAllowed) ? distance : 0;
        SmartDashboard.putNumber("yaw after error", yaw);
        SmartDashboard.putNumber("x after error", x);
        SmartDashboard.putNumber("dist after error", distance);
        turningSpeed *= yaw == 0 ? 0: smallSpeed;
        xSpeed *= x == 0 ? 0: smallSpeed;
        ySpeed *= distance == 0 ? 0: smallSpeed;

        turningSpeed = (Math.abs(targetYaw - yaw) > yawErrorAllowed) ? turningSpeed : 0;
        xSpeed = (Math.abs(x - targetX) > xErrorAllowed) ? xSpeed : 0;
        ySpeed = (Math.abs(targetDistance - distance) > distanceErrorAllowed) ? ySpeed : 0;

        SmartDashboard.putNumber("yaw difference", yaw-targetYaw);
        SmartDashboard.putNumber("x difference", targetX - x);
        SmartDashboard.putNumber("dist difference", targetDistance-distance);

        // xSpeed = turningSpeed == 0 ? xSpeed : 0;
        // ySpeed = turningSpeed == 0 ? ySpeed : 0;
        
        
        // Apply deadzones: set values to zero if they are within target range
        // yaw = (yaw < targetYaw-yawErrorAllowed || yaw > targetYaw+yawErrorAllowed) ? yaw : 0;
        // x = (x < targetX-xErrorAllowed || x > targetX+xErrorAllowed) ? x : 0;
        // distance = (distance < targetDistance-distanceErrorAllowed || distance > targetDistance+distanceErrorAllowed) ? distance : 0;
   
        // If current values are within target range, set the speeds to zero
        // If they aren't, set speeds to the error between the target and current values
        // turningSpeed = yaw == 0 ? 0 : targetYaw-yaw;
        // ySpeed = distance == 0 ? 0 : targetDistance-distance;
        // xSpeed = x == 0 ? 0 : x-targetX;

        // Set all the speeds to a constant drive speed
        // Set to negative if the speed needs to be reversed
        // ySpeed = ySpeed < 0 ? driveSpeed : ySpeed > 0 ? -driveSpeed : 0;
        // turningSpeed = turningSpeed < 0 ? -driveSpeed : turningSpeed > 0 ? driveSpeed : 0;
        // xSpeed = xSpeed < 0 ? -driveSpeed : xSpeed > 0 ? driveSpeed : 0;

        // Slow down the x speed if the error is small to prevent overshooting
        // xSpeed *= (Math.abs(x-targetX) < 3.5) ? 0.7 : 1;

        // Drive the robot
        chassisSpeeds = new ChassisSpeeds(-ySpeed,xSpeed,turningSpeed);
        mSubsystem.drive(chassisSpeeds);
        
        // Update smart dahsboard
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turningSpeed", turningSpeed);
    }

    @Override
    public void end(boolean isFinished) {
        mSubsystem.toggleFindingPos();
    }

    @Override
    public boolean isFinished() {
        // End method when aligned
        if ((Math.abs(mLimelight.getX() - targetX) < xErrorAllowed) && (Math.abs(targetYaw - mLimelight.getYaw()) < yawErrorAllowed) && (Math.abs(targetDistance - mLimelight.getDistance()) < distanceErrorAllowed)) {
            return true;
        }
        return false;
    }

    public double[] getTargets() {
        return new double[]{targetX, targetDistance, targetYaw};
    }
}
