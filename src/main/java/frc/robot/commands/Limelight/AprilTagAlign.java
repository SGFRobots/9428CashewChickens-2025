package frc.robot.commands.Limelight;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;


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


    public AprilTagAlign(SwerveSubsystem pSubsystem, Limelight pLimelight, double pTargetX, double pTargetDistance, double pTargetYaw) {
        // Set up subsystems
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;

        // Set up speed and deadzone
        driveSpeed = 0.013 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
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
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        // Get data from limelight
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        distance = mLimelight.getDistance();
        
        // Apply deadzones: set values to zero if they are within target range
        yaw = (yaw < targetYaw-yawErrorAllowed || yaw > targetYaw+yawErrorAllowed) ? yaw : 0;
        x = (x < targetX-xErrorAllowed || x > targetX+xErrorAllowed) ? x : 0;
        distance = (distance < targetDistance-distanceErrorAllowed || distance > targetDistance+distanceErrorAllowed) ? distance : 0;
   
        // If current values are within target range, set the speeds to zero
        // If they aren't, set speeds to the error between the target and current values
        turningSpeed = yaw == 0 ? 0 : targetYaw-yaw;
        ySpeed = distance == 0 ? 0 : targetDistance-distance;
        xSpeed = x == 0 ? 0 : x-targetX;

        // Set all the speeds to a constant drive speed
        // Set to negative if the speed needs to be reversed
        ySpeed = ySpeed < 0 ? driveSpeed : ySpeed > 0 ? -driveSpeed : 0;
        turningSpeed = turningSpeed < 0 ? -driveSpeed : turningSpeed > 0 ? driveSpeed : 0;
        xSpeed = xSpeed < 0 ? -driveSpeed : xSpeed > 0 ? driveSpeed : 0;

        // Slow down the x speed if the error is small to prevent overshooting
        xSpeed *= (Math.abs(x-targetX) < 3.5) ? 0.7 : 1;

        // Drive the robot
        chassisSpeeds = new ChassisSpeeds(ySpeed,-xSpeed,turningSpeed);
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
        // End method when aligned or limelight is no longer picking up data
        if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
            return true;
        }
        return false;
    }
}
