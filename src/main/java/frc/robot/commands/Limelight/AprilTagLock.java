package frc.robot.commands.Limelight;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;


public class AprilTagLock extends Command {
    private final SwerveSubsystem mSubsystem;
    private final Limelight mLimelight;
    private double x;
    private double area;
    private double yaw;
    private double y;
    private double targetX;
    private double targetDistance;
    private double targetYaw;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private final double rotationSpeed;
    private final double strafeSpeed;
    private double driveSpeed;
    private double limelightRotation;
    private double robotRotation;
    private double xErrorAllowed;
    private double distanceErrorAllowed;
    private double yawErrorAllowed;


    public AprilTagLock(SwerveSubsystem pSubsystem, Limelight pLimelight, double pTargetX, double pTargetDistance, double pTargetYaw) {
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;
        rotationSpeed = 0.0005 * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        strafeSpeed = 0.03 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        driveSpeed = 0.04 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        targetX = pTargetX;
        targetDistance = pTargetDistance;
        targetYaw = pTargetYaw;
        xErrorAllowed = 0.75;
        distanceErrorAllowed = 0.02;
        yawErrorAllowed = 1.5;
    }

    @Override
    public void initialize() {
        mSubsystem.toggleFindingPos();
        yaw = rotationSpeed;
        x = strafeSpeed;
        area = strafeSpeed;
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        y = mLimelight.getDistance();
        // area = mLimelight.getA();
        
        // Set the current values to zero if they are within the desired range
        // If they aren't, set them to the necessary speed variable
        yaw = (yaw < targetYaw-yawErrorAllowed || yaw > targetYaw+yawErrorAllowed) ? yaw : 0;
        x = (x < targetX-xErrorAllowed || x > targetX+xErrorAllowed) ? x : 0;
        y = (y < targetDistance-distanceErrorAllowed || y > targetDistance+distanceErrorAllowed) ? y : 0;
        // area *= (area > targetDistance+distanceErrorAllowed) ? -1 : 1;
        // area = area > 0 ? 70-area : area < 0 ? -70-area : 0;
        
        // Set speeds
        turningSpeed = yaw == 0 ? 0 : targetYaw-yaw;
        ySpeed = y == 0 ? 0 : targetDistance-y;
        xSpeed = x == 0 ? 0 : x-targetX;


        // turningSpeed = yaw == 0 ? 0 : rotationSpeed * (targetYaw-yaw);
        // ySpeed = y == 0 ? 0 : y-targetDistance * driveSpeed;
        ySpeed = ySpeed < 0 ? driveSpeed : ySpeed > 0 ? -driveSpeed : 0;
        turningSpeed = turningSpeed < 0 ? -driveSpeed : turningSpeed > 0 ? driveSpeed : 0;
        xSpeed = xSpeed < 0 ? -driveSpeed : xSpeed > 0 ? driveSpeed : 0;
        xSpeed *= (x-targetX < 3.5) ? 0.7 : 1;

        // Convert angle of direction to move in relation to the limelight rather than the robot
        limelightRotation = Math.toDegrees(Math.atan2(ySpeed, xSpeed));
        robotRotation = (limelightRotation - 46);

        // if (xSpeed != 0){
        //     xSpeed = Math.cos(Math.toRadians(robotRotation));
        //     // ySpeed = Math.sin(Math.toRadians(robotRotation));
        // }

        // xSpeed *= strafeSpeed;
        // ySpeed *= driveSpeed;
        
        // if (xSpeed != 0){
        //     ySpeed = 0;
        //     turningSpeed = 0;
        // }

        SmartDashboard.putNumber("limelightRotation", limelightRotation);
        SmartDashboard.putNumber("robotRotation", robotRotation);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("yaw difference", targetDistance - y);
        SmartDashboard.putNumber("turningSpeed", turningSpeed);

        // xSpeed = (turningSpeed == 0) && (ySpeed == 0) ? xSpeed : 0;
        // xSpeed = (x == 0) && (yaw ==0) && (area == 0) ? 0.2 : xSpeed;
        chassisSpeeds = new ChassisSpeeds(ySpeed,-xSpeed,turningSpeed);
        // chassisSpeeds = new ChassisSpeeds();

        mSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean isFinished) {
        System.out.println("Done");
        mSubsystem.toggleFindingPos();
    }

    @Override
    public boolean isFinished() {
        if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
        // if (((x > targetX-2) && (x < targetX+2)) && ((yaw>targetYaw-3) && (yaw<targetYaw+2)) && ((area > targetDistance-1 && area < targetDistance+1))) {
            return true;
        }
        return false;
    }
}
