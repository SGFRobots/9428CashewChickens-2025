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
    private double targetX;
    private double targetArea;
    private double targetYaw;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private final double rotationSpeed;
    private final double strafeSpeed;
    private final double driveSpeed;
    private double limelightRotation;
    private double robotRotation;

    public AprilTagLock(SwerveSubsystem pSubsystem, Limelight pLimelight, double pTargetX, double pTargetArea, double pTargetYaw) {
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;
        rotationSpeed = 0.005 * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        strafeSpeed = 0.0065 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        driveSpeed = 0.05 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        targetX = pTargetX;
        targetArea = pTargetArea;
        targetYaw = pTargetYaw;
    }

    public AprilTagLock(SwerveSubsystem pSubsystem, Limelight pLimelight) {
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;
        rotationSpeed = 0.005 * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        strafeSpeed = 0.003 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        driveSpeed = 0.006 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        targetX = 0;
        targetArea = 9;
        targetYaw = 0;
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
        System.out.println("Locking on");
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        area = mLimelight.getA();
        
        yaw = (yaw < targetYaw-3 || yaw > targetYaw+3) ? yaw : 0;
        x = (x < targetX-2 || x > targetX+2) ? x : 0;
        area = (area < targetArea-1 || area > targetArea+1) ? area : 0;
        area *= (area > targetArea+1) ? -1 : 1;
        area = area > 0 ? 70-area : area < 0 ? -70-area : 0;

        turningSpeed = yaw == 0 ? 0 : rotationSpeed * (yaw-targetYaw);
        ySpeed = area == 0 ? 0 : area;
        xSpeed = x == 0 ? 0 : x-targetX;

        limelightRotation = Math.toDegrees(Math.atan2(ySpeed, xSpeed));
        robotRotation = (limelightRotation - 46);

        xSpeed = Math.cos(Math.toRadians(robotRotation)) * driveSpeed;
        ySpeed = Math.sin(Math.toRadians(robotRotation)) * driveSpeed;

        // if (xSpeed != 0){
        //     ySpeed = 0;
        //     turningSpeed = 0;
        // }

        SmartDashboard.putNumber("limelightRotation", limelightRotation);
        SmartDashboard.putNumber("robotRotation", robotRotation);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        // xSpeed = (turningSpeed == 0) && (ySpeed == 0) ? xSpeed : 0;
        // xSpeed = (x == 0) && (yaw ==0) && (area == 0) ? 0.2 : xSpeed;
        chassisSpeeds = new ChassisSpeeds(ySpeed,-xSpeed,turningSpeed);

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
        // if (((x > targetX-2) && (x < targetX+2)) && ((yaw>targetYaw-3) && (yaw<targetYaw+2)) && ((area > targetArea-1 && area < targetArea+1))) {
            return true;
        }
        return false;
    }
}
