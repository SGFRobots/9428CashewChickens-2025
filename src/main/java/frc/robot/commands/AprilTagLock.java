package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final double rotationSpeed;
    private final double strafeSpeed;
    private final double driveSpeed;

    public AprilTagLock(SwerveSubsystem pSubsystem, Limelight pLimelight) {
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;
        rotationSpeed = 0.001 * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        strafeSpeed = 0.0065 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
        driveSpeed = 0.001 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
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
        area = mLimelight.getA();

        
        yaw = (yaw < -3 || yaw > 3) ? yaw : 0;
        x = (x < -2 || x > 2) ? x : 0;
        area = (area < 8 || area > 10) ? area : 0;
        area *= (area > 10) ? -1 : 1;
        area = area > 0 ? 70-area : area < 0 ? -70-area : 0;
        // area = (area < 1) ? area : 1;
        // area = 6;

        double turningSpeed = rotationSpeed * yaw;
        double xSpeed = strafeSpeed * x;
        double ySpeed = driveSpeed * area;
        chassisSpeeds = new ChassisSpeeds(-ySpeed,xSpeed,turningSpeed);

        mSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean isFinished) {
        mSubsystem.toggleFindingPos();
    }

    // @Override
    // public boolean isFinished() {
    //     if (((x<3) && (x>-3)) && ((yaw<2) && (yaw>-2)) && (area >= 6)) {
    //         return true;
    //     }
    //     return false;
    // }
}
