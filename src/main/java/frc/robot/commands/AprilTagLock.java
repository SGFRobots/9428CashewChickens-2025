package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagLock extends Command{
    private final SwerveSubsystem mSubsystem;
    private final Limelight mLimelight;
    private double x;
    private double yaw;
    private final double rotationSpeed;
    private final double strafeSpeed;

    public AprilTagLock(SwerveSubsystem pSubsystem, Limelight pLimelight) {
        mSubsystem = pSubsystem;
        mLimelight = pLimelight;
        rotationSpeed = 0.001 * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        strafeSpeed = 0.0065 * Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
    }

    @Override
    public void initialize() {
        mSubsystem.toggleFindingPos();
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;
        
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();

        yaw = (yaw < -3 || yaw > 3) ? yaw : 0;
        x = (x < -3 || x > 3) ? x : 0;

        double turningSpeed = rotationSpeed * yaw;
        double xSpeed = strafeSpeed * x;
        chassisSpeeds = new ChassisSpeeds(0,xSpeed,turningSpeed);

        mSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean isFinished) {
        mSubsystem.toggleFindingPos();
    }

    @Override
    public boolean isFinished() {
        if (((x<3) && (x>-3)) && ((yaw<3) && (yaw>-3))) {
            return true;
        }
        return false;
    }
}
