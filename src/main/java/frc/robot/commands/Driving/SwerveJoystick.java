package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants;

import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends Command {
    private final SwerveSubsystem mSwerveSubsystem;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final GenericHID mController;

    // Constructor
    public SwerveJoystick(SwerveSubsystem pSwerveSubsystem, GenericHID pController) {
        // Subsystem and controller instances
        mSwerveSubsystem = pSwerveSubsystem;
        mController = pController;

        // Slew Rate Limiter smooths the robot's accelerations
        xLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(Constants.Mechanical.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (!mSwerveSubsystem.getFindingPos()) {
            // Get joystick inputs
            double xSpeed = mController.getRawAxis(Constants.Controllers.selected.LeftYPort);
            double ySpeed = -mController.getRawAxis(Constants.Controllers.selected.LeftXPort);
            double turningSpeed = -mController.getRawAxis(Constants.Controllers.selected.RightXPort) / 2; 
            
            // System.out.println(turningSpeed);
            // Calculate joystick hypotenuse for speed
            double joystickHypotense = Math.sqrt((Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)));
            
            // Field oriented calculations
            double robotRotation = mSwerveSubsystem.getHeading();
            double joystickAngle = Math.toDegrees(Math.atan2(xSpeed, ySpeed));
            double rotation = (90 - robotRotation) + joystickAngle;
            // xSpeed = Math.sin(Math.toRadians(rotation));
            // ySpeed = Math.cos(Math.toRadians(rotation));
            
            // Make Driving Smoother using Slew Rate Limiter - less jerky by accelerating slowly
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
            turningSpeed = turningLimiter.calculate(turningSpeed);
            
            // Calculate speed in m/s
            xSpeed *= joystickHypotense * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed *= joystickHypotense * Constants.Mechanical.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed *= Constants.Mechanical.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            // SmartDashboard.putNumber("joystickangle", joystickAngle);
            // SmartDashboard.putNumber("rot", rotation);
            
            // Apply Deadzone
            xSpeed = Math.abs(xSpeed) > Constants.Mechanical.kDeadzone ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > Constants.Mechanical.kDeadzone ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > Constants.Mechanical.kDeadzone ? turningSpeed : 0.0;

            // Set desire chassis speeds based on field or robot relative
            ChassisSpeeds chassisSpeed;
            // ChassisSpeeds chassisSpeed2;
            // chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, mSwerveSubsystem.geRotation2d());
            chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            chassisSpeed = ChassisSpeeds.discretize(chassisSpeed, 0.02);
            // chassisSpeed.vxMetersPerSecond *= -1;
            // chassisSpeed.vyMetersPerSecond *= -1;

            // Drive
            mSwerveSubsystem.drive(chassisSpeed);

            // Test
            // mSwerveSubsystem.driveIndividualModule(xSpeed, turningSpeed);

        }
        // System.out.println(mSwerveSubsystem.getFindingPos());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all modules
        mSwerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
