package frc.robot.commands.Limelight;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;

// Official Aligning to April Tag with Limelight
public class AprilTagAlign extends Command {
    private final SwerveSubsystem mSubsystem;
    private final Limelight mLimelight;
    private final Timer timer;
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
        // targetArea = pTargetDistance;

        timer = new Timer();

    }

    @Override
    public void initialize() {
        String name = mLimelight.getName();
        mSubsystem.toggleFindingPos();
        if (name.equals("left")) {
            xPID = Constants.PIDAlignment.leftXPID;
            yPID = Constants.PIDAlignment.leftYPID;
            turnPID = Constants.PIDAlignment.leftTurnPID;
        } else if (name.equals("right")) {
            xPID = Constants.PIDAlignment.rightXPID;
            yPID = Constants.PIDAlignment.rightYPID;
            turnPID = Constants.PIDAlignment.rightTurnPID;
        }

        timer.restart();
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        // Get data from limelight
        yaw = mLimelight.getYaw();
        x = mLimelight.getX();
        distance = mLimelight.getDistance();
        // area = mLimelight.getArea();

        // Calculate Speeds with PID
        xSpeed = xPID.calculate(x, targetX);
        turningSpeed = turnPID.calculate(yaw, targetYaw);
        ySpeed = -yPID.calculate(distance, targetDistance);
        // ySpeed = yPID.calculate(distance, targetDistance);

        // Apply Deadzones to speed
        xSpeed = (Math.abs(xSpeed) > 0.02) ? xSpeed : 0;
        ySpeed = (Math.abs(ySpeed) > 0.02) ? ySpeed : 0;
        turningSpeed = (Math.abs(turningSpeed) > 0.02) ? turningSpeed : 0;
        
        // Update SmartDashboard
        SmartDashboard.putNumber("xPID", xSpeed);
        SmartDashboard.putNumber("yawPID", turningSpeed);
        SmartDashboard.putNumber("distPID",ySpeed);
        

        // Drive the robot
        chassisSpeeds = new ChassisSpeeds(ySpeed,xSpeed,turningSpeed);
        // chassisSpeeds = new ChassisSpeeds(ySpeed,0,0);
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
        // End method when aligned (if timedout, controller moved, limelight within errorAllowed, speed <0.02, or tag ID invalid)
        // if ((timer.get() >= 5) || (RobotContainer.driveControllerMoving()) || (((Math.abs(mLimelight.getX() - targetX) < xErrorAllowed) && (Math.abs(targetYaw - mLimelight.getYaw()) < yawErrorAllowed) && (Math.abs(targetDistance - mLimelight.getDistance()) < distanceErrorAllowed)) || (Math.abs(xSpeed) < 0.02 && Math.abs(ySpeed) < 0.02) && Math.abs(turningSpeed) < 0.02) || (mLimelight.getID() == -1)) {
        //     return true;
        // }

        // If speed is 0 or controller moved
        return ((xSpeed == 0) && (ySpeed == 0) && (turningSpeed == 0)) || RobotContainer.driveControllerMoving();
        // return false;
    }

    public double[] getTargets() {
        return new double[]{targetX, targetDistance, targetYaw};
    }
}
