package frc.robot.commands.Auto.Driving;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

// Drive based on chassisspeeds for certain time limit
public class AutoDrive extends Command {
    private Timer timer;
    private final SwerveSubsystem mSubsystem;
    private ChassisSpeeds chassisSpeeds;
    private double secs;
    
    public AutoDrive (SwerveSubsystem pSubsystem, ChassisSpeeds pChassisSpeeds, double seconds) {
        mSubsystem = pSubsystem;
        chassisSpeeds = pChassisSpeeds;
        timer = new Timer();
        secs = seconds;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override 
    public void execute() {

        mSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean isFinished) {
        mSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= secs;
    }
}
