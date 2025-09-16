package frc.robot.commands.Limelight;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

// Backup from April Tag to realign
public class ReAlign extends Command {
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private final SwerveSubsystem mSubsystem;
    // private final Limelight mLimelight;
    private final Timer timer;

    public ReAlign(SwerveSubsystem pSubsystem, Limelight pLimelight, String side) {
        mSubsystem = pSubsystem;
        // mLimelight = pLimelight;
        timer = new Timer();
        ySpeed = -0.25;
        xSpeed  = side.equals("left") ? -0.25 : 0.25;
        turningSpeed = 0.0;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override 
    public void execute() {
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(ySpeed,xSpeed,turningSpeed);
        mSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean isFinished) {

    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 0.25){
            return true;
        }
        return false;
    }
}
