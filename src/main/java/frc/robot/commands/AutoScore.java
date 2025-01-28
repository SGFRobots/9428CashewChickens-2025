package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AprilTagLock;

public class AutoScore extends Command{
    private final Limelight mLimelight;
    private double initialX;
    private double initialArea;
    private double initialYaw;
    private double scoringX;
    private double scoringArea;
    private double scoringYaw;
    private AprilTagLock initialAprilTagLock;
    private AprilTagLock scoringAprilTagLock;
    private final SwerveSubsystem mSwerveSubsystem;

    /*Lock on to april tag
    Store starting location
    Go to the left or the right reef depending on the parameter
    Score
    Go back to starting location
    */
    public AutoScore(SwerveSubsystem pSubsystem, Limelight pLimelight, String coralSide) {
        mLimelight = pLimelight;
        mSwerveSubsystem = pSubsystem;
        scoringX = (coralSide == "right") ? -1 : 1;
        scoringYaw = (coralSide == "right") ? 12 : -12;
        scoringArea = (coralSide == "right") ? 20 : 25;
        scoringAprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, scoringX, scoringArea, scoringYaw);
    }

    @Override
    public void initialize() {
        initialYaw = mLimelight.getYaw();
        initialX = mLimelight.getX();
        initialArea = mLimelight.getA();
        initialAprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, initialX, initialArea, initialYaw);
    }

    @Override 
    public void execute() {
        scoringAprilTagLock.schedule();
        System.out.println("Score");
        initialAprilTagLock.schedule();
    }

    @Override
    public void end(boolean isFinished) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
