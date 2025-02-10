package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagScore extends Command{
    private final Limelight mLimelight;
    private final GenericHID mController;
    private AprilTagLock aprilTagLock;
    private final SwerveSubsystem mSwerveSubsystem;
    private AutoDrive moveOffset;

    /*Lock on to april tag
    Store starting location
    Go to the left or the right reef depending on the parameter
    Score
    Go back to starting location
    */
    public AprilTagScore(SwerveSubsystem pSubsystem, Limelight pLimelight, GenericHID pController) {
        mLimelight = pLimelight;
        mSwerveSubsystem = pSubsystem;
        mController = pController;
        aprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, 11, 8, -1);
        moveOffset = new AutoDrive(pSubsystem, 0, 0, 0, 0);
    }

    @Override
    public void initialize() {
        aprilTagLock.schedule();
        System.out.println("scheduled 1");
    }

    @Override 
    public void execute() {
        System.out.println("execute");
        if (aprilTagLock.isFinished() && !moveOffset.isScheduled()) {
            System.out.println("next");
            double offset = mController.getRawAxis(Constants.Controllers.selected.SwitchE);
            moveOffset = new AutoDrive(mSwerveSubsystem, 0, offset / 2, 0, 0);
            moveOffset.schedule();
        }
    }

    @Override
    public void end(boolean isFinished) {
    }

    @Override
    public boolean isFinished() {
        System.out.println(moveOffset.isFinished());
        return moveOffset.isFinished();
    }
}
