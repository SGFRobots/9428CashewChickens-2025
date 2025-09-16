package frc.robot.commands.Auto;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.commands.Limelight.ReAlign;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// AprilTagAlign + ReAlign until aligned
public class AutoAlign extends Command {
    private final Limelight mLimelight;
    private final AprilTagAlign mAlignment;
    private final ReAlign mReAlign;

    public AutoAlign(SwerveSubsystem pSubsystem, Limelight pLimelight, AprilTagAlign pAlignment) {
        mLimelight = pLimelight;
        mAlignment = pAlignment;

        String side;
        if (Arrays.equals(mAlignment.getTargets(), Constants.AprilTags.leftCoral)) {
            side = "left";
        } else {
            side = "right";
        }

        mReAlign = new ReAlign(pSubsystem, pLimelight, side);
    }

    @Override
    public void initialize() {
        mAlignment.schedule();
    }

    @Override 
    public void execute() {
        if (Robot.stage.equals("teleOp") && (RobotContainer.driveControllerMoving())) {
            cancel();
        }

        if ((mLimelight.getID() == -1) && !mReAlign.isScheduled() && Robot.stage.equals("auto")) {
            mReAlign.schedule();
        } else if (mAlignment.isFinished() && !mLimelight.isAligned() && !mReAlign.isScheduled()) {
            mAlignment.schedule();
        }
    }

    @Override
    public void end(boolean isFinished) {
        mAlignment.cancel();
    }

    @Override
    public boolean isFinished() {
        return mLimelight.isAligned();
    }
}
