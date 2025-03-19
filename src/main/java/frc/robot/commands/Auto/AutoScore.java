package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoCoralScore;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoScore extends Command {
    private double initialX;
    private double initialDistance;
    private double initialYaw;
    private Limelight mLimelight;
    private Elevator mElevator;
    private AprilTagAlign mScoringAlign;
    private AprilTagAlign mStartingPosAlign;
    private SwerveSubsystem mSubsystem;
    private AutoCoralScore mCoralScore;
    private boolean commadFinished;
    private SequentialCommandGroup commandSequence;


    public AutoScore(Limelight pLimelight, Elevator pElevator, Coral pCoral, SwerveSubsystem pSwerveSubsystem, AprilTagAlign pScoringAlign) {
        mLimelight = pLimelight;
        mElevator = pElevator;
        mScoringAlign = pScoringAlign;
        mSubsystem = pSwerveSubsystem;
        mCoralScore = new AutoCoralScore(pCoral);
    }

    @Override
    public void initialize() {
        // Starting position varibales
        initialX = mLimelight.getX();
        initialDistance = mLimelight.getDistance();
        initialYaw = mLimelight.getYaw();

        // Command used to align back to the starting position
        mStartingPosAlign = new AprilTagAlign(mSubsystem, mLimelight, initialX, initialDistance, initialYaw);
        
        // Set elevator to level 4
        mElevator.setDesiredPosition("coral", 4);

        commandSequence = new SequentialCommandGroup(
            mScoringAlign, 
            new WaitCommand(0.25),
            mCoralScore,
            new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0)),
            mStartingPosAlign
        );
        commandSequence.schedule();
    }

    @Override 
    public void execute() {


        // Align to scoring position
        // if (!(mScoringAlign.isScheduled() || mCoralScore.isScheduled() || mStartingPosAlign.isScheduled())){
        //     System.out.println("scoring align scheduled");
        //     mScoringAlign.schedule();
        // }

        // // Score in level four
        // if (mScoringAlign.isFinished() && !mCoralScore.isScheduled()){
        //     System.out.println("coralscore scheduled");
        //     mCoralScore.schedule();
        // }

        // // Reset the elevator and align back to starting position
        // if (mCoralScore.isFinished() && !mStartingPosAlign.isScheduled()){
        //     System.out.println("starting pos align scheduled");
        //     mElevator.setDesiredPosition("coral", 0);
        //     mStartingPosAlign.schedule();
        // }

    }

    @Override
    public void end(boolean isFinished) {
        System.out.println("done");
    }

    @Override
    public boolean isFinished() {
        // End the command when 
        return commandSequence.isFinished();
    }
}
