package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoScore extends SequentialCommandGroup{
    private Elevator mElevator;
    private AprilTagAlign mScoringAlign;
    private AutoCoralScore mCoralScore;

    public AutoScore(Elevator pElevator, Coral pCoral, AprilTagAlign pScoringAlign){
        mElevator = pElevator;
        mScoringAlign = pScoringAlign;
        mCoralScore = new AutoCoralScore(pCoral);

        addCommands(
            mScoringAlign, 
            new InstantCommand(() -> mElevator.setDesiredPosition("coral", 4)),
            new WaitCommand(1.25),
            mCoralScore,
            new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0))
            );
    }
}
