package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Limelight.AprilTagLock;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto extends SequentialCommandGroup{
    // private SwerveSubsystem mSubsystem;
    public Auto(SwerveSubsystem subsystem, Limelight pLimelight, double pTargetX, double pTargetArea, double pTargetYaw) {
        // addCommands(null);
        addCommands(
            new AprilTagLock(subsystem, pLimelight, 0, pTargetArea, pTargetYaw),
            new AprilTagLock(subsystem, pLimelight, pTargetX, pTargetArea, pTargetYaw)
        );
    }
}
