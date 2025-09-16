package frc.robot.commands.Auto.Driving;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// Sequential AutoDrive
public class AutoPath extends SequentialCommandGroup{
    public AutoPath(SwerveSubsystem mSubsystem) {

        addCommands(
            new AutoDrive(mSubsystem, new ChassisSpeeds(.25,0,0), 1.5)
        );
    }
}
