// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
// Subsystems and commands
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.AprilTagLock;
import frc.robot.commands.AprilTagScore;
import frc.robot.commands.CoralScore;
import frc.robot.commands.LimeLightControl;
import frc.robot.commands.ResetRotations;
import frc.robot.commands.SpeedControl;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorControl;
import frc.robot.commands.ElevatorDesiredPosition;
import frc.robot.commands.AutoScore;
import frc.robot.commands.CoralIntake;

public class RobotContainer {

  // Instances of controllers
  // public final XboxController mController;
  public static final GenericHID mController = new GenericHID(0);
  private final SendableChooser<Command> autoChooser;
  // Subsystems and commands
  private final SwerveSubsystem mSwerveSubsystem;
  private final ResetRotations mResetRotations;
  private final AprilTagLock mAprilTagLock;
  private final SpeedControl mSpeeds;
  private final Limelight mLimelight;
  private final AprilTagScore mAprilTagScore;
  private final ElevatorDesiredPosition mElevatorPosition1;
  private final ElevatorDesiredPosition mElevatorPosition2;
  private final ElevatorDesiredPosition mElevatorPosition3;
  private final ElevatorDesiredPosition mElevatorPosition4;
  private final AutoScore mAutoScoreRight;
  private final AutoScore mAutoScoreLeft;
  private final CoralIntake mCoralIntake;
  private final Elevator mElevator;
  private final Coral mCoral;
  // private final CoralScore mCoralScore;
  // private final ElevatorControl mElevatorControl;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems and commands
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mController));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    mSpeeds = new SpeedControl(mSwerveSubsystem);
    mLimelight = new Limelight();
    mLimelight.setDefaultCommand(new LimeLightControl(mLimelight));
    mAprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, 0, 9, 0);
    mAprilTagScore = new AprilTagScore(mSwerveSubsystem, mLimelight, mController);
    mElevator = new Elevator();
    mElevator.setDefaultCommand(new ElevatorControl(mElevator, mController));
    mElevatorPosition1 = new ElevatorDesiredPosition(mElevator, Constants.Mechanical.ElevatorLevelOneHeight);
    mElevatorPosition2 = new ElevatorDesiredPosition(mElevator, Constants.Mechanical.ElevatorLevelTwoHeight);
    mElevatorPosition3 = new ElevatorDesiredPosition(mElevator, Constants.Mechanical.ElevatorLevelThreeHeight);
    mElevatorPosition4 = new ElevatorDesiredPosition(mElevator, Constants.Mechanical.ElevatorLevelFourHeight);
    mAutoScoreRight = new AutoScore(mSwerveSubsystem, mLimelight, "right");
    mAutoScoreLeft = new AutoScore(mSwerveSubsystem, mLimelight, "left");
    mCoral = new Coral();
    mCoral.setDefaultCommand(new CoralScore(mCoral, mController));
    mCoralIntake = new CoralIntake(mCoral);

    // mElevatorControl = new ElevatorControl(elevatorSubsystem, mController); // Come back to this
    
    // Autonomous
    // Auto commands
    new EventTrigger("lockOn").onTrue(Commands.runOnce(()->{System.out.println("Locking on");}));
    NamedCommands.registerCommand("goToLevel4", mElevatorPosition4);
    NamedCommands.registerCommand("goToSourceLevel", mElevatorPosition1);
    NamedCommands.registerCommand("coralScoreLeft", mAutoScoreLeft);
    NamedCommands.registerCommand("coralScoreRight", mAutoScoreRight);
    NamedCommands.registerCommand("coralIntake", mCoralIntake);

    // Auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // new JoystickButton(mController, Constants.Controllers.selected.ButtonAPort).onTrue(mResetRotations);
    new JoystickButton(mController, Constants.Controllers.selected.ButtonAPort).onTrue(new InstantCommand(() -> mElevator.resetPositions()));
    // new JoystickButton(mController, Constants.Controllers.selected.UpperC).whileTrue(mSpeeds.fast);
    // new JoystickButton(mController, Constants.Controllers.selected.LowerC).whileTrue(mSpeeds.slow);
    new JoystickButton(mController, Constants.Controllers.selected.ButtonDPort).toggleOnTrue(mAprilTagScore);
    new JoystickButton(mController, Constants.Controllers.selected.UpperB).toggleOnTrue(mElevatorPosition4);
    new JoystickButton(mController, Constants.Controllers.selected.MiddleB).toggleOnTrue(mElevatorPosition3);
    new JoystickButton(mController, Constants.Controllers.selected.LowerB).toggleOnTrue(mElevatorPosition2);
    // new JoystickButton(mController, Constants.Controllers.selected.UpperC).onTrue(mCoralScore);
    // new JoystickButton(mController, Constants.Controllers.selected.LowerC).onTrue(mCoralScore);
    // new JoystickButton(mController, Constants.Controllers.selected.)
    // new JoystickButton(mController,Constants.Controllers.selected.UpperB).whileTrue(mElevatorControl);
  }

  public void updateCoralStatus() {
    // mCoral.updateCoralStatus();
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
