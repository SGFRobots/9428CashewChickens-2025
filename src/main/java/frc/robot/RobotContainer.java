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
import frc.robot.commands.Driving.ResetRotations;
import frc.robot.commands.Driving.SpeedControl;
import frc.robot.commands.Driving.SwerveJoystick;
import frc.robot.commands.Limelight.AprilTagLock;
import frc.robot.commands.Limelight.AprilTagScore;
import frc.robot.commands.Limelight.LimeLightControl;

import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.commands.Arm.AlgaeIntake;
import frc.robot.commands.Arm.AlgaeWheel;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.commands.Arm.CoralScore;
import frc.robot.commands.Arm.ElevatorControl;
import frc.robot.commands.Arm.ElevatorDesiredPosition;
import frc.robot.commands.Auto.AutoScore;

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
  private final ElevatorDesiredPosition mElevatorPosition0;
  private final ElevatorDesiredPosition mElevatorPosition1;
  private final ElevatorDesiredPosition mElevatorPosition2;
  private final ElevatorDesiredPosition mElevatorPosition3;
  private final AutoScore mAutoScoreRight;
  private final AutoScore mAutoScoreLeft;
  private final CoralIntake mCoralIntake;
  private final Elevator mElevator;
  private final Coral mCoral;
  private final Algae mAlgae;
  private final AlgaeIntake mAlgaeIntake;
  private final AlgaeWheel mAlgaeWheel;

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
    mElevatorPosition0 = new ElevatorDesiredPosition(mElevator, 0);
    mElevatorPosition1 = new ElevatorDesiredPosition(mElevator, 1);
    mElevatorPosition2 = new ElevatorDesiredPosition(mElevator, 2);
    mElevatorPosition3 = new ElevatorDesiredPosition(mElevator, 3);
    mAutoScoreRight = new AutoScore(mSwerveSubsystem, mLimelight, "right");
    mAutoScoreLeft = new AutoScore(mSwerveSubsystem, mLimelight, "left");
    mCoral = new Coral();
    mCoral.setDefaultCommand(new CoralScore(mCoral, mController));
    mCoralIntake = new CoralIntake(mCoral);
    mAlgae = new Algae();
    mAlgaeIntake = new AlgaeIntake(mAlgae);
    mAlgaeWheel = new AlgaeWheel(mAlgae, mController);
    mAlgae.setDefaultCommand(mAlgaeWheel);

    // mElevatorControl = new ElevatorControl(elevatorSubsystem, mController); // Come back to this
    
    // Autonomous
    // Auto commands
    new EventTrigger("lockOn").onTrue(Commands.runOnce(()->{System.out.println("Locking on");}));
    NamedCommands.registerCommand("goToLevel4", mElevatorPosition3);
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
    new JoystickButton(mController, Constants.Controllers.selected.UpperC).whileTrue(mSpeeds.fast);
    new JoystickButton(mController, Constants.Controllers.selected.LowerC).whileTrue(mSpeeds.slow);
    new JoystickButton(mController, Constants.Controllers.selected.ButtonDPort).toggleOnTrue(mAlgaeIntake);
    new JoystickButton(mController, Constants.Controllers.selected.UpperB).whileTrue(mElevatorPosition3);
    new JoystickButton(mController, Constants.Controllers.selected.MiddleB).whileTrue(mElevatorPosition2);
    new JoystickButton(mController, Constants.Controllers.selected.LowerB).whileTrue(mElevatorPosition1);
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
