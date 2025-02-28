// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.XMLConstants;

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
import frc.robot.commands.Limelight.AprilTagLock2;
import frc.robot.commands.Limelight.AprilTagScore;
import frc.robot.commands.Limelight.LimeLightControl;

import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.Constants.Controllers.XBox;
import frc.robot.commands.Arm.AlgaeIntake;
import frc.robot.commands.Arm.AlgaeWheel;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.commands.Arm.CoralScore;
import frc.robot.commands.Arm.ElevatorControl;
import frc.robot.commands.Arm.ElevatorDesiredPosition;
import frc.robot.commands.Auto.Auto;
import frc.robot.commands.Auto.AutoScore;

public class RobotContainer {

  // Instances of controllers
  // public final XboxController mDroneComtroller;
  public static final GenericHID mDroneComtroller = new GenericHID(0);
  public static final GenericHID mXBoxController = new GenericHID(1);
  private final SendableChooser<Command> autoChooser;
  // Subsystems and commands
  private final SwerveSubsystem mSwerveSubsystem;
  private final ResetRotations mResetRotations;
  private final AprilTagLock mAprilTagLock;
  private final AprilTagLock2 mAprilTagLock2;
  private final Auto lock;
  private final SpeedControl mSpeeds;
  private final Limelight mLimelight;
  private final AprilTagScore mAprilTagScore;
  // private final ElevatorDesiredPosition mElevatorPosition0;
  // private final ElevatorDesiredPosition mElevatorPosition1;
  // private final ElevatorDesiredPosition mElevatorPosition2;
  // private final ElevatorDesiredPosition mElevatorPosition3;
  private final ElevatorDesiredPosition mElevatorDesiredPosition;
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
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mDroneComtroller));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    mSpeeds = new SpeedControl(mSwerveSubsystem, mDroneComtroller);
    mSpeeds.schedule();
    mLimelight = new Limelight();
    mLimelight.setDefaultCommand(new LimeLightControl(mLimelight));
    lock = new Auto(mSwerveSubsystem, mLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]);
    // mAprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]);
    mAprilTagLock = new AprilTagLock(mSwerveSubsystem, mLimelight, 5, 5, -40);
    mAprilTagLock2 = new AprilTagLock2(mSwerveSubsystem, mLimelight, 5, 5, -40);
    mAprilTagScore = new AprilTagScore(mSwerveSubsystem, mLimelight, mDroneComtroller);
    mElevator = new Elevator();
    mElevator.setDefaultCommand(new ElevatorControl(mElevator, mXBoxController));
    mElevatorDesiredPosition = new ElevatorDesiredPosition(mElevator);
    mAutoScoreRight = new AutoScore(mSwerveSubsystem, mLimelight, "right");
    mAutoScoreLeft = new AutoScore(mSwerveSubsystem, mLimelight, "left");
    mCoral = new Coral();
    mCoral.setDefaultCommand(new CoralScore(mCoral, mDroneComtroller));
    mCoralIntake = new CoralIntake(mCoral);
    mAlgae = new Algae();
    mAlgaeIntake = new AlgaeIntake(mAlgae);
    mAlgaeWheel = new AlgaeWheel(mAlgae, mDroneComtroller);
    mAlgae.setDefaultCommand(mAlgaeWheel);

    // mElevatorControl = new ElevatorControl(elevatorSubsystem, mDroneComtroller); // Come back to this
    
    // Autonomous
    // Auto commands
    new EventTrigger("lockOn").onTrue(Commands.runOnce(()->{System.out.println("Locking on");}));
    // NamedCommands.registerCommand("goToLevel4", mElevatorPosition3);
    // NamedCommands.registerCommand("goToSourceLevel", mElevatorPosition1);
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
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonAPort).onTrue(mResetRotations);
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonMinues).onTrue(new InstantCommand(() -> mElevator.resetPositions()));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.UpperC).whileTrue(mSpeeds.fast);
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.LowerC).whileTrue(mSpeeds.slow);
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonB).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition(0)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonY).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition(1)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonA).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition(2)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonX).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition(3)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonPlus).toggleOnTrue(new InstantCommand(() -> mElevator.setOverride(!mElevator.getOverride())));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftBumper).toggleOnTrue(mAprilTagLock);
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.RightBumper).toggleOnTrue(mCoralIntake);
    new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonEPort).whileTrue(mAlgaeIntake);
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonDPort).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition(3)));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.UpperC).onTrue(mCoralScore);
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.LowerC).onTrue(mCoralScore);
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.)
    // new JoystickButton(mDroneComtroller,Constants.Controllers.selected.UpperB).whileTrue(mElevatorControl);
  }

  public void updateCoralStatus() {
    // mCoral.updateCoralStatus();
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public Command getElevatorCommand(){
    return mElevatorDesiredPosition;
  }

  public void displayLimelightData(){
    mLimelight.displayData();
  }
}
