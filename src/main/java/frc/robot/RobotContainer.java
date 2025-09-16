// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Subsystems and commands
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Driving.SpeedControl;
import frc.robot.commands.Driving.ResetRotations;
import frc.robot.commands.Driving.SwerveJoystick;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.commands.Limelight.LimeLightControl;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Cage;
import frc.robot.commands.Arm.AlgaeControl;
import frc.robot.commands.Arm.CageControl;
import frc.robot.commands.Arm.CoralScore;
import frc.robot.commands.Arm.ElevatorControl;
import frc.robot.commands.Arm.ElevatorDesiredPosition;
import frc.robot.commands.Auto.AutoScore;
import frc.robot.commands.Auto.WaitForCoral;
import frc.robot.commands.Auto.Driving.AutoPath;
import frc.robot.commands.Auto.Driving.AutoDrive;

public class RobotContainer {

  // Controllers
  public static final GenericHID mDroneComtroller = new GenericHID(Constants.Controllers.DrivingControllerPort);
  public static final GenericHID mXBoxController = new GenericHID(Constants.Controllers.XBoxControllerPort);
  // public static final GenericHID mDriveController = new GenericHID(Constants.Controllers.DriveControllerPort);

  // auto
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  private final SwerveSubsystem mSwerveSubsystem;
  private final SpeedControl mSpeedControl;
  private final Limelight mLeftLimelight;
  private final Limelight mRightLimelight;
  private final Elevator mElevator;
  private final Coral mCoral;
  private final Algae mAlgae;
  // private final Cage mCage;

  // Commands
  private final ResetRotations mResetRotations;
  private final AprilTagAlign mTestAlign;
  private final AprilTagAlign mAprilTagLockLeft;
  private final AprilTagAlign mAprilTagLockRight;
  private final ElevatorDesiredPosition mElevatorDesiredPosition;
  private final ElevatorControl mElevatorControl;
  private final AlgaeControl mAlgaeIntake;
  private final CoralIntake mCoralIntake;
  private final AutoScore mAutoScoreRight;
  private final AutoScore mAutoScoreLeft;
  private final AutoPath mAutoPath;
  private final AutoDrive mAutoDrive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Driving
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mDroneComtroller));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    
    // Limelight and ALignment
    mLeftLimelight = new Limelight(Constants.MotorPorts.kLeftLimelightKey);
    mLeftLimelight.setDefaultCommand(new LimeLightControl(mLeftLimelight));
    mRightLimelight = new Limelight(Constants.MotorPorts.kRightLimelightKey);
    mRightLimelight.setDefaultCommand(new LimeLightControl(mRightLimelight));
    mAprilTagLockLeft = new AprilTagAlign(mSwerveSubsystem, mRightLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]);
    // mAprilTagLockLeft = new AprilTagAlign(mSwerveSubsystem, mRightLimelight, -0.04, 0.34, 0);
    // mAprilTagLockRight = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, 0, 0.3, 0);
    mAprilTagLockRight = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]);
    mTestAlign = new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]);
    
    // Elevator
    mElevator = new Elevator();
    mElevatorDesiredPosition = new ElevatorDesiredPosition(mElevator);
    mElevatorControl = new ElevatorControl(mElevator, mXBoxController);
    mElevator.setDefaultCommand(mElevatorDesiredPosition);
    
    // Speed Control
    mSpeedControl = new SpeedControl(mSwerveSubsystem, mDroneComtroller, mElevator);
    
    // Algae
    mAlgae = new Algae();
    mAlgaeIntake = new AlgaeControl(mAlgae, mXBoxController);
    mAlgae.setDefaultCommand(mAlgaeIntake);

    // Coral
    mCoral = new Coral();
    mCoral.setDefaultCommand(new CoralScore(mCoral, mAlgae, mElevator, mXBoxController));
    mCoralIntake = new CoralIntake(mCoral, mElevator);

    // Cage
    // mCage = new Cage();
    // mCage.setDefaultCommand(new CageControl(mCage));

    // Autonomous commands
    mAutoPath = new AutoPath(mSwerveSubsystem);
    mAutoDrive = new AutoDrive(mSwerveSubsystem, new ChassisSpeeds(0,1,0), 1);
    mAutoScoreLeft = new AutoScore(mElevator, mCoral, new AprilTagAlign(mSwerveSubsystem, mRightLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]));
    mAutoScoreRight = new AutoScore(mElevator, mCoral, new AprilTagAlign(mSwerveSubsystem, mLeftLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]));
    setUpAuto();

    // Auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    // new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftJoystickButton).onTrue(new InstantCommand(() -> mElevator.resetPositions()));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftJoystickButton).onTrue(new InstantCommand(() -> mSwerveSubsystem.zeroHeading()));

    new JoystickButton(mXBoxController, Constants.Controllers.XBox.RightJoystickButton).onTrue(new InstantCommand(() -> mAlgae.resetPos()));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonB).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0)));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonFPort).onTrue(new AutoPath(mSwerveSubsystem));
    new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonAPort).onTrue(mResetRotations);
    new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonAPort).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 1)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonY).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 2)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonA).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 3)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonX).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 4)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftBumper).toggleOnTrue(mAprilTagLockLeft);
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.RightBumper).toggleOnTrue(mAprilTagLockRight);
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonMinus).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition("algae", 0)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonPlus).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition("algae", 1)));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonEPort).onTrue(new InstantCommand(() -> mCage.setDesiredPos(1)));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonEPort).onFalse(new InstantCommand(() -> mCage.setDesiredPos(0)));
    
    // new JoystickButton(mDriveController, Constants.Controllers.XBoxBackup.ButtonA).onTrue(new InstantCommand(() -> mSpeedControl.slowModeOn()));
    // new JoystickButton(mDriveController, Constants.Controllers.XBoxBackup.ButtonB).onTrue(new InstantCommand(() -> mSpeedControl.normalModeOn()));
    // new JoystickButton(mDriveController, Constants.Controllers.XBoxBackup.ButtonX).onTrue(new InstantCommand(() -> mSpeedControl.normalModeOn()));
    // new JoystickButton(mDriveController, Constants.Controllers.XBoxBackup.ButtonY).onTrue(new InstantCommand(() -> mSpeedControl.fastModeOn()));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonEPort).whileTrue(mAlgaeIntake);
  }

  // Set up auto commands
  private void setUpAuto() {
    NamedCommands.registerCommand("gotToSourceLevel", (new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0))));
    NamedCommands.registerCommand("goToLevel2",(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 1)))); 
    NamedCommands.registerCommand("goToLevel3",(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 2))));
    NamedCommands.registerCommand("goToLevel4", (new InstantCommand(() -> mElevator.setDesiredPosition("coral", 3))));
    NamedCommands.registerCommand("reefAlignLeft", mAprilTagLockLeft);
    NamedCommands.registerCommand("reefAlignRight", mAprilTagLockRight);
    NamedCommands.registerCommand("coralIntake", mCoralIntake);
    NamedCommands.registerCommand("coralScoreLeft", mAutoScoreLeft);
    NamedCommands.registerCommand("coralScoreRight", mAutoScoreRight);
    NamedCommands.registerCommand("waitForCoral", new WaitForCoral(mCoral));
    // NamedCommands.registerCommand("TimeBased", mAutoPath);

    // NamedCommands.registerCommand("gotToSourceLevel", new WaitCommand(1));
    // NamedCommands.registerCommand("goToLevel2", new WaitCommand(1)); 
    // NamedCommands.registerCommand("goToLevel3", new WaitCommand(1));
    // NamedCommands.registerCommand("goToLevel4", new WaitCommand(1));
    // NamedCommands.registerCommand("reefAlignLeft", new WaitCommand(1));
    // NamedCommands.registerCommand("reefAlignRight", new WaitCommand(1));
    // NamedCommands.registerCommand("coralIntake", new WaitCommand(1));
    // NamedCommands.registerCommand("coralScoreLeft", new WaitCommand(1));
    // NamedCommands.registerCommand("coralScoreRight", new WaitCommand(1));
    // NamedCommands.registerCommand("TimeBased", new WaitCommand(1));
  }

  // get selected auto
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return mAutoPath;
  }

  // get override elevator command
  public Command getElevatorCommand() {
    return mElevatorControl;
  }

  public Command getSpeedControlCommand() {
    return mSpeedControl;
  }

  public void displayElevatorPosition(){
    SmartDashboard.putNumber("Elevator Position", mElevator.getPositionRelativeToZero());
  }

  public void displayCoralSensors(){
    SmartDashboard.putBoolean("In Sensor", mCoral.getInSensorBroken());
    SmartDashboard.putBoolean("Out Sensor", mCoral.getOutSensorBroken());
  }

  public void displayAligningState(){
    SmartDashboard.putBoolean("Aligning", mSwerveSubsystem.getFindingPos());
  }

  public void autoReset() {
    mElevator.resetPositions();
    mAlgae.resetPos();
  }

  public void resetElevator(){
    mElevator.setDesiredPosition("coral", 0);
  }

  public void resetRotations(){
    mResetRotations.schedule();
  }

  public static boolean driveControllerMoving() {
    boolean leftX = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.selected.LeftXPort)) >= Constants.Mechanical.kDeadzone;
    boolean leftY = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.selected.LeftYPort)) >= Constants.Mechanical.kDeadzone;
    boolean rightX = Math.abs(mDroneComtroller.getRawAxis(Constants.Controllers.selected.RightXPort)) >= Constants.Mechanical.kDeadzone;

    return leftX || leftY || rightX;
  }

  public void resetHeading() {
    mSwerveSubsystem.zeroHeading();
  }

}
