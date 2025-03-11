// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
// import frc.robot.commands.Driving.ResetRotations;
import frc.robot.commands.Driving.SwerveJoystick;
import frc.robot.commands.Limelight.AprilTagAlign;
import frc.robot.commands.Limelight.LimeLightControl;
import frc.robot.subsystems.Coral;
import frc.robot.commands.Arm.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import frc.robot.commands.Arm.AlgaeControl;
import frc.robot.commands.Arm.CoralScore;
import frc.robot.commands.Arm.ElevatorControl;
import frc.robot.commands.Arm.ElevatorDesiredPosition;
import frc.robot.commands.Auto.AutoAlign;
import frc.robot.commands.Auto.AutoScore;

public class RobotContainer {

  // Controllers
  public static final GenericHID mDroneComtroller = new GenericHID(Constants.Controllers.DrivingControllerPort);
  public static final GenericHID mXBoxController = new GenericHID(Constants.Controllers.XBoxControllerPort);
  
  // auto
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  private final SwerveSubsystem mSwerveSubsystem;
  private final SpeedControl mSpeedControl;
  private final Limelight mLimelight;
  private final Elevator mElevator;
  private final Coral mCoral;
  private final Algae mAlgae;

  // Commands
  // private final ResetRotations mResetRotations;
  private final AprilTagAlign mAprilTagLockLeft;
  private final AprilTagAlign mAprilTagLockRight;
  private final ElevatorDesiredPosition mElevatorDesiredPosition;
  private final ElevatorControl mElevatorControl;
  private final AutoScore mAutoScore;
  private final AlgaeControl mAlgaeIntake;
  private final CoralIntake mCoralIntake;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Driving
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mDroneComtroller));
    // mResetRotations = new ResetRotations(mSwerveSubsystem);
    
    // Limelight and ALignment
    mLimelight = new Limelight();
    mLimelight.setDefaultCommand(new LimeLightControl(mLimelight));
    mAprilTagLockLeft = new AprilTagAlign(mSwerveSubsystem, mLimelight, Constants.AprilTags.leftCoral[0], Constants.AprilTags.leftCoral[1], Constants.AprilTags.leftCoral[2]);
    mAprilTagLockRight = new AprilTagAlign(mSwerveSubsystem, mLimelight, Constants.AprilTags.rightCoral[0], Constants.AprilTags.rightCoral[1], Constants.AprilTags.rightCoral[2]);
    
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
    mAutoScore = new AutoScore(mCoral);
    mCoralIntake = new CoralIntake(mCoral);

    // Autonomous commands
    setUpAuto();

    // Auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftJoystickButton).onTrue(new InstantCommand(() -> mElevator.resetPositions()));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.RightJoystickButton).onTrue(new InstantCommand(() -> mAlgae.resetPos()));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonB).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0, mSwerveSubsystem)));
    new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonFPort).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 1, mSwerveSubsystem)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonY).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 2, mSwerveSubsystem)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonA).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 3, mSwerveSubsystem)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonX).onTrue(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 4, mSwerveSubsystem)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.LeftBumper).toggleOnTrue(new AutoAlign(mSwerveSubsystem, mLimelight, mAprilTagLockLeft));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.RightBumper).toggleOnTrue(new AutoAlign(mSwerveSubsystem, mLimelight, mAprilTagLockRight));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonMinus).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition("algae", 0, mSwerveSubsystem)));
    new JoystickButton(mXBoxController, Constants.Controllers.XBox.buttonPlus).toggleOnTrue(new InstantCommand(() -> mElevator.setDesiredPosition("algae", 1, mSwerveSubsystem)));
    // new JoystickButton(mDroneComtroller, Constants.Controllers.selected.ButtonEPort).whileTrue(mAlgaeIntake);
  }

  // Set up auto commands
  private void setUpAuto() {
    NamedCommands.registerCommand("gotToSourceLevel", (new InstantCommand(() -> mElevator.setDesiredPosition("coral", 0, mSwerveSubsystem))));
    NamedCommands.registerCommand("goToLevel2",(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 2, mSwerveSubsystem)))); 
    NamedCommands.registerCommand("goToLevel3",(new InstantCommand(() -> mElevator.setDesiredPosition("coral", 3, mSwerveSubsystem))));
    NamedCommands.registerCommand("goToLevel4", (new InstantCommand(() -> mElevator.setDesiredPosition("coral", 4, mSwerveSubsystem))));
    NamedCommands.registerCommand("coralScore", mAutoScore);
    NamedCommands.registerCommand("reefAlignLeft", new AutoAlign(mSwerveSubsystem, mLimelight, mAprilTagLockLeft));
    NamedCommands.registerCommand("reefAlignRight", new AutoAlign(mSwerveSubsystem, mLimelight, mAprilTagLockRight));
    NamedCommands.registerCommand("coralIntake", mCoralIntake);

  }

  // get selected auto
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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

  public void autoReset() {
    mElevator.resetPositions();
    mAlgae.resetPos();
  }

  public void resetElevator(){
    mElevator.setDesiredPosition("coral", 0, mSwerveSubsystem);
  }

}
