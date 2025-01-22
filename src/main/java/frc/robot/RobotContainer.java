// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Limelight;
// Subsystems and commands
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.LimeLightControl;
import frc.robot.commands.ResetRotations;
import frc.robot.commands.SpeedControl;


public class RobotContainer {

  // Instances of controllers
  // public final XboxController mController;
  public static final GenericHID mController = new GenericHID(0);
  private final SendableChooser<Command> autoChooser;
  // Subsystems and commands
  private final SwerveSubsystem mSwerveSubsystem;
  private final ResetRotations mResetRotations;
  private final SpeedControl mSpeeds;
  private final Limelight mLimelight;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems and commands
    mSwerveSubsystem = new SwerveSubsystem();
    mSwerveSubsystem.setDefaultCommand(new SwerveJoystick(mSwerveSubsystem, mController));
    mResetRotations = new ResetRotations(mSwerveSubsystem);
    mSpeeds = new SpeedControl(mSwerveSubsystem);
    mLimelight = new Limelight();
    mLimelight.setDefaultCommand(new LimeLightControl(mLimelight));

    // Autonomous
    autoChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    mController.getType();

    System.out.println(mController.getName() + "  " + mController.getType());
    SmartDashboard.putString("name", mController.getName());
    SmartDashboard.putString("type", mController.toString());
    // Bind buttons and commands
    configureButtonBindings();
  }

  // Assign buttons to commands
  private void configureButtonBindings() {
    new JoystickButton(mController, Constants.Controllers.selected.ButtonAPort).onTrue(mResetRotations);
    new JoystickButton(mController, Constants.Controllers.selected.UpperC).whileTrue(mSpeeds.fast);
    new JoystickButton(mController, Constants.Controllers.selected.LowerC).whileTrue(mSpeeds.slow);
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
