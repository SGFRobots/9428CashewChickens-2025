// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {        
    public static final String ModuleNameSim = "Swerve Modules";
    public static final boolean fieldOriented = false;
    
    public static final class MotorPorts {
        // CAN IDs of driving motors
        public static final int kFLDriveMotorID = 4;
        public static final int kBLDriveMotorID = 1;
        public static final int kFRDriveMotorID = 3;
        public static final int kBRDriveMotorID = 2;

        // CAN IDs of turning motors
        public static final int kFLTurningMotorID = 3;
        public static final int kBLTurningMotorID = 11;
        public static final int kFRTurningMotorID = 12;
        public static final int kBRTurningMotorID = 9;

        // CAN IDs of Elevator
        public static final int kLElevator = 2;
        public static final int kRElevator = 7;

        // CAN IDs of Coral scorer
        public static final int kLCoral = 5;
        public static final int kRCoral = 6;

        // CAN IDs of Algae scorer
        public static final int kAlgaePosMotorID = 13;
        public static final int kAlgaeWheelMotor = 10;
        
        // CAN IDs of CANCoders
        public static final int kFLDriveAbsoluteEncoderID = 8;
        public static final int kBLDriveAbsoluteEncoderID = 5;
        public static final int kFRDriveAbsoluteEncoderID = 7;
        public static final int kBRDriveAbsoluteEncoderID = 6;

        // Gyro
        public static final int kGyroPort = 2;
    }

    // Reversed motors
    public static final class Reversed {
        // Turning motors
        public static final boolean kFLTurningReversed = true;
        public static final boolean kBLTurningReversed = true;
        public static final boolean kFRTurningReversed = true;
        public static final boolean kBRTurningReversed = true;
    
        // Driving motors
        public static final boolean kFLDriveReversed = false;
        public static final boolean kBLDriveReversed = false;
        public static final boolean kFRDriveReversed = false;
        public static final boolean kBRDriveReversed = false;

        // Turning encoders
        public static final boolean kFLTurningEncoderReversed = false;
        public static final boolean kBLTurningEncoderReversed = true;
        public static final boolean kFRTurningEncoderReversed = true;
        public static final boolean kBRTurningEncoderReversed = true;
    
        // Driving encoders
        public static final boolean kFLDriveEncoderReversed = false;
        public static final boolean kBLDriveEncoderReversed = true;
        public static final boolean kFRDriveEncoderReversed = true;
        public static final boolean kBRDriveEncoderReversed = false;
    
        // CANCoders
        public static final boolean kFLDriveAbsoluteEncoderReversed = false;
        public static final boolean kBLDriveAbsoluteEncoderReversed = false;
        public static final boolean kFRDriveAbsoluteEncoderReversed = false;
        public static final boolean kBRDriveAbsoluteEncoderReversed = false;
    }

    // Physical and mechanical variables
    public static final class Mechanical {

        // Robot's physical measurements
        public static final double kWheelRadiusMeters = 0.0508;
        public static final double kWheelDiameterMeters = kWheelRadiusMeters * 2;
        public static final double kDriveMotorGearRatio = 6.12;
        public static final double kTurningMotorGearRatio = 150/7;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kDriveEncoderResolution = 2048;
        public static final double kTurningEncoderResolution = 42;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kEncoderCPR = 1024;
        public static final double kDistancePerPulse = (kWheelCircumferenceMeters) / kEncoderCPR;
        // The SysId tool provides a convenient method for obtaining these values for your robot.
        // the values being the volt related lines below 
        public static final double kVoltSecondPerRadian = 3.41;
        public static final double kVoltSecondsSquaredPerRadian = 0.111;
        public static final DCMotor kDriveGearBox = DCMotor.getKrakenX60(1);
        public static final DCMotor kTurnGearBox = DCMotor.getKrakenX60(1);

        
        // Distance between right and left wheels (in meters)
        public static final double kRobotWidthMeters = 0.6731;
        // Distance between front and back wheels (in meters)
        public static final double kRobotLengthMeters = 0.6731;

        // CANCoders' offsets
        public static final double kFLDriveAbsoluteEncoderOffset = -0.20215; 
        public static final double kBLDriveAbsoluteEncoderOffset = -0.48193;
        public static final double kFRDriveAbsoluteEncoderOffset = -0.16211; 
        public static final double kBRDriveAbsoluteEncoderOffset = -0.28857; 

        // Module Positions on Robot
        public static final Translation2d[] kModulePositions = {
            new Translation2d(kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(kRobotLengthMeters / 2, -kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, kRobotWidthMeters / 2),
            new Translation2d(-kRobotLengthMeters / 2, -kRobotWidthMeters / 2)
        };
        // Kinematics map of robot
        // FL, FR, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            kModulePositions[0],
            kModulePositions[1],
            kModulePositions[2],
            kModulePositions[3]);
        
        // Deadzone
        public static final double kDeadzone = 0.1;
        public static final double AprilTagDeadzone = 3;
    
        // Speeds and Accelerations
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0292;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 12.119;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 10;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;

        // Elevator
        // Coral Heights
        public static final double ElevatorLowestHeight = 0;
        public static final double ElevatorLevelZeroHeight = -80;
        public static final double ElevatorLevelOneHeight = -175;
        public static final double ElevatorLevelTwoHeight = -362;
        public static final double ElevatorMaxHeight = -610;
        // Algae heights
        public static final double ElevatorAlgaeOneHeight = -270;
        public static final double ElevatorAlgaeTwoHeight = -475;

        // Algae Arm positions
        public static final double AlgaeUpPos = 6;

        // LED
        public static final int LEDChannel = 0;
    }

    public static final class AprilTags {
        // x, distance, yaw
        // public static final double[] leftCoral = {16.6807, 2.5845, -41.7339};
        public static final double[] leftCoral = {17.65, 0.22, -43};
        // public static final double[] rightCoral = {-19.2193, 15.10175, -44.1119};
        public static final double[] rightCoral = {-20.49, 0.25, -35};
        // public static final double[] rightCoral = {-12.0838, 0.227, -45.3921};

        public static final double xErrorAllowed = 0.6;
        public static final double distanceErrorAllowed = 0.02;
        public static final double yawErrorAllowed = 1.5;
    }

    // Controller ports
    public static final class Controllers {
        // Controller ports
        public static final int DrivingControllerPort = 0;
        public static final int XBoxControllerPort = 1;

        // Black drone controller
        public static final class backup {
            // Joysticks and triggers
            public static final int LeftXPort = 0;
            public static final int LeftYPort = 1;
            public static final int RightXPort = 2;
            public static final int RightYPort = 3;
            
            // Dials
            public static final int s1DialPort = 4;
            public static final int s2DialPort = 5;
            
            // Switches
            public static final int SwitchE = 6;
            public static final int SwitchF = 7;
            public static final int UpperB = 1;
            public static final int MiddleB = 2;
            public static final int LowerB = 3;
            public static final int UpperC = 4;
            public static final int MiddleC = 5;
            public static final int LowerC = 6;
            
            // Buttons
            public static final int ButtonAPort = 7;
            public static final int ButtonDPort = 8;
        }
        
        // White drone controller
        public static final class selected {
            // Joysticks and triggers
            public static final int LeftXPort = 3;
            public static final int LeftYPort = 2;
            public static final int RightXPort = 0;
            public static final int RightYPort = 1;

            // Switches
            public static final int SwitchA = 4;
            public static final int SwitchB = 5;
            public static final int SwitchC = 7;
            public static final int SwitchD = 6;

            // Buttons
            public static final int ButtonEPort = 1;
            public static final int ButtonFPort = 2;
            public static final int buttonB = 1;
            public static final int buttonA = 2;
        }

        public static final class XBox {
            // Joysticks
            public static final int LeftXPort = 0;
            public static final int LeftYPort = 1;
            public static final int RightXPort = 4;
            public static final int RightYPort = 5;
            public static final int LeftJoystickButton = 9;
            public static final int RightJoystickButton = 10;

            // Triggers
            public static final int LeftTriggerPort = 2;
            public static final int RightTriggerPort = 3;

            // Bumpers
            public static final int LeftBumper = 5;
            public static final int RightBumper = 6;

            // Buttons
            public static final int buttonB = 1;
            public static final int buttonA = 2;
            public static final int buttonY = 3;
            public static final int buttonX = 4;
            public static final int buttonPlus = 8;
            public static final int buttonMinus = 7;

            // D-Pad
            public static final int DPadUp = 0;
            public static final int DPadRight = 90;
            public static final int DPadDown = 180;
            public static final int DPadLeft = 270;
        }
  }
}