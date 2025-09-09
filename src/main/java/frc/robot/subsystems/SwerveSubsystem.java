package frc.robot.subsystems;

import static frc.robot.Constants.Mechanical.kModulePositions;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Limelight.LimelightHelpers;

import com.studica.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {
    // Make instances of all 4 modules
    private final Module[] modules = {
            // Front Left
            new Module(
                    Constants.MotorPorts.kFLDriveMotorID,
                    Constants.MotorPorts.kFLTurningMotorID,
                    Constants.Reversed.kFLDriveReversed,
                    Constants.Reversed.kFLTurningReversed,
                    Constants.MotorPorts.kFLDriveAbsoluteEncoderID,
                    Constants.Mechanical.kFLDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kFLDriveAbsoluteEncoderReversed),

            // Front Right
            new Module(
                    Constants.MotorPorts.kFRDriveMotorID,
                    Constants.MotorPorts.kFRTurningMotorID,
                    Constants.Reversed.kFRDriveReversed,
                    Constants.Reversed.kFRTurningReversed,
                    Constants.MotorPorts.kFRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kFRDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kFRDriveAbsoluteEncoderReversed),

            // Back Left
            new Module(
                    Constants.MotorPorts.kBLDriveMotorID,
                    Constants.MotorPorts.kBLTurningMotorID,
                    Constants.Reversed.kBLDriveReversed,
                    Constants.Reversed.kBLTurningReversed,
                    Constants.MotorPorts.kBLDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBLDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kBLDriveAbsoluteEncoderReversed),
            
            // Back Right
            new Module(
                    Constants.MotorPorts.kBRDriveMotorID,
                    Constants.MotorPorts.kBRTurningMotorID,
                    Constants.Reversed.kBRDriveReversed,
                    Constants.Reversed.kBRTurningReversed,
                    Constants.MotorPorts.kBRDriveAbsoluteEncoderID,
                    Constants.Mechanical.kBRDriveAbsoluteEncoderOffset,
                    Constants.Reversed.kBRDriveAbsoluteEncoderReversed),
            

    };

    private boolean findingPos = false;

    // Positions stored in mOdometer
    private final SwerveDriveOdometry mOdometer;
    private RobotConfig pathPlannerConfig;

    private final AHRS mGyro;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // Simulated field
    public static final Field2d mField2d = new Field2d();
    // Simulated modules
    Pose2d[] mModulePose = {
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    private SwerveDrivePoseEstimator poseEstimator;

        
    // Constructor
    public SwerveSubsystem() {
        
        mGyro = new AHRS(AHRS.NavXComType.kUSB1);
        zeroHeading();

        mOdometer = new SwerveDriveOdometry(Constants.Mechanical.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d(7.598, 4.070, getRotation2d()));

        LimelightHelpers.setPipelineIndex("limelight-left", 0);
        LimelightHelpers.setPipelineIndex("limelight-right", 0);
        
        // Simulated field
        SmartDashboard.putData("Field", mField2d);

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Mechanical.kDriveKinematics, getRotation2d(), getModulePositions(), getPose());

        pathPlannerConfig = new RobotConfig(56.155, 7.396, new ModuleConfig(0.051, 4.7, 0.96, DCMotor.getKrakenX60(1), 6.12, 60, 1), kModulePositions);
        try{
            pathPlannerConfig = RobotConfig.fromGUISettings();
            System.out.println("---------configured from GUI---------");
        } catch (Exception e) {
            e.printStackTrace(); // error handling 
            System.out.println("---------failed to configure from GUI---------");
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
            ),
            pathPlannerConfig, // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
        );
    }
    
    public void zeroHeading() {
        mGyro.zeroYaw();
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Blue) {
            mGyro.setAngleAdjustment(180);
        } else {
            mGyro.setAngleAdjustment(0);
        }
    }
    
    public double getAngle() {
        return -mGyro.getAngle();
    }
    
    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360);
        // return MathUtil.inputModulus(mGyro.getAngle(), 0, 360);
    }
    
    public Rotation2d getGyroRotation2d() {
        return mGyro.getRotation2d();
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }
    
    // Get position of robot based on odometer
    public Pose2d getPose() {
        return mOdometer.getPoseMeters();
    }
    
    public void resetPose(Pose2d pose) {
        mOdometer.resetPose(pose);
    }
    
    @Override
    public void periodic() {
        // Update modules' positions
        mOdometer.update(getRotation2d(), getModulePositions());
        // updateOdometry();
        // resetOdometry(poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Odometry x", mOdometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry y", mOdometer.getPoseMeters().getY());

        // Update Pose for swerve modules - Position of the rotation and the translation matters
        for (int i = 0; i < modules.length; i++) {
            // No gyro - new Rotation2d() instead
            Translation2d updatedModulePosition = kModulePositions[i].rotateBy(getRotation2d()).plus(getPose().getTranslation());
            // Module heading is the angle relative to the chasis heading
            mModulePose[i] = new Pose2d(updatedModulePosition, modules[i].getState().angle.plus(getRotation2d()));

            modules[i].periodic();
        }

        // Sets robot and modules positions on the field
        mField2d.setRobotPose(getPose());
        // mField2d.setRobotPose(poseEstimator.getEstimatedPosition());
        // mField2d.getObject(Constants.ModuleNameSim).setPoses(mModulePose);

        // Logs in Swerve Tab
        // double loggingState[] = {
        //         modules[0].getState().angle.getDegrees(), modules[0].getState().speedMetersPerSecond,
        //         modules[1].getState().angle.getDegrees(), modules[1].getState().speedMetersPerSecond,
        //         modules[2].getState().angle.getDegrees(), modules[2].getState().speedMetersPerSecond,
        //         modules[3].getState().angle.getDegrees(), modules[3].getState().speedMetersPerSecond
        // };

        // Debug telemetry
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Angle", mGyro.getAngle());
        SmartDashboard.putNumber("xSpeed", getRobotRelativeSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("ySpeed", getRobotRelativeSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("turningSpeed", getRobotRelativeSpeeds().omegaRadiansPerSecond);
        // SmartDashboard.putString("Gyro", getGyroRotation2d().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // SmartDashboard.putNumberArray("SwerveModuleLOGGINGStates", loggingState);
        SmartDashboard.putNumber("rotation2d", getRotation2d().getDegrees());
        SmartDashboard.putData("Field", mField2d);
        SmartDashboard.putNumber("raw gyro z", mGyro.getRawGyroZ());
        SmartDashboard.putNumber("velocity z", mGyro.getVelocityZ());
    }

    // Reset odometer
    public void resetOdometry(Pose2d pose) {
        mOdometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    // Update odometry based on limelight
    public void updateOdometry() {
        // boolean update = ()
        poseEstimator.update(getRotation2d(), getModulePositions());
        Pair<Pose2d,Double> leftLimelightPos = getLimelightPose("limelight-left");
        Pair<Pose2d,Double> rightLimelightPos = getLimelightPose("limelight-right");

        Pose2d leftLLPose = leftLimelightPos.getFirst();
        Pose2d rightLLPose = rightLimelightPos.getFirst();

        double leftLLtimeStamp = leftLimelightPos.getSecond();
        double rightLLtimeStamp = rightLimelightPos.getSecond();
        
        if (leftLLPose != null) {
            SmartDashboard.putNumber("rotaion?", leftLLPose.getRotation().getDegrees());
        }
        if (Math.abs(mGyro.getRawGyroZ()) < 720) {
            if ((leftLLPose != null) && (rightLLPose != null)) {
                Pose2d botpose = new Pose2d(new Translation2d((leftLLPose.getX() + rightLLPose.getX()) / 2, (leftLLPose.getY() + rightLLPose.getY()) / 2), poseEstimator.getEstimatedPosition().getRotation());
                double botTimeStamp = (leftLLtimeStamp + rightLLtimeStamp) / 2;
                poseEstimator.addVisionMeasurement(botpose, botTimeStamp);
            }

            if ((leftLLPose != null) && (rightLLPose == null)) {
                poseEstimator.addVisionMeasurement(leftLLPose, leftLLtimeStamp);
            }

            if ((leftLLPose == null) && (rightLLPose != null)) {
                poseEstimator.addVisionMeasurement(rightLLPose, rightLLtimeStamp);
            }
        }
    }
    
    public Pair<Pose2d,Double> getLimelightPose(String limelightName) {
        var alliance = DriverStation.getAlliance();
        LimelightHelpers.SetRobotOrientation(limelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate megaTag2 = (alliance.get() == DriverStation.Alliance.Blue) ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName) : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
        if ((megaTag2 != null) && (Math.abs(mGyro.getRawGyroZ()) < 720) && (megaTag2.tagCount != 0)) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            return Pair.of(megaTag2.pose, megaTag2.timestampSeconds);
        }
        return Pair.of(null,0.0);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition() };
    }

    // Stop the robot completely
    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].stop();
        }
    }

    // Drive the robot
    public void drive(ChassisSpeeds newChassisSpeed) {
        this.chassisSpeeds = newChassisSpeed;
        // Convert chassis speeds to each module states
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = Constants.Mechanical.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        // Cap max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond);

        // Move each module
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i], getFindingPos());
        }
    }

    public void toggleFastMode(boolean fastOn) {
        Module.fast = fastOn;
    }

    public boolean getFastMode() {
        return Module.fast;
    }

    public void toggleSlowMode(boolean slowOn) {
        Module.slow = slowOn;
    }

    public boolean getSlowMode() {
        return Module.slow;
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Mechanical.kDriveKinematics.toChassisSpeeds(
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState()
        );
    }

    // Test one module at a time
    public void driveIndividualModule(double speed, double rotation) {
        modules[3].driveIndividually(speed, rotation);
    }

    // Reset modules rotations to 0
    public void resetEncoders() {
        for (Module module : modules) {
            module.resetting = true;
        }
    }
    
    // Reset modules rotations to 0
    public void stopReset() {
        for (Module module : modules) {
            module.resetting = false;
        }
    }

    // Check if all modules are done resetting angles
    public boolean checkEncoderResetted() {
        for (Module module : modules) {
            if (module.resetting) {
                return false;
            }
        }
        return true;
    }

    public void toggleFindingPos() {
        findingPos = !findingPos;
    }

    public boolean getFindingPos() {
        return findingPos;
    }
}
