package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {

    // motors and encoders
    private final TalonFX mDriveMotor;
    private final SparkMax mTurnMotor;
    // private ClosedLoopGeneralConfigs configs;

    // PID controllers - feedback method
    private final PIDController turningPID;

    // aBSOLUTE ENCODER - knows where the wheels are facing at all times
    private final CANcoder absoluteEncoder;
    private final boolean AbsoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public boolean resetting;
    public static boolean fast;
    public static boolean slow;

    private SwerveModuleState currentSpeeds;

    // turning and driving power/distance
    private double turnOutput;
    private double driveOutput;
    private double driveDistance;
    private double turnDistance;
    public Field2d field;

    // Constructores
    public Module(
        int pDrivePort, int pTurnPort, boolean pDriveReversed, boolean pTurnReversed,
        int pAbsoluteEncoderPort, double pAbsoluteEncoderOffset, boolean pAbsoluteEncoderReversed) {

            // Motors
            mDriveMotor = new TalonFX(pDrivePort);
            mTurnMotor = new SparkMax(pTurnPort, MotorType.kBrushless);
            // mTurnMotor.setInverted(pTurnReversed);
            mDriveMotor.setNeutralMode(NeutralModeValue.Brake);

            // Absolute Encoder
            absoluteEncoder = new CANcoder(pAbsoluteEncoderPort);
            AbsoluteEncoderReversed = pAbsoluteEncoderReversed;
            absoluteEncoderOffset = pAbsoluteEncoderOffset;
            
            //PID Controller - change PID values when get feedback
            turningPID = new PIDController(0.008, 0, 0);
            turningPID.enableContinuousInput(-Math.PI, Math.PI); // minimize rotations to 180
            // P = Proportional error
            // I = Integral of errors
            // D = Derivative (rate of change) of P (slow when get closer)
            fast = true;
            slow = false;

            currentSpeeds = new SwerveModuleState(Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond, new Rotation2d(getCurrentAngleRad()));
            PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
            field = new Field2d();
            // SmartDashboard.putData("Field", field);
    }
 
    // Return all data of the position of the robot - type SwerveModuleState
    public SwerveModuleState getState() {
        return currentSpeeds;
    }

    // Return all data of the position of the robot - type SwerveModulePosition
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveDistance, new Rotation2d(turnDistance)
        );
    }

    // Move module
    public void setDesiredState(SwerveModuleState pNewState) {
        // If normal
        if (!resetting) {
            // Don't move back to 0 after moving
            if (pNewState.speedMetersPerSecond < 0.001) {
                stop();
                return;
            }

            // Optimize angle
            pNewState.optimize(new Rotation2d(getCurrentAngleRad()));
            currentSpeeds = pNewState;   

            // Set power to motor
            driveOutput = (currentSpeeds.speedMetersPerSecond * Math.cos(turningPID.getError())) / Constants.Mechanical.kPhysicalMaxSpeedMetersPerSecond;
            turnOutput = turningPID.calculate(getCurrentAngleRad(), currentSpeeds.angle.getRadians()) * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond * 2;
            speedChange();
            
            mDriveMotor.set(driveOutput);
            mTurnMotor.set(turnOutput*2); 
            
            // Telemetry
            // SmartDashboard.putNumber("before" + mDriveMotor.getDeviceID(), pNewState.angle.getDegrees());
            // SmartDashboard.putNumber("turn " + mDriveMotor.getDeviceID() + " output", turnOutput);
            // SmartDashboard.putNumber("drive " + mDriveMotor.getDeviceID() + " output", driveOutput);
            // SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", currentSpeeds.toString());
            
        } else {
            // Reset wheel rotations
            resetRotation();
        }
    }

    public void speedChange() {
        driveOutput *= fast ? 2 : 1;
        driveOutput *= slow ? 0.15 : 1;
    } 
    
    // Read current module angle in Radians
    public double getCurrentAngleRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2;
        angle *= (AbsoluteEncoderReversed) ? -1 : 1;
        return MathUtil.angleModulus(angle - (absoluteEncoderOffset * Math.PI * 2));
    }
    
    // Test one module at a time
    public void driveIndividually(double speed, double rotation) {
        mDriveMotor.set(speed);
        mTurnMotor.set(rotation);
    }
    
    // Telemetry
    public void periodic() {
        // if (mDriveMotor.getDeviceID() == 4) {
        //     // SmartDashboard.putNumber("mv: drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getMotorVoltage().getValueAsDouble());
        //     // SmartDashboard.putNumber("sv: drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getSupplyVoltage().getValueAsDouble());
        //     SmartDashboard.putNumber("mv: turn motor" + mTurnMotor.getDeviceId(), turnOutput);
        //     // SmartDashboard.putNumber("drive motor" + mDriveMotor.getDeviceID(), mDriveMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("currentAngle" + mDriveMotor.getDeviceID(), absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        //     SmartDashboard.putBoolean("resetting", resetting);
        // }
        // SmartDashboard.putNumber("currentAngle" + mDriveMotor.getDeviceID(), Math.toDegrees(getCurrentAngleRad()));
        // System.out.println("drive: " + mDriveMotor.getDeviceID() + "  turn: " + mTurnMotor.getDeviceId() + "  encoder: " + absoluteEncoder.getDeviceID());
    }

    // Turn module back to 0 position
    public void resetRotation() {
        turnOutput = turningPID.calculate(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2, absoluteEncoderOffset * Math.PI * 2) * Constants.Mechanical.kPhysicalMaxAngularSpeedRadiansPerSecond;
        if (Math.abs(turnOutput) < 0.01) {
            resetting = false;
            return;
        }
        mTurnMotor.set(turnOutput);
        mDriveMotor.set(0);  
    }

    // Stop all motors
    public void stop() {
        mDriveMotor.set(0);
        mTurnMotor.set(0);
    }
}