package frc.robot.swerve;

import frc.robot.swerve.Constants.kSwerve;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import monologue.Logged;
import monologue.Annotations.Log;

public class Module implements Logged {
    private final TalonFX driveMotor;
    private final StatusSignal<Double> driveVoltSignal, driveAmpSignal, drivePosSignal, driveVeloSignal;
    private final ControlRequest driveMotorClosedReq;
    private final ControlRequest driveMotorOpenReq;

    private final TalonFX angleMotor;
    private final StatusSignal<Double> angleVoltSignal, angleAmpSignal;
    private final PositionDutyCycle angleMotorReq = new PositionDutyCycle(0)
            .withUpdateFreqHz(0);

    private final CANcoder angleEncoder;
    private final StatusSignal<Double> angleAbsoluteSignal, angleAbsoluteVeloSignal;

    public final int moduleNumber;
    private final double rotationOffset;

    @Log
    private Rotation2d lastAngle = new Rotation2d();
    @Log
    public double driveVeloMPS = 0.0;
    @Log
    public double targetDriveVeloMPS = 0.0;
    @Log
    public double drivePositionMeters = 0.0;
    @Log
    public double driveVolts = 0.0;
    @Log
    public double driveAmps = 0.0;
    @Log
    public double angleVeloRadPS = 0.0;
    @Log
    public double angleAbsoluteRads = 0.0;
    @Log
    public double targetAngleAbsoluteRads = 0.0;
    @Log
    public double angleVolts = 0.0;
    @Log
    public double angleAmps = 0.0;

    public Module(int moduleId, double offset) {

        this.rotationOffset = offset;

        moduleNumber = moduleId;
        angleEncoder = new CANcoder(log("encoderId", moduleId + 21), "DriveBus");
        driveMotor = new TalonFX(log("velocityId", (moduleId*2) + 1), "DriveBus");
        angleMotor = new TalonFX(log("angleId", (moduleId*2) + 2), "DriveBus");

        driveVoltSignal = driveMotor.getMotorVoltage();
        driveAmpSignal = driveMotor.getTorqueCurrent();

        angleVoltSignal = angleMotor.getMotorVoltage();
        angleAmpSignal = angleMotor.getTorqueCurrent();

        angleAbsoluteSignal = angleEncoder.getAbsolutePosition();
        angleAbsoluteVeloSignal = angleEncoder.getVelocity();

        drivePosSignal = driveMotor.getPosition();
        driveVeloSignal = driveMotor.getVelocity();

        driveMotor.getConfigurator().apply(driveMotorConfig(), 1.0);
        angleMotor.getConfigurator().apply(angleMotorConfig(), 1.0);
        angleEncoder.getConfigurator().apply(cancoderConfig(), 1.0);

        driveMotorOpenReq = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);
        driveMotorClosedReq = new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
    }

    private TalonFXConfiguration driveMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;

        cfg.Slot0.kP = 0.27;
        cfg.Slot0.kV = 12.0
                / (kSwerve.MAX_DRIVE_VELOCITY / (kSwerve.WHEEL_CIRCUMFERENCE / kSwerve.DRIVE_GEAR_RATIO));
        cfg.Slot0.kS = 0.15;

        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = kSwerve.SLIP_CURRENT;
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = kSwerve.SLIP_CURRENT;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -kSwerve.SLIP_CURRENT;

        return cfg;
    }

    private TalonFXConfiguration angleMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        cfg.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;

        cfg.Slot0.kP = 11.0;

        cfg.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        cfg.Feedback.RotorToSensorRatio = kSwerve.ANGLE_GEAR_RATIO;
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        return cfg;
    }

    private CANcoderConfiguration cancoderConfig() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = rotationOffset;

        return canCoderConfig;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;
        this.targetAngleAbsoluteRads = angle.getRadians();

        angleMotor.setControl(angleMotorReq.withPosition(angle.getRotations()));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        this.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY;
            driveMotor.setControl(((VoltageOut) driveMotorOpenReq).withOutput(percentOutput * RobotController.getBatteryVoltage()));
        } else {
            double rps = (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE) * kSwerve.DRIVE_GEAR_RATIO;
            log("DriveRPS", rps);
            driveMotor.setControl(((VelocityVoltage) driveMotorClosedReq).withVelocity(rps));
        }
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                this.driveVeloMPS,
                getAngle());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                this.drivePositionMeters,
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(this.angleAbsoluteRads);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    public void periodic() {
        this.angleAbsoluteRads = Units.rotationsToRadians(angleAbsoluteSignal.getValueAsDouble());
        this.angleVeloRadPS = Units.rotationsToRadians(angleAbsoluteVeloSignal.getValueAsDouble());
        this.angleVolts = angleVoltSignal.getValueAsDouble();
        this.angleAmps = angleAmpSignal.getValueAsDouble();

        this.drivePositionMeters = driveRotationsToMeters(drivePosSignal.getValueAsDouble());
        this.driveVeloMPS = driveRotationsToMeters(driveVeloSignal.getValueAsDouble());
        this.driveVolts = driveVoltSignal.getValueAsDouble();
        this.driveAmps = driveAmpSignal.getValueAsDouble();
    }

    public void setVoltageOut(double volts, Rotation2d angle) {
        setAngle(new SwerveModuleState(0.0, angle));
        driveMotor.setVoltage(volts);
    }
}
