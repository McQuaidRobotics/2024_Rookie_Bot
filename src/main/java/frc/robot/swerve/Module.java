package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Logged;
import edu.wpi.first.math.util.Units;

public class Module implements Logged {

    private final TalonFX angleMotor;
    private final TalonFX velocityMotor;
    private final CANcoder encoder;
    private final StatusSignal<Double> encoderAngle;
    private final StatusSignal<Double> angleMotorAngle;
    private final StatusSignal<Double> velocityMotorVelocity;
    private final StatusSignal<Double> velocityMotorPosition;
    private final int moduleId;
    private Rotation2d lastAngle = new Rotation2d();

    private static final double WHEEL_CIRCUMFERENCE = 0.31918;
    private static final double DRIVE_GEAR_RATIO = 5.35714;
    private static final double ANGLE_GEAR_RATIO = 21.428571428571427;

    public Module(int moduleId, double offset){
        this.moduleId = moduleId;
        this.encoder = new CANcoder(log("encoderId", moduleId + 21), "DriveBus");
        this.velocityMotor = new TalonFX(log("velocityId", (moduleId*2) + 1), "DriveBus");
        this.angleMotor = new TalonFX(log("angleId", (moduleId*2) + 2), "DriveBus");
        this.encoderAngle = encoder.getAbsolutePosition();
        this.angleMotorAngle = angleMotor.getPosition();
        this.velocityMotorVelocity = velocityMotor.getRotorVelocity();
        this.velocityMotorPosition = velocityMotor.getPosition();

        this.angleMotor.getConfigurator()
            .apply(angleMotorConfig());
        this.velocityMotor.getConfigurator()
            .apply(velocityMotorConfig());
        this.encoder.getConfigurator()
            .apply(encoderConfig(offset));
    }

    private TalonFXConfiguration angleMotorConfig() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 3.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        cfg.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.RotorToSensorRatio = ANGLE_GEAR_RATIO;
        cfg.ClosedLoopGeneral.ContinuousWrap = true;

        return cfg;
    }

    private TalonFXConfiguration velocityMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return cfg;
    }
    private CANcoderConfiguration encoderConfig(double offset){
        var cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = offset;
        return cfg;
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations / DRIVE_GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
    }

    public SwerveModulePosition getPosition() {
        double veloPos = driveRotationsToMeters(velocityMotorPosition.getValue());
        double rotPos = angleMotorAngle.getValue();
        return new SwerveModulePosition(veloPos, Rotation2d.fromRotations(rotPos));
    }

    public void applyState(SwerveModuleState state, boolean isOpenLoop){
        log("Preoptimized", state);
        state = SwerveModuleState.optimize(state, Rotation2d.fromRotations(encoderAngle.getValue()));
        log("Postoptimized", state);

        // Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= 0.05 ? lastAngle : state.angle;
        log("Angle", state.angle);

        angleMotor.setControl(new PositionDutyCycle(state.angle.getRotations()));

        // lastAngle = angle;
        if (isOpenLoop){
            double percentOutput = state.speedMetersPerSecond/5.0;
            velocityMotor.set(percentOutput);
        } else {

        }
    }
    public void periodic() {
        log("ActualModulePosition/AngleMotorRads", (angleMotor.getPosition().getValueAsDouble() * Math.PI * 2.0));
        log("ActualModulePosition/AngleMotorVelocityRPM", 60.0*(angleMotor.getVelocity().getValueAsDouble()));
        log("ActualModulePosition/VelocityMotorVelocityRPM", 60.0*(velocityMotorVelocity.getValueAsDouble())/DRIVE_GEAR_RATIO);
        log("ActualModulePosition/VelocityMotorVelocityVoltage", (velocityMotor.getMotorVoltage().getValueAsDouble()));
    }

    @Override
    public String getOverrideName() {
        return "Module[" + moduleId + "]";
    }
}
        