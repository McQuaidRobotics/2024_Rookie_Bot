package frc.robot.shoot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
    private final TalonFX rollerMotor;
    private final StatusSignal<Double> veloSignalShooter;

    private final VelocityVoltage veloVolt = new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

    private double targetVelocity = 20000;

    public Shooter() {
        this.rollerMotor = new TalonFX(10);
        this.rollerMotor.getConfigurator()
            .apply(rollerMotorConfig());
        this.veloSignalShooter = rollerMotor.getVelocity();

        // this.setDefaultCommand(this.run(() -> runShooterRpm(6000)));
    }

    private TalonFXConfiguration rollerMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.Slot0.kP = 0.2;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.15;
        cfg.Slot0.kV = 0.113;

        cfg.MotorOutput.PeakReverseDutyCycle = 0.0;
        cfg.Voltage.PeakReverseVoltage = 0.0;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;


        return cfg;
    }

    public void runShooterRpm(double rpm) {
        targetVelocity = rpm;
        log("targetRPM", targetVelocity);
        this.rollerMotor.setControl(veloVolt.withVelocity(rpm / 60.0));
    }

    public void runShooterRads(double velocity) {
        targetVelocity = Units.radiansPerSecondToRotationsPerMinute(velocity);
        this.runShooterRpm(targetVelocity);
    }

    public boolean hasSpunUp() {
        return MathUtil.isNear(targetVelocity, getVelocityRpm(), 75.0);
    }

    public boolean hasSpunUp(double rpm) {
        return MathUtil.isNear(rpm, getVelocityRpm(), 75.0);
    }

    public void stopShooting() {
        rollerMotor.stopMotor();
    }

    public double getVelocityRpm() {
        return veloSignalShooter.refresh().getValueAsDouble() * 60.0;
    }

    public Command spinUpRPM(DoubleSupplier doubleSupplier) {
        return this.run(() -> this.runShooterRpm(doubleSupplier.getAsDouble()))
            .finallyDo(() -> this.runShooterRpm(0.0));
    }
    @Override
    public void periodic() {
        log("shooterRPM", getVelocityRpm());
        log("hasSpunUp", hasSpunUp());
    }

}
