package frc.robot.shoot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
    private final TalonFX motor;
    private final StatusSignal<Double> veloSignalShooter;

    private double targetVelocity = 0.0;

    public Shooter() {
        this.motor = new TalonFX(10);
        this.motor.getConfigurator()
            .apply(leadMotorConfig());
        this.veloSignalShooter = motor.getVelocity();
    }

    private TalonFXConfiguration leadMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }

    public void runShooterRpm(double velocity) {
        this.targetVelocity = velocity;
        this.motor.setControl(new VelocityVoltage(velocity / 60.0));
    }

    public void runShooterRads(double velocity) {
        this.targetVelocity = Units.radiansPerSecondToRotationsPerMinute(velocity);
        this.runShooterRpm(targetVelocity);
    }

    public boolean hasSpunUp() {
        return MathUtil.isNear(targetVelocity, getVelocityRpm(), 3.0);
    }

    public void stopShooting() {
        motor.stopMotor();
    }

    public double getVelocityRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(veloSignalShooter.refresh().getValueAsDouble());
    }

    public Command spinUpRpm(double rpm) {
        return spinUpRpm(() -> rpm);
    }

    public Command spinUpRpm(DoubleSupplier rpmSupplier) {
        return run(() -> this.runShooterRpm(rpmSupplier.getAsDouble()))
            .until(this::hasSpunUp)
            .withName("spinUp");
    }
    @Override
    public void periodic() {
        log("ShooterVelocity", motor.getVelocity().getValueAsDouble() * 60.0);
        log("ShooterAmperage", motor.getStatorCurrent().getValueAsDouble());
        log("ShooterPosition", motor.getPosition().getValueAsDouble());
        log("TargetVelocity", targetVelocity);
    }
}
