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
    private final TalonFX roller_Motor;
    private final StatusSignal<Double> veloSignalShooter;

    private double targetVelocity = 0.0;

    public Shooter() {
        this.roller_Motor = new TalonFX(10);
        this.roller_Motor.getConfigurator()
            .apply(rollerMotorConfig());
        this.veloSignalShooter = roller_Motor.getVelocity();
    }

    private TalonFXConfiguration rollerMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.15;
        cfg.Slot0.kV = 0.012;


        return cfg;
    }

    public void runShooterRpm(double velocity) {
        targetVelocity = velocity;
        this.roller_Motor.setControl(new VelocityVoltage(velocity / 60.0));
    }

    public void runShooterRads(double velocity) {
        targetVelocity = Units.radiansPerSecondToRotationsPerMinute(velocity);
        this.runShooterRpm(targetVelocity);
    }

    public boolean hasSpunUp() {
        return MathUtil.isNear(targetVelocity, getVelocityRpm(), 3.0);
    }

    public void stopShooting() {
        roller_Motor.stopMotor();
    }

    public double getVelocityRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(veloSignalShooter.refresh().getValueAsDouble());
    }

    public Command spinUpRPM() {
        return this.run(() -> this.runShooterRpm(5600.0));
    }

    }

        