package frc.robot.shoot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.Intake;

public class Shoot extends SubsystemBase {
    private final TalonFX motor;
    private final StatusSignal<Double> ampSignalShooter;
    private final Intake intake;
    public Shoot(Intake intake) {
        this.motor = new TalonFX(10, "DriveBus");
        this.motor.getConfigurator()
            .apply(leadMotorConfig());
        this.ampSignalShooter = motor.getTorqueCurrent();
        this.intake = intake;
    }

    public void runShooterRpm(double velocity) {
        this.motor.setControl(new VelocityVoltage(velocity / 60.0));
    }

    public void runShooterRads(double velocity) {
        this.motor.setControl(new VelocityVoltage(velocity / (Math.PI * 2)));
    }
    public boolean hasSpunUp() {
        return motor.getVelocity().getValueAsDouble() >= 5600;
    }

    public void stopShooting() {
        motor.stopMotor();
    }

    public double getVelocity() {
        var velocitySignal = motor.getVelocity();
        double velocity = velocitySignal.getValue();

        return velocity;
    }
    public boolean hasNoteLeft(){
        return ampSignalShooter.refresh().getValueAsDouble() < 100; //No note because its easy to move
    }

    private TalonFXConfiguration leadMotorConfig() {
        var cfg = new TalonFXConfiguration();
        
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }
    public Command spinUp() {
        return run(() ->this.runShooterRpm(5600.0))
            .until(this::hasSpunUp)
            .withName("spinUp");
    }
    public Command shootNote() {
        return this.run(() -> this.spinUp())
            .andThen(intake.transferNote())
            .until(this::hasNoteLeft) // will spin the motor untill note has left and will stop spinning if note is not in
            .withName("shootNote"); // gives name shoot note to command for logging
    }
}
