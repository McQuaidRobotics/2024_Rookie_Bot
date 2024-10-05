package frc.robot.shoot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {
    private final TalonFX motor;

    public Shoot() {
        this.motor = new TalonFX(10, "DriveBus");
        this.motor.getConfigurator()
            .apply(leadMotorConfig());
        
        // our motors are set up now shoot.
        // set the motors RPM to launch
        
    

    }
    public void shootNote() {
        double percentOutput = 1.0;

        this.motor.setControl(new VoltageOut(12.0 * percentOutput));
    } 
    public void stopShooting() {
        motor.stopMotor();
    }
    private TalonFXConfiguration leadMotorConfig() {
        var cfg = new TalonFXConfiguration();
        
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }
}
