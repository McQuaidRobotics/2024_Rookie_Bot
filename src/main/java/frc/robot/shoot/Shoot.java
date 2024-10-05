package frc.robot.shoot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shoot {
    private final TalonFX lead;
    private final TalonFX follow;

    public Shoot() {

        this.lead = new TalonFX(0);
        this.follow = new TalonFX(1);
        this.lead.getConfigurator()
            .apply(leadMotorConfig());
        this.follow.getConfigurator()
            .apply(followMotorConfig());
        
        // our motors are set up now shoot.
        // set the motors RPM to launch
        
        double percentOutput = 100.0;

        this.lead.setControl(new VoltageOut(12 * percentOutput));
        this.follow.setControl(new Follower(0, true));

    } 
    public void stopShooting() {
        lead.stopMotor();
        follow.stopMotor();
    }
    private TalonFXConfiguration leadMotorConfig() {
        var cfg = new TalonFXConfiguration();
        
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }

    private TalonFXConfiguration followMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        

        return cfg;
    }
}
