package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final TalonFX pivot;
    private final TalonFX top_Intake;
    private final TalonFX bot_Intake;

    public Intake() {
        this.pivot = new TalonFX(11, "DriveBus");
        this.top_Intake = new TalonFX(12, "DriveBus");
        this.bot_Intake = new TalonFX(13, "DriveBus");
        this.pivot.getConfigurator()
            .apply(pivotMotorConfiguration());
        this.top_Intake.getConfigurator()
            .apply(topMotorConfiguration());
        this.bot_Intake.getConfigurator()
            .apply(botMotorConfiguration());
    }
    public void intakeNote() { // should run intake motors in opposite directions to pick up note I do not know how to make it stop if note is in
        double percentOutput = 1.0;
        this.top_Intake.setControl(new VoltageOut(12.0 * percentOutput));
        this.bot_Intake.setControl(new Follower(12, true));
    }

    public void expellNote() {
        double percentOutput = -1.0;
        this.top_Intake.setControl(new VoltageOut(12.0 * percentOutput));
        this.bot_Intake.setControl(new Follower(12, true));
    }

    public void transferNote(boolean isInChamber) {

        if(isInChamber){
            double percentOutput = .5;
            this.top_Intake.setControl(new VoltageOut(12.0 * percentOutput));
            this.bot_Intake.setControl(new Follower(12, true));
        }

    }
    // TODO 
    //CHECK IF NOTE IS IN THE CHAMBER.
    // Ensure that -12.0 volts will spin the motors backwards.
    // Test Configs

    private TalonFXConfiguration pivotMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }
    private TalonFXConfiguration botMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }
    private TalonFXConfiguration topMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }
}

