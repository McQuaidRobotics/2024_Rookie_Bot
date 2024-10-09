package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final TalonFX arm;
    private final TalonFX top_Intake;
    private final VoltageOut controlReqVolts = new VoltageOut(0.0).withUpdateFreqHz(0);
    private final PositionVoltage controlReqPosistion = new PositionVoltage(0.0).withUpdateFreqHz(0);
    private final boolean isArmCurrentTripped = false;
    private final StatusSignal<Double> ampSignalArm, ampSignalRoller;
    private final double BACK_HARD_STOP = 0.0;
    private final double ARM_RATIO = 0.2;
    // private final TalonFX bot_Intake;

    public Intake() {
        this.arm = new TalonFX(11, "DriveBus");
        this.top_Intake = new TalonFX(12, "DriveBus");
        this.ampSignalArm = arm.getTorqueCurrent();
        this.ampSignalRoller = this.top_Intake.getTorqueCurrent();
        // this.bot_Intake = new TalonFX(13, "DriveBus");
        this.arm.getConfigurator()
            .apply(pivotMotorConfiguration());
        this.top_Intake.getConfigurator()
            .apply(topMotorConfiguration());
        // this.bot_Intake.getConfigurator()
        //     .apply(botMotorConfiguration());
    }
    public void setRollerVoltageOut(double volts) {
        top_Intake.setControl(controlReqVolts.withOutput(volts));
    }
    public void setArmPosition(double position){
        this.arm.setControl(controlReqPosistion.withPosition(position));
    }
    public void setArmVoltageOut(double volts) {
        this.arm.setControl(controlReqVolts.withOutput(volts));
    }
    public boolean isArmCurrentTripped() {
        
        return ampSignalArm.refresh().getValueAsDouble() > 60.0;
        // if it is too hard to move the motors then return it
    }
    public  boolean isRollerCurrentTripped() {
        return ampSignalRoller.refresh().getValueAsDouble() > 115.0;
    }
    public void homeArmHere() {
        arm.setPosition(BACK_HARD_STOP / ARM_RATIO);
    }
    public boolean hasNoteLeft() {
        return ampSignalRoller.refresh().getValueAsDouble() < 100;
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
    public Command homeIntake() {
        return this.run(() -> this.setRollerVoltageOut(-5.0))
            .until(this::isArmCurrentTripped)
            .withTimeout(2.0)
            .andThen(() -> this.setArmVoltageOut(0.0))
            .andThen(this::homeArmHere)
            .withName("HomeAcquisition");
    }

    public Command stowAcquisition() {
        //TODO: get stow pos
        return this.run(() -> this.setArmPosition(0));
    }

    public Command intakeAcquisition() {
        return this.run(() -> {
                this.setRollerVoltageOut(-12.0);
                //TODO: get intake pos
                this.setArmPosition(0.0);
            })
            .until(this::isRollerCurrentTripped)
            .andThen(() -> this.setRollerVoltageOut(0.0))
            .andThen(this.stowAcquisition())
            .withName("IntakeAcquisition");
    }
    public Command transferNote() {
        return this.run(() -> this.setRollerVoltageOut(-6.0))
            .until(this::hasNoteLeft)
            .withName("TransferingNote");
        //run motors backwards untill it gets easier to spin rollers    
    }
}

