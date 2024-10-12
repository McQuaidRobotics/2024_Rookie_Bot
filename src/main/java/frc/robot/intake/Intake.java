package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
    private final TalonFX armMotor;
    private final TalonFX rollerMotor;
    private final StatusSignal<Double> ampSignalArm;
    public static final double BACK_HARD_STOP = 0.0;
    private final double ARM_RATIO = 0.2;
    // private final TalonFX bot_Intake;
    private final double stowPosition = BACK_HARD_STOP/ARM_RATIO;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;

    DoubleEntry tunableArmCurrentTripValue = NetworkTableInstance.getDefault()
        .getDoubleTopic("/Tunable/ArmCurrentTripValue")
        .getEntry(60.0);

    boolean hasNote = false;

    public Intake() {
        this.armMotor = new TalonFX(11);
        this.rollerMotor = new TalonFX(12);
        this.ampSignalArm = armMotor.getTorqueCurrent();
        this.armMotor.getConfigurator()
            .apply(pivotMotorConfiguration());
        this.rollerMotor.getConfigurator()
            .apply(rollerMotorConfiguration());
        tunableArmCurrentTripValue.set(60.0);
        this.revLimitSignal = rollerMotor.getReverseLimit();
        this.revLimitSignal.setUpdateFrequency(300);
    }

    public void setRollerVoltageOut(double volts) {
        rollerMotor.setControl(new VoltageOut(volts));
    }

    public void setArmPosition(double position){
        this.armMotor.setControl(new PositionVoltage(Units.degreesToRotations(position)));
    }

    public void setArmVoltageOut(double volts) {
        this.armMotor.setControl(new VoltageOut(volts));
    }

    public boolean isArmCurrentTripped() {
        return ampSignalArm.refresh().getValueAsDouble() > tunableArmCurrentTripValue.get();
    }

    public void homeArmHere() {
        armMotor.setPosition(stowPosition);
    }
    public boolean isArmAt(double position, double threshold) {
        double motorPosition = position/ARM_RATIO;
        return MathUtil.isNear(motorPosition, Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()), threshold);
    }
    public boolean isArmAt(double position) {
        return isArmAt(position, 2.0);
    }
    
    public boolean isLimitTripped(){
        return revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
    }
    private TalonFXConfiguration pivotMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = 0.1;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;


        return cfg;
    }

    private TalonFXConfiguration rollerMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.HardwareLimitSwitch.ForwardLimitEnable = false;
        cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

        return cfg;
    }
    

    public Command homeIntake() {
        return this.run(() -> this.setArmVoltageOut(-3.0))
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
            .until(this::isLimitTripped)
            .andThen(() -> this.setRollerVoltageOut(0.0))
            .andThen(
                this.stowAcquisition(),
                Commands.runOnce(() -> hasNote = true)
            )
            .withName("IntakeAcquisition");
    }

    public Command transferNote() {
        return this.run(() -> this.setRollerVoltageOut(6.0))
            .withTimeout(0.5)
            .withName("TransferingNote"); 
    }

    @Override
    public String getOverrideName() {
        return "Intake";
    }

    @Override
    public void periodic() {
        log("RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        log("RollerMotorVelocity", rollerMotor.getVelocity().getValueAsDouble());
        log("RollerMotorAmperage", rollerMotor.getStatorCurrent().getValueAsDouble());
        log("ArmMotorVoltage", armMotor.getMotorVoltage().getValueAsDouble());
        log("ArmMotorVelocity", armMotor.getVelocity().getValueAsDouble());
        log("ArmMotorVoltage", armMotor.getStatorCurrent().getValueAsDouble());
        log("HasNote", hasNote);
    }
}

