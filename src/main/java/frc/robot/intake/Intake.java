package frc.robot.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableValues;
import frc.robot.util.TunableValues.TunableDouble;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
    private final TalonFX armMotor;
    private final TalonFX rollerMotor;
    private final StatusSignal<Double> ampSignalArm;
    public static final double BACK_HARD_STOP = 0.0;
    private static final double ARM_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (22.0 / 12.0);
    // private final TalonFX bot_Intake;
    private final double stowPosition = BACK_HARD_STOP/ARM_RATIO;
    private final StatusSignal<ReverseLimitValue> revLimitSignal;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final PositionVoltage posOut = new PositionVoltage(0.0).withUpdateFreqHz(0.0);

    DoubleEntry tunableArmCurrentTripValue = NetworkTableInstance.getDefault()
        .getDoubleTopic("/Tunable/ArmCurrentTripValue")
        .getEntry(20.0);

    boolean hasNote = false;

    public Intake() {
        this.armMotor = new TalonFX(11);
        this.rollerMotor = new TalonFX(12);
        this.ampSignalArm = armMotor.getTorqueCurrent();
        this.armMotor.getConfigurator()
            .apply(armMotorConfiguration());
        this.rollerMotor.getConfigurator()
            .apply(rollerMotorConfiguration());
        tunableArmCurrentTripValue.set(60.0);
        this.revLimitSignal = rollerMotor.getReverseLimit();
        this.revLimitSignal.setUpdateFrequency(300);
    }

    public void setRollerVoltageOut(double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

    public void setArmPosition(double position){
        this.armMotor.setControl(posOut.withPosition(Units.degreesToRotations(position) * ARM_RATIO));
    }

    public void setArmVoltageOut(double volts) {
        this.armMotor.setControl(voltageOut.withOutput(volts));
    }

    public boolean isArmCurrentTripped() {
        return Math.abs(ampSignalArm.refresh().getValueAsDouble()) > tunableArmCurrentTripValue.get();
    }

    public void homeArmHere() {
        armMotor.setPosition(stowPosition);
    }

    public double getArmDegrees() {
        return Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble() / ARM_RATIO);
    }

    public boolean isArmAt(double position, double threshold) {
        return MathUtil.isNear(position, getArmDegrees(), threshold);
    }
    public boolean isArmAt(double position) {
        return isArmAt(position, 5.0);
    }
    
    public boolean isLimitTripped(){
        return revLimitSignal.refresh().getValue()
            .equals(ReverseLimitValue.ClosedToGround);
    }

    private TalonFXConfiguration armMotorConfiguration() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = 0.3;
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
            .andThen(Commands.waitSeconds(1.0))
            .andThen(this::homeArmHere)
            .withName("HomeAcquisition");
    }

    public Command stowAcquisition() {
        return this.run(() -> this.setArmPosition(stowPosition))
            .until(() -> this.isArmAt(stowPosition))
            .withName("StowAcquisition");
    }

    public Command intakeAcquisition() {
        Debouncer intakeSensorDebouncer = new Debouncer(0.1);
        BooleanSupplier shouldRetract = () -> {
            boolean ret = intakeSensorDebouncer.calculate(
                isLimitTripped() && getArmDegrees() > 150.0);
            log("shouldRetract", ret);
            return ret;
        };
        return this.run(() -> {
                this.setRollerVoltageOut(-3.0);
                this.setArmPosition(180.0);
            })
            .until(shouldRetract)
            .andThen(() -> this.setRollerVoltageOut(0.0))
            .andThen(
                new ScheduleCommand(this.stowAcquisition()),
                Commands.runOnce(() -> hasNote = true)
            )
            .withName("IntakeAcquisition");
    }

    public Command transferNote() {
        return this.run(() -> this.setRollerVoltageOut(12.5))
            .withTimeout(.8)
            .finallyDo(() -> this.setRollerVoltageOut(0.0)) 
            .withName("TransferingNote"); 
    }

    public Command moveToIntake() {
        return this.run(() -> this.setArmPosition(180.0))
            .until(()-> this.isArmAt(180)); 
    }
    public Command moveToStow() {
        return this.run(() -> this.setArmPosition(0.0))
            .until(()-> this.isArmAt(0.0));
    }
    public Command expellNote() {
        return Commands.deadline(
            Commands.run(() -> this.setRollerVoltageOut(3))
                .withTimeout(1.0)
                .beforeStarting(Commands.waitSeconds(1.0)),
            this.run(() -> setArmPosition(150))
        ).andThen(new ScheduleCommand(moveToStow()));
    }
    public Command ampNote() {
        TunableDouble voltage = TunableValues.getDouble("ampVoltage", 3.0);
        TunableDouble position = TunableValues.getDouble("ampPosition", 84.0);
        return Commands.deadline(
            Commands.run(()-> this.setRollerVoltageOut(voltage.value()))
                .withTimeout(1.0)
                .beforeStarting(Commands.waitSeconds(1.0)),
            this.run(() -> setArmPosition(position.value()))
        ).andThen(new ScheduleCommand(moveToStow()));
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
        log("ArmMotorAmperage", armMotor.getStatorCurrent().getValueAsDouble());
        log("armMotorVoltage", armMotor.getMotorVoltage().getValueAsDouble());
        log("armMotorPosition", getArmDegrees());
        log("HasNote", hasNote);
        log("isLimitTripped", isLimitTripped());
    }
}

