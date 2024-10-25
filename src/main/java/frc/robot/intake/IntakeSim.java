package frc.robot.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.util.TunableValues;
import frc.robot.util.TunableValues.TunableDouble;

public class IntakeSim extends Intake {
    private final DCMotorSim armMotorSim;
    private final DCMotorSim rollerMotorSim;
    public static final double BACK_HARD_STOP = 0.0;
    private static final double ARM_RATIO = (5.0 / 1) * (5.0 / 1.0) * (22.0 / 12.0);
    private static final double ROLLER_RATIO = 1.0;
    private final double stowPosition = BACK_HARD_STOP/ARM_RATIO;

    boolean hasNote = false;

    private IntakeVisualizer intakeVisualizer;

    public IntakeSim() {
        super();
        intakeVisualizer = new IntakeVisualizer();

        this.armMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), ARM_RATIO, 0.001);
        this.rollerMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), ROLLER_RATIO, 0.001);;
        tunableArmCurrentTripValue.set(60.0);
       
    }
public void setRollerVoltageOut(double volts) {
        rollerMotorSim.setInputVoltage(volts);
    }

    public void setArmPosition(double position){
        armMotorSim.setState(Units.rotationsToRadians(
            Units.degreesToRotations(position) * ARM_RATIO), 0);
    }

    public void setArmVoltageOut(double volts) {
        armMotorSim.setInputVoltage(volts);
    }

    public boolean isArmCurrentTripped() {
        return Math.abs(armMotorSim.getCurrentDrawAmps()) > tunableArmCurrentTripValue.get();
    }

    public void homeArmHere() {
        armMotorSim.setState(stowPosition, 0);
    }

    public double getArmDegrees() {
        return Units.rotationsToDegrees(armMotorSim.getAngularPositionRotations() / ARM_RATIO);
    }

    public boolean isArmAt(double position, double threshold) {
        return MathUtil.isNear(position, getArmDegrees(), threshold);
    }
    public boolean isArmAt(double position) {
        return isArmAt(position, 5.0);
    }
    
    public boolean isLimitTripped(){
        return false;
        // return revLimitSignal.refresh().getValue()
        //     .equals(ReverseLimitValue.ClosedToGround);
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
        TunableDouble voltage = TunableValues.getDouble("ampVoltage", 6.0);
        TunableDouble position = TunableValues.getDouble("ampPosition", 80.0);
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
        // log("RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        log("RollerMotorVelocity", rollerMotorSim.getAngularVelocityRPM());
        log("RollerMotorAmperage", rollerMotorSim.getCurrentDrawAmps());
        // log("ArmMotorVoltage", armMotor.getMotorVoltage().getValueAsDouble());
        log("ArmMotorVelocity", armMotorSim.getAngularVelocityRPM());
        log("ArmMotorAmperage", armMotorSim.getCurrentDrawAmps());
        log("armMotorPosition", getArmDegrees());
        log("HasNote", hasNote);
        log("isLimitTripped", isLimitTripped());

        armMotorSim.update(0.02);
        rollerMotorSim.update(0.02);

        intakeVisualizer.update(Rotation2d.fromDegrees(getArmDegrees()), rollerMotorSim.getAngularVelocityRPM());
    }
}
