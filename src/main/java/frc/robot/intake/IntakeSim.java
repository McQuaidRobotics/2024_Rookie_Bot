package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        // rollerMotor.setControl(new VoltageOut(volts));
    }

    public void setArmPosition(double position){
        this.armMotorSim.setState(Units.degreesToRadians(position), 0);
        // this.armMotor.setControl(new PositionVoltage(Units.degreesToRotations(position) * ARM_RATIO));
    }

    public void setArmVoltageOut(double volts) {
        this.armMotorSim.setInputVoltage(volts);
        // this.armMotor.setControl(new VoltageOut(volts));
    }

    public boolean isArmCurrentTripped() {
        return false;
        // return ampSignalArm.refresh().getValueAsDouble() > tunableArmCurrentTripValue.get();
    }

    public void homeArmHere() {
        this.armMotorSim.setState(stowPosition, 0);
        // armMotor.setPosition(stowPosition);
    }
    public double getArmDegrees() {
        return Units.radiansToDegrees(armMotorSim.getAngularPositionRad());
        // return armMotor.getPosition().getValueAsDouble()*9.0;
    }
    public boolean isArmAt(double position, double threshold) {
        double motorPosition = position/ARM_RATIO;
        return MathUtil.isNear(motorPosition, Units.rotationsToDegrees(armMotorSim.getAngularPositionRotations()), threshold);
        // return MathUtil.isNear(motorPosition, Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()), threshold);
    }
    public boolean isArmAt(double position) {
        return isArmAt(position, 2.0);
    }
    
    public boolean isLimitTripped(){
        return false;
        // return revLimitSignal.getValue().equals(ReverseLimitValue.ClosedToGround);
    }

    public Command homeIntake() {
        return this.run(() -> this.setArmVoltageOut(-3.0))
            .until(() -> this.isArmAt(stowPosition))
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
                this.setArmPosition(180.0 + stowPosition);
            })
            .until(() -> this.isArmAt(180.0))
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
        // log("RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        // log("RollerMotorVelocity", rollerMotor.getVelocity().getValueAsDouble());
        // log("RollerMotorAmperage", rollerMotor.getStatorCurrent().getValueAsDouble());
        // log("ArmMotorVoltage", armMotor.getMotorVoltage().getValueAsDouble());
        // log("ArmMotorVelocity", armMotor.getVelocity().getValueAsDouble());
        // log("ArmMotorVoltage", armMotor.getStatorCurrent().getValueAsDouble());
        // log("HasNote", hasNote);
        
        log("RollerMotorVelocity", rollerMotorSim.getAngularVelocityRPM());
        log("RollerMotorAmperage", rollerMotorSim.getCurrentDrawAmps());
        log("ArmMotorVelocity", armMotorSim.getAngularVelocityRPM());
        log("HasNote", hasNote);

        armMotorSim.update(0.02);
        rollerMotorSim.update(0.02);

        intakeVisualizer.update(Rotation2d.fromRadians(armMotorSim.getAngularPositionRad()), rollerMotorSim.getAngularVelocityRPM());
    }
}
