package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.swerve.Constants.kSwerve;
import monologue.Annotations.Log;

public class ModuleSim extends Module {

    private final DCMotorSim driveMotorSim;
    private final DCMotorSim angleMotorSim;

    @Log
    private Rotation2d lastAngle = new Rotation2d();
    @Log
    public double driveVeloMPS = 0.0;
    @Log
    public double targetDriveVeloMPS = 0.0;
    @Log
    public double drivePositionMeters = 0.0;
    @Log
    public double driveVolts = 0.0;
    @Log
    public double driveAmps = 0.0;
    @Log
    public double angleVeloRadPS = 0.0;
    @Log
    public double angleAbsoluteRads = 0.0;
    @Log
    public double targetAngleAbsoluteRads = 0.0;
    @Log
    public double angleVolts = 0.0;
    @Log
    public double angleAmps = 0.0;

    public ModuleSim(int moduleId){
        super(moduleId, 0.0);
        this.driveMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), kSwerve.DRIVE_GEAR_RATIO, 0.025);
        this.angleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), kSwerve.ANGLE_GEAR_RATIO, 0.004);
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? lastAngle
                : desiredState.angle;
        this.targetAngleAbsoluteRads = angle.getRadians();

        angleMotorSim.setState(angle.getRadians(), 0);
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        this.targetDriveVeloMPS = desiredState.speedMetersPerSecond;
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY;
            driveMotorSim.setInputVoltage(percentOutput * RobotController.getBatteryVoltage());
        } else {
            double rps = (desiredState.speedMetersPerSecond / kSwerve.WHEEL_CIRCUMFERENCE) * kSwerve.DRIVE_GEAR_RATIO;
            log("DriveRPS", rps);
            driveMotorSim.setState(this.drivePositionMeters + driveRotationsToMeters(rps * 0.02), Units.rotationsToRadians(rps));
        }
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                this.driveVeloMPS,
                getAngle());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                this.drivePositionMeters,
                getAngle());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(this.angleAbsoluteRads);
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    public void periodic() {
        this.angleAbsoluteRads = angleMotorSim.getAngularPositionRad();
        this.angleVeloRadPS = angleMotorSim.getAngularVelocityRadPerSec();
        // this.angleVolts = angleVoltSignal.getValueAsDouble();
        this.angleAmps = angleMotorSim.getCurrentDrawAmps();

        this.drivePositionMeters = driveRotationsToMeters(driveMotorSim.getAngularPositionRotations());
        this.driveVeloMPS = driveRotationsToMeters(driveMotorSim.getAngularVelocityRPM() / 60.0);
        // this.driveVolts = driveVoltSignal.getValueAsDouble();
        this.driveAmps = driveMotorSim.getCurrentDrawAmps();

        driveMotorSim.update(0.02);
        angleMotorSim.update(0.02);
    }

    public void setVoltageOut(double volts, Rotation2d angle) {
        setAngle(new SwerveModuleState(0.0, angle));
        driveMotorSim.setInputVoltage(volts);
    }
}
        