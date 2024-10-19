package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleSim extends Module {

    private final DCMotorSim velocityMotorSim;
    private final DCMotorSim angleMotorSim;
    private final int moduleId;

    private static final double WHEEL_CIRCUMFERENCE = 0.31918;
    private static final double DRIVE_GEAR_RATIO = 5.35714;
    private static final double ANGLE_GEAR_RATIO = 21.428571428571427;

    public ModuleSim(int moduleId){
        super(moduleId, 0.0);
        this.moduleId = moduleId;
        this.velocityMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), DRIVE_GEAR_RATIO, 0.025);
        this.angleMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), ANGLE_GEAR_RATIO, 0.004);
    }

    private double driveRotationsToMeters(double rotations) {
        return (rotations) * WHEEL_CIRCUMFERENCE;
    }

    @Override
    public SwerveModulePosition getPosition() {
        double veloPos = driveRotationsToMeters(velocityMotorSim.getAngularPositionRotations());
        double rotPos = angleMotorSim.getAngularPositionRotations();
        return new SwerveModulePosition(veloPos, Rotation2d.fromRotations(rotPos));
    }

    @Override
    public SwerveModuleState getState() {
        double veloVel = driveRotationsToMeters(velocityMotorSim.getAngularVelocityRPM() / 60.0);
        double rotPos = angleMotorSim.getAngularPositionRotations();
        return new SwerveModuleState(veloVel, Rotation2d.fromRotations(rotPos));
    }

    @Override
    public void applyState(SwerveModuleState state, boolean isOpenLoop){
        log("Preoptimized", state);
        state = SwerveModuleState.optimize(state, getState().angle);
        log("Postoptimized", state);

        // Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= 0.05 ? lastAngle : state.angle;
        log("Angle", state.angle);

        angleMotorSim.setState(state.angle.getRadians(), 0.0);

        // lastAngle = angle;
        if (isOpenLoop){
            double percentOutput = state.speedMetersPerSecond / 5.0;
            velocityMotorSim.setInputVoltage(percentOutput * RobotController.getBatteryVoltage());
        } else {

        }
    }
    
    @Override
    public void periodic() {
        velocityMotorSim.update(0.02);
        angleMotorSim.update(0.02);
    }

    @Override
    public String getOverrideName() {
        return "ModuleSim[" + moduleId + "]";
    }
}
        