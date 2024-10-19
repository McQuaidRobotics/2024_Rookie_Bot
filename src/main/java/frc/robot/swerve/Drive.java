package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import monologue.Logged;

public class Drive extends SubsystemBase implements Logged {
    public final Pigeon2 gyro;
    public final StatusSignal<Double> gyroDegrees;
    private double gyroSimDegrees;
    
    public final Field2d field;
    private final SwerveVisualizer visualizer;

    private static final double TRACK_WIDTH = 0.551;
    public final Module[] modules;
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH/2.0, -TRACK_WIDTH/2.0),
        new Translation2d(-TRACK_WIDTH/2.0, -TRACK_WIDTH/2.0),
        new Translation2d(-TRACK_WIDTH/2.0, TRACK_WIDTH/2.0),
        new Translation2d(TRACK_WIDTH/2.0, TRACK_WIDTH/2.0)
    );

    public Drive(){
        this.gyro = new Pigeon2(33, "DriveBus");
        this.gyroDegrees = gyro.getYaw();
        this.field = new Field2d();
        if (Robot.isReal()) {
            this.modules = new Module[] {
                new Module(0, -0.216064453125),
                new Module(1, -0.39453125),
                new Module(2, -0.41796875),
                new Module(3, 0.106201171875)
            };
        }
        else {
            this.modules = new Module[] {
                new ModuleSim(0),
                new ModuleSim(1),
                new ModuleSim(2),
                new ModuleSim(3)
            };
        }
        visualizer = new SwerveVisualizer(modules);

    }

    SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        KINEMATICS,
        new Rotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(1.0, 1.0, 1.0)
    );

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    public void drive(ChassisSpeeds speed, boolean isOpenLoop){
        log("speed", speed);
        speed = ChassisSpeeds.discretize(speed, 0.2);
        setModuleState(KINEMATICS.toSwerveModuleStates(speed), isOpenLoop);
        gyroSimDegrees += Units.radiansToDegrees(speed.omegaRadiansPerSecond * 0.02);
        field.getRobotObject().setPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    private void setModuleState(SwerveModuleState[] states, boolean isOpenLoop){
        log("states", states);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5.0);
        log("desaturated", states);
        for (int i = 0; i < states.length; i ++) {
            modules[i].applyState(states[i], isOpenLoop);
        }
    }

    public void setYaw(Rotation2d rot){
        gyro.setYaw(rot.getDegrees());
    }

    public Rotation2d getYaw(){
        if (Robot.isReal()) {
            return Rotation2d.fromDegrees(gyroDegrees.getValue());
        }
        else {
            return Rotation2d.fromDegrees(gyroSimDegrees);
        }
    }

    @Override
    public void periodic() {
        log("Yaw", getYaw());
        log("Pose", swerveDrivePoseEstimator.update(getYaw(), getModulePositions()));
        log("Positions", getModulePositions());

        for (var module : modules) {
            module.periodic();
        }

        visualizer.update();
    }
}
