package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.led.Lights;
import monologue.Logged;

public class Drive extends SubsystemBase implements Logged {
    public final Pigeon2 gyro;
    public final StatusSignal<Double> gyroDegrees;

    private static final double TRACK_WIDTH = 0.551;
    public final Module[] modules;
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH/2.0, -TRACK_WIDTH/2.0),
        new Translation2d(-TRACK_WIDTH/2.0, TRACK_WIDTH/2.0),
        new Translation2d(-TRACK_WIDTH/2.0, -TRACK_WIDTH/2.0),
        new Translation2d(TRACK_WIDTH/2.0, TRACK_WIDTH/2.0)
    );
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/Robot/Drive/Pose", Pose2d.struct)
        .publish();

    public Drive(){
        this.gyro = new Pigeon2(33, "DriveBus");
        this.gyroDegrees = gyro.getYaw();
        this.modules= new Module[] {
            new Module(0, -0.1015),
            new Module(1, 0.4253),
            new Module(2, -0.4182),
            new Module(3, -0.1086)
        };

    

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
        //Lights.lights(speed);
        setModuleState(KINEMATICS.toSwerveModuleStates(speed), isOpenLoop);
        var p = swerveDrivePoseEstimator.update(getYaw(), getModulePositions());
        posePublisher.set(p);

    }
    
    private void setModuleState(SwerveModuleState[] states, boolean isOpenLoop){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5.0);
        
        for (int i = 0; i < states.length; i ++) {
            modules[i].applyState(states[i], isOpenLoop);
        }
    }
    
    public void setYaw(Rotation2d rot){
        gyro.setYaw(rot.getDegrees());
    }
    public Rotation2d getYaw(){
        return Rotation2d.fromDegrees(gyroDegrees.getValue()); // getting the angle of the robot
    }
    
}
