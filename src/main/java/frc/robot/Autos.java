package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shooter;
import frc.robot.swerve.Drive;

public class Autos{
    public final Shooter shooter;
    public final Drive drive;
    public final Intake intake;

    public Autos(Shooter shooter, Drive drive, Intake intake) {
        this.shooter = shooter;
        this.drive = drive;
        this.intake = intake;
    }

    public Command forwardThenBackwards() {
        return Commands.sequence(
            drive.runOnce(() -> drive.setYaw(new Rotation2d(0.0))),
            intake.homeIntake(),
            drive.move(new ChassisSpeeds(3.0, 0.0, 0.0), 1.5),
            drive.move(new ChassisSpeeds(-3.0, 0.0, 0.0), 1.0)
        );
    }

    public Command shootAndReturn() {
        return Commands.sequence(
            intake.homeIntake(),
            HigherOrderCommands.transferAndShoot(intake, shooter),
            forwardThenBackwards()
        );
    }

    public Command shootNoMove() {
        return Commands.sequence(
            drive.runOnce(() -> drive.setYaw(new Rotation2d(0.0))),
            intake.homeIntake(),
            HigherOrderCommands.transferAndShoot(intake, shooter)
        );
    }

    public Command shootandIntake() {
        return Commands.sequence(
            drive.runOnce(() -> drive.setYaw(new Rotation2d(0.0))),
            intake.homeIntake(),
            HigherOrderCommands.transferAndShoot(intake, shooter),
            Commands.parallel(
                drive.move(new ChassisSpeeds(3.0, 0.0, 0.0), 2.0),
                intake.intakeAcquisitionNoStow()
            ),
            Commands.parallel(
                drive.move(new ChassisSpeeds(-3.0, 0, 0), 2.0),
                intake.stowAcquisition()
            )
        );
    }
}
