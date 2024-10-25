package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shooter;
import frc.robot.swerve.Drive;
import frc.robot.HigherOrderCommands;

public class Autos{
    public final Shooter shooter = new Shooter();
    public final Drive drive = new Drive();
    public final Intake intake = new Intake();

    public Command 
    autonomousForwardThenBackwards() {
        return Commands.runOnce(() -> drive.setYaw(new Rotation2d(0.0)))
            .andThen(()->intake.homeIntake())
            .andThen(() -> drive.drive(new ChassisSpeeds(3.0, 0.0, 0.0), false))
            .withTimeout(1.5)
            .andThen(() -> drive.drive(new ChassisSpeeds(-3.0, 0.0, 0.0), false))
            .withTimeout(1.0);
    }

    public Command autonomousShootAndReturn() {
        return Commands.runOnce(() -> intake.homeIntake())
            .andThen(() -> drive.setYaw(new Rotation2d(0.0)))
            .andThen(() -> HigherOrderCommands.transferAndShoot(intake, shooter))
            .andThen(() -> this.autonomousForwardThenBackwards());
    }
    
    public Command shootandIntake() {
        return Commands.runOnce(() -> drive.setYaw(new Rotation2d(0.0)))
            .andThen(() -> intake.homeIntake())
            .andThen(() -> HigherOrderCommands.transferAndShoot(intake, shooter))
            .andThen(() -> drive.drive(new ChassisSpeeds(3.0, 0.0, 0.0), false))
            .withTimeout(2.0)
            .alongWith(intake.intakeAcquisition())
            .andThen(() -> drive.drive(new ChassisSpeeds(-3.0, 0.0, 0.0), false))
            .withTimeout(1.0)
            .andThen(() -> HigherOrderCommands.transferAndShoot(intake, shooter));
    }
}
