package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shooter;
import frc.robot.util.TunableValues;
import frc.robot.util.TunableValues.TunableDouble;

public class HigherOrderCommands {

    private static TunableDouble shooterRPM = TunableValues.getDouble("shooterRPM", 4000);

    public static Command transferAndShoot(Intake intake, Shooter shooter) {
        return Commands.deadline(
            intake.transferNote().beforeStarting(
                Commands.waitUntil(() -> shooter.hasSpunUp(3200.0))
                .andThen(Commands.waitSeconds(0.2))
            ),
            shooter.spinUpRPM(shooterRPM::value)
        ).beforeStarting(
            intake.stowAcquisition()
        );
    }
}