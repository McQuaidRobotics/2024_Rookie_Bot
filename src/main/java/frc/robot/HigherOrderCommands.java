package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shooter;
import frc.robot.util.TunableValues;
import frc.robot.util.TunableValues.TunableDouble;

public class HigherOrderCommands {

    private static final TunableDouble shooterRpm = TunableValues.getDouble("ShooterRpm", 5600.0);


    public static Command transferAndShoot(Intake intake, Shooter shooter) {
        return Commands.parallel(
            shooter.spinUpRpm(shooterRpm::value),
            intake.transferNote()
                .beforeStarting(Commands.waitUntil(shooter::hasSpunUp))
        ).beforeStarting(
            intake.stowAcquisition()
                .until(() -> intake.isArmAt(Intake.BACK_HARD_STOP))
        );
    }
}
