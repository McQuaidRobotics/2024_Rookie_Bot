package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;

public class Commands {
    public static Command homeIntake(Intake intake) {
        return intake.run(() -> intake.setRollerVoltageOut(-5.0))
            .until(intake::isArmCurrentTripped)
            .withTimeout(2.0)
            .andThen(() -> intake.setArmVoltageOut(0.0))
            .andThen(intake::homeArmHere)
            .withName("HomeAcquisition");
    }

    public static Command stowAcquisition(Intake intake) {
        //TODO: get stow pos
        return intake.run(() -> intake.setArmPosition(0));
    }

    public static Command intakeAcquisition(Intake intake) {
        return intake.run(() -> {
                intake.setRollerVoltageOut(-12.0);
                //TODO: get intake pos
                intake.setArmPosition(0.0);
            })
            .until(intake::isRollerCurrentTripped)
            .andThen(() -> intake.setRollerVoltageOut(0.0))
            .andThen(stowAcquisition(intake))
            .withName("IntakeAcquisition");
    }
}
