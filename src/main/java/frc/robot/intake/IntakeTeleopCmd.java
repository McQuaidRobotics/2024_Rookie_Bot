package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IntakeTeleopCmd extends Command{
    
    public final CommandXboxController intakeController;
    public final Intake sub;

    public IntakeTeleopCmd(Intake sub, CommandXboxController intakeCommandXboxController) {
        this.sub = sub;
        this.intakeController = intakeCommandXboxController;

        addRequirements(sub);
    }

    @Override
    public void execute() {
        intakeController.x().onTrue(sub.homeIntake());
        intakeController.y().onTrue(sub.intakeAcquisition()); // will pick up stow then transfer note

    }
}