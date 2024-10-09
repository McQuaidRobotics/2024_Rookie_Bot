package frc.robot.shoot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShootTeleopCmd extends Command{
    
    public final CommandXboxController shootingController;
    public final Shoot sub;

    public ShootTeleopCmd(Shoot sub, CommandXboxController shootingController) {
        this.sub = sub;
        this.shootingController = shootingController;

        addRequirements(sub);
    };

    @Override
    public void execute() {

        double rTrig = shootingController.getRightTriggerAxis();
        double lTrig = shootingController.getLeftTriggerAxis();

        rTrig = MathUtil.applyDeadband(rTrig, .1);
        lTrig = MathUtil.applyDeadband(lTrig, .1);

        if (rTrig > lTrig) {
            sub.runShooterRads(50.0);
        } else{
            sub.stopShooting();
        }
    }

    
}
