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
        // on pressing the right trigger it will run the shooter untill the note leaves the shooter

        shootingController.rightTrigger(.3).onTrue(sub.shootNote()); // IDK IF THIS WILL WORK IDK ABOUT THE ONTRUE
    }       
}

    

