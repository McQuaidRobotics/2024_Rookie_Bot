package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Monologue;

public class SwerveTeleopCmd extends Command {
    public final Drive subsystem; // we pass it a swerve drive
    public final CommandXboxController controller;

    public SwerveTeleopCmd(Drive subsystem, CommandXboxController controller) {
        this.subsystem = subsystem; // assign the swerve drive so we can use its methoids
        this.controller = controller;

        addRequirements(subsystem);
    }


    @Override
    public void execute() {
        double x = controller.getLeftX();
        double y = controller.getLeftY() * -1.0;
        double r = controller.getRightX();
        x = Monologue.log("/Robot/Drive/Controller/x", MathUtil.applyDeadband(x, .1));
        y = Monologue.log("/Robot/Drive/Controller/y", MathUtil.applyDeadband(y, .1));
        r = Monologue.log("/Robot/Drive/Controller/r", MathUtil.applyDeadband(r, .1));
        ChassisSpeeds zoomies = ChassisSpeeds.fromFieldRelativeSpeeds(x * 5.0, y* 5.0, r * Math.PI * 4, subsystem.getYaw());
        subsystem.drive(zoomies, true);


    }
    
}
