package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopCmd extends Command {
    public final Drive subsystem; // we pass it a swerve drive
    public final CommandXboxController controller;

    public TeleopCmd(Drive subsystem, CommandXboxController controller) {
        this.subsystem = subsystem; // assign the swerve drive so we can use its methoids
        this.controller = controller;

        addRequirements(subsystem);
    }


    @Override
    public void execute() {
        // TODO Auto-generated method stub

        double x = controller.getLeftX();
        double y = controller.getLeftY() * -1.0;
        double r = controller.getRightX() * -1.0;
        x = MathUtil.applyDeadband(x, .1);
        y = MathUtil.applyDeadband(y, .1);
        r = MathUtil.applyDeadband(r, .1);
        ChassisSpeeds zoomies = ChassisSpeeds.fromFieldRelativeSpeeds(x * 5.0, y* 5.0, r * Math.PI * 2, subsystem.getYaw());
        subsystem.drive(zoomies, true);


    }
    
}
