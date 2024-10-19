package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {

    private static final double MAX_LENGTH = 5.0;

    Mechanism2d mechanism;
    MechanismLigament2d armLig;
    MechanismLigament2d intakeLig;
    MechanismLigament2d rollerLig;
    NetworkTable table;
    
    public IntakeVisualizer() {
        mechanism = new Mechanism2d(10.0, 10.0);
        armLig = new MechanismLigament2d(
                "Intake Arm Visualizer",
                2.0,
                0.0,
                
                2.0,
                new Color8Bit(0, 0, 255));
        
        intakeLig = new MechanismLigament2d(
                "Intake Visualizer 1",
                2.0,
                315.0,
                3.0,
                new Color8Bit(0, 0, 255));

        armLig.append(intakeLig);

        rollerLig = new MechanismLigament2d(
                "Intake Roller Visualizer",
                0.5,
                0.0,
                2.0,
                new Color8Bit(0, 255, 0));

        intakeLig.append(rollerLig);

        mechanism
            .getRoot("pivot", MAX_LENGTH * 1.1, MAX_LENGTH * 1.1)
            .append(armLig);

        table = NetworkTableInstance.getDefault().getTable("Visualizers");

        mechanism.initSendable(getBuilder("Intake"));
    }

    public void update(Rotation2d armRotation, double rollerSpeedRPM) {
        armLig.setAngle(armRotation.getDegrees());
        
        rollerLig.setAngle(rollerLig.getAngle() + Units.rotationsToDegrees(rollerSpeedRPM) / 60.0 * 0.02);
    }

    private SendableBuilderImpl getBuilder(String subtable) {
        var builder = new SendableBuilderImpl();
        builder.setTable(table.getSubTable(subtable));
        return builder;
    }
}