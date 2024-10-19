// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeSim;
import frc.robot.swerve.Drive;
import frc.robot.swerve.SwerveTeleopCmd;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;

public class Robot extends TimedRobot implements Logged {
  private final Drive drive = new Drive();
  // private final Intake intake = new Intake();
  private final Intake intake = new IntakeSim();
  // private final Shooter shooter = new Shooter();
  private final CommandXboxController driverController = new CommandXboxController(0);

  @Override
  public void robotInit() {
    drive.setDefaultCommand(
      new SwerveTeleopCmd(drive, driverController)
    );

    cofigureBindings();

    Monologue.setupMonologue(
      this,
      "/Robot",
      new MonologueConfig()
        .withDatalogPrefix("")
    );

    Monologue.publishSendable("/Visualizers/Field", drive.field);
    Monologue.log("/Robot/myValue", 42.0);
    log("myValue", 42.0);
  }

  void cofigureBindings() {
    driverController.x().onTrue(intake.homeIntake());
    driverController.y().onTrue(intake.intakeAcquisition());

    // driverController.rightTrigger(.25)
    //   .onTrue(HigherOrderCommands.transferAndShoot(intake, shooter));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}


  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
