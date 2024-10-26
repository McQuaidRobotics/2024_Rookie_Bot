
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shooter;
import frc.robot.swerve.Drive;
import frc.robot.swerve.SwerveTeleopCmd;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;

public class Robot extends TimedRobot implements Logged {
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Autos autos = new Autos(shooter, drive, intake);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final SendableChooser<Command> autoRoutineChooser = new SendableChooser<>();

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

    autoRoutineChooser.setDefaultOption("Nothing", Commands.print("Doing nothing"));
    autoRoutineChooser.addOption("ForwardThenBackwards", autos.forwardThenBackwards());
    autoRoutineChooser.addOption("ShootAndReturn", autos.shootAndReturn());
    autoRoutineChooser.addOption("ShootAndIntake", autos.shootandIntake());
    autoRoutineChooser.addOption("justShoot", autos.shootNoMove());

    SmartDashboard.putData("Auto Chooser", autoRoutineChooser);

  }

  void cofigureBindings() {
    driverController.a().onTrue(intake.intakeAcquisition());
    driverController.b().onTrue(intake.ampNote());
    driverController.x().onTrue(intake.expellNote());

    driverController.povDown().onTrue(intake.homeIntake());

    driverController.back().or(driverController.start())
        .onTrue(Commands.runOnce(() -> drive.setYaw(new Rotation2d())));

    driverController.rightTrigger(0.25)
      .onTrue(HigherOrderCommands.transferAndShoot(intake, shooter));
    driverController.leftTrigger(0.25)
      .whileTrue(shooter.spinUpRPM(() -> 3000.0));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(autoRoutineChooser.getSelected());
  }

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
