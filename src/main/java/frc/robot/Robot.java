// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.intake.Intake;
import frc.robot.shoot.Shoot;
import frc.robot.shoot.ShootTeleopCmd;
import frc.robot.swerve.Drive;
import frc.robot.swerve.SwerveTeleopCmd;

public class Robot extends TimedRobot {
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Shoot shoot = new Shoot(intake);
  private final CommandXboxController xboxController = new CommandXboxController(0);

  

  @Override
  public void robotInit() {
    drive.setDefaultCommand(
      new SwerveTeleopCmd(drive, xboxController)
    );

   
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
