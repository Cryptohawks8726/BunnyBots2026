// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.Constants.AutoConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final SendableChooser<String> autoChooser;
  private final SendableChooser<Command> cmdAutoChooser;
  private final SendableChooser<Boolean> booleanAutoChooser;

  public Robot() {
    autoChooser = new SendableChooser<>();
    cmdAutoChooser = new SendableChooser<>();
    booleanAutoChooser = new SendableChooser<>();


    // TODO: Fix this
    // cmdAutoChooser.addOption("crossKitchenLine", new InstantCommand(() -> {
    // }))
    //   .andThen(new WaitUntilCommand(() -> State.getCurrentCommandName().equals("Manual Intake")))
    //   .andThen(m_robotContainer.swerve.driveToPose8726(AutoConstants.crossKitchenLine));
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    
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
  public void autonomousInit() {

    
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

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

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
