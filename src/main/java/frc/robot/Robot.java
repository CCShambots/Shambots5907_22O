package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import frc.robot.util.dashboard.DashboardInterface;
import static frc.robot.util.dashboard.DashboardInterface.Tab.*;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
    m_robotContainer.determineRobotManagerState();

    Constants.pullAllianceFromFMS();
    m_robotContainer.enableLights();
    addPeriodic(m_robotContainer::runControlLoops, 0.05, 0.001);
  }


  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    SubsystemManager.getInstance().disableAllSubsystems();

    CommandScheduler.getInstance().cancelAll();
    DashboardInterface.getInstance().setTab(Auto);

    Constants.botEnabledStatus = Constants.RobotEnabled.Disabled;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    SubsystemManager.getInstance().prepSubsystems().schedule();
    DashboardInterface.getInstance().setTab(Auto);

    Constants.botEnabledStatus = Constants.RobotEnabled.Auto;

    Constants.pullAllianceFromFMS();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    SubsystemManager.getInstance().prepSubsystems().schedule();
    DashboardInterface.getInstance().setTab(Teleop);

    Constants.botEnabledStatus = Constants.RobotEnabled.Teleop;

    Constants.pullAllianceFromFMS();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    DashboardInterface.getInstance().setTab(Test);

    Constants.botEnabledStatus = Constants.RobotEnabled.Test;
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
