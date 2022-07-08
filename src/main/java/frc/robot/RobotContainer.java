package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.RobotManager;

public class RobotContainer {

  private final RobotManager robotManager = new RobotManager();

  public RobotContainer() {

    robotManager.enable();

    configureButtonBindings();
  }


  private void configureButtonBindings() {}

  
  public Command determineRobotManagerState() {
    return robotManager.goToState(robotManager.getEntryState());
  }


}
