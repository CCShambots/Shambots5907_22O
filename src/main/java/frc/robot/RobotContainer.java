package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.RobotManager;

public class RobotContainer {
  
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  
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
