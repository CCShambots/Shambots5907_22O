package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

public class RobotContainer {
  
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Climber climber = new Climber();

  private final RobotManager robotManager = new RobotManager();

  public RobotContainer() {

    robotManager.enable();

    SubsystemManager.getInstance().registerSubsystem(climber);

    configureButtonBindings();
  }


  private void configureButtonBindings() {}

  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

}
