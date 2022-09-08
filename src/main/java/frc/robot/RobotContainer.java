package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

public class RobotContainer {
  
  private final Joystick driverController = new Joystick(0);
  // private final Joystick operatorController = new Joystick(1);

  private final Drivetrain drivetrain = new Drivetrain(driverController);

  private final RobotManager robotManager = new RobotManager();

  public RobotContainer() {

    SubsystemManager.getInstance().registerSubsystem(drivetrain);

    robotManager.enable();

    configureButtonBindings();
  }


  private void configureButtonBindings() {

    new JoystickButton(driverController, 1).whenPressed(drivetrain.goToStateCommand(Drivetrain.SwerveState.XShape));
    new JoystickButton(driverController, 2).whenPressed(drivetrain.goToStateCommand(Drivetrain.SwerveState.Teleop));
  }

  public void runControlLoops() {
    drivetrain.runModuleControlLoops();
  }

  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

}
