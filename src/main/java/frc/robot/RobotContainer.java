package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Lights.LEDState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

import static frc.robot.subsystems.Conveyor.ConveyorState.*;

public class RobotContainer {
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Drivetrain drivetrain = new Drivetrain(driverController);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(); //Instantiated in the constructor because I need Robot.java
  private final Climber climber = new Climber();
  private final Lights lights = new Lights();

  private final RobotManager robotManager = new RobotManager(drivetrain, intake, conveyor, turret, climber, lights);


  public RobotContainer() {
    robotManager.enable();

    SubsystemManager.getInstance().registerSubsystems(drivetrain, intake, conveyor, turret, climber);
    SubsystemManager.getInstance().sendSubsystemToNT(lights);

    turret.turnOffLimelight();

    SubsystemManager.getInstance().registerSubsystem(turret);

    configureButtonBindings();

  }


  private void configureButtonBindings() {

  }

  
  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

  public void enableLights() {
    SmartDashboard.putData(lights.getName(), lights);
    lights.enable();
    lights.requestTransition(lights.getEntryState());
  }

}
