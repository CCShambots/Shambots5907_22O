package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;


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

    setupSubsystems();

    configureButtonBindings();

  }

  private void setupSubsystems() {
    SubsystemManager.getInstance().registerSubsystems(drivetrain, intake, conveyor, turret, climber);

    //The lights subsystem is always enabled, because the lights are always giving information
    SubsystemManager.getInstance().sendSubsystemToNT(lights);

    //The robot manager isn't a normal subsystem, but should still be sent over NT
    SubsystemManager.getInstance().sendSubsystemToNT(robotManager);

    //Both of these are constantly enabled
    robotManager.enable();
    lights.enable();

    //Make them transition to the idle state immediately
    robotManager.requestTransition(robotManager.getEntryState()); //This will only finish determining its state once all the other subsystems finish
    lights.requestTransition(lights.getEntryState());

    //This is just for everyone's comfort once the robot code turns on.
    turret.turnOffLimelight();
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
