package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousLoader;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.RobotManager.RobotState;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain.SwerveState;
import frc.robot.subsystems.Lights;

import java.util.*;

public class RobotContainer {
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Drivetrain drivetrain = new Drivetrain(driverController);
  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Turret turret = new Turret(); //Instantiated in the constructor because I need Robot.java
  private final Climber climber = new Climber();
  private final Lights lights = new Lights();

  private final RobotManager robotManager;

  private final AutonomousLoader autoLoader;
  private final Map<String, PathPlannerTrajectory> paths = new HashMap<>();

  public RobotContainer() {

    //Pull trajectories from Pathplanner
     paths.putAll(loadPaths(List.of(
             "example"
     )));

     //Instantiate the robot manager
    robotManager = new RobotManager(drivetrain, intake, conveyor, turret, climber, lights, paths);

    autoLoader = new AutonomousLoader(robotManager, paths);
    SmartDashboard.putData("Choose Auto Route", autoLoader.getSendableChooser());

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

    //Intake control
    new JoystickButton(operatorController, 1).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.IntakeLeft)));
    new JoystickButton(operatorController, 4).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.IntakeRight)));
    new JoystickButton(operatorController, 2).whenPressed(new InstantCommand(() -> {
      if(robotManager.isInState(RobotState.IntakeRight, RobotState.IntakeLeft)) {
        robotManager.requestTransition(RobotState.Idle);
      }
    }));
    new JoystickButton(operatorController, 3).whenPressed(new InstantCommand(() -> intake.requestTransition(intake.getPumpState())));

    new JoystickButton(driverController, 6).whenPressed(new InstantCommand(() -> drivetrain.incrementModules()));

    new JoystickButton(driverController, 5).whenPressed(new InstantCommand(() -> drivetrain.resetGyro()));


    new JoystickButton(operatorController, 5).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.AttemptShooting)));
    new JoystickButton(operatorController, 6).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.EjectBottom)));

    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.XShape)))
      .whenReleased(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.Teleop)));
  }

  public void runControlLoops() {
    drivetrain.runModuleControlLoops();
  }

  /**
   * Loads the different trajectories from built Pathplanner JSON files
   *
   * @param names A list of the different trajectories to load
   * @return A map of the loaded trajectories
   */
  public Map<String, PathPlannerTrajectory> loadPaths(List<String> names, boolean reversed) {
    Map<String, PathPlannerTrajectory> trajectories = new HashMap<>();

    for (String n : names) {
      trajectories.put(n, PathPlanner.loadPath(n, Constants.SwerveDrivetrain.MAX_LINEAR_SPEED,
              Constants.SwerveDrivetrain.MAX_LINEAR_ACCELERATION, reversed));
    }

    return trajectories;
  }

  public Map<String, PathPlannerTrajectory> loadPaths(List<String> names) {
    return loadPaths(names, false);
  }

  
  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

  public void rescheduleRobotManagerState() { 
    robotManager.rescheduleContinuousCommand();
  }


  public void enableLights() {
    SmartDashboard.putData(lights.getName(), lights);
    lights.enable();
    lights.requestTransition(lights.getEntryState());
  }

}
