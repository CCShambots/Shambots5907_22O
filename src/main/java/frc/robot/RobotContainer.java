package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.ShamLib.SMF.SubsystemManager;
import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.OneBall;
import frc.robot.commands.auto.TwoBall;
import frc.robot.subsystems.*;
import frc.robot.ShamLib.AutonomousLoader;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.RobotManager.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain.SwerveState;
import frc.robot.subsystems.Lights;

import java.util.*;

import static edu.wpi.first.math.geometry.Rotation2d.fromDegrees;

public class RobotContainer {
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);
  private final CommandXboxController driveXboxController = new CommandXboxController(2);

  private final Drivetrain drivetrain = new Drivetrain(driveXboxController);
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
            "five-ball-1", "five-ball-2", "five-ball-3", "five-ball-4", "one-ball", "two-ball"
     )));

     //Instantiate the robot manager
    robotManager = new RobotManager(drivetrain, intake, conveyor, turret, climber, lights, paths);

    autoLoader = new AutonomousLoader(
            Map.of(
                    AutoRoute.FiveBall, new FiveBall(robotManager),
                    AutoRoute.OneBall, new OneBall(robotManager),
                    AutoRoute.TwoBall, new TwoBall(robotManager)
            )
    );
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
    
    //Drivetrain control
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.XShape)))
      .whenReleased(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.Teleop)));

    new JoystickButton(driverController, 4)
      .whenPressed(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.TeleopLimeLightTracking)))
      .whenReleased(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.Teleop)));

    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> turret.setHoodTargetAngle(5)));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand(() -> turret.setHoodTargetAngle(25)));

    new JoystickButton(driverController, 5).whenPressed(new InstantCommand(() -> drivetrain.resetGyro()));

    //Intake control
    new JoystickButton(operatorController, 1).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.IntakeLeft)));
    new JoystickButton(operatorController, 4).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.IntakeRight)));
    new JoystickButton(operatorController, 2).whenPressed(new InstantCommand(() -> {
      if(robotManager.isInState(RobotState.IntakeRight, RobotState.IntakeLeft)) {
        robotManager.requestTransition(RobotState.Idle);
      }
    }));
    new JoystickButton(operatorController, 3).whenPressed(new InstantCommand(() -> intake.requestTransition(intake.getPumpState())));

    //Shoting controls
    new JoystickButton(operatorController, 5).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.AttemptShooting)))
      .whenReleased(new InstantCommand(() -> {
        if(robotManager.isInState(RobotState.AttemptShooting)) robotManager.requestTransition(RobotState.Idle);
      }));
    

      //Ejection control
    new JoystickButton(operatorController, 6).whenPressed(new InstantCommand(() -> robotManager.requestTransition(RobotState.EjectBottom)));

    //Climber control
    new JoystickButton(operatorController, 7).whenPressed(new InstantCommand(() -> robotManager.advanceClimbState()));

    // driveXboxController.a().toggleOnTrue(drivetrain.calculateTurnKF(() -> driveXboxController.getLeftX() > 0));
  
    driveXboxController.leftBumper().onTrue(new InstantCommand(() -> drivetrain.resetGyro()));

    driveXboxController.a().onTrue(
      new InstantCommand(() -> {
        drivetrain.runTrajectoryWithEndTracking(
                drivetrain.buildTrajectory(new PathConstraints(1.5, 1))
                        .splineToHeading(new Pose2d(14.5, 2.75, new Rotation2d(0)), fromDegrees(0))
                        .build()
                , false
        ).schedule();
      })
    );
    driveXboxController.b().onTrue(drivetrain.goToStateCommand(SwerveState.Teleop));

    driveXboxController.x().onTrue(new InstantCommand(() -> drivetrain.requestTransition(SwerveState.XShape)));
  }


  public Command getAutoCommand() {
    return autoLoader.getCurrentSelection();
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

  public void climberTest() {
    climber.climbTest();
  }

  public void clearClimbTest() {

  }


  public void enableLights() {
    SmartDashboard.putData(lights.getName(), lights);
    lights.enable();
    lights.requestTransition(lights.getEntryState());
  }

  public enum AutoRoute {
    FiveBall, OneBall, TwoBall
  }

}
