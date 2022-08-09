package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Intake;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.LEDState;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Conveyor;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

import static frc.robot.subsystems.Conveyor.ConveyorState.*;

public class RobotContainer {
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Climber climber = new Climber();
  private final Conveyor conveyor = new Conveyor();
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

public class RobotContainer {

  private final Turret turret = new Turret(); //Instantiated in the constructor because I need Robot.java
  
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Drivetrain drivetrain = new Drivetrain(driverController);

  private final RobotManager robotManager = new RobotManager();
  
  private final Intake intake = new Intake();

  public RobotContainer() {
    SubsystemManager.getInstance().registerSubsystems(intake);
    robotManager.enable();

    SubsystemManager.getInstance().registerSubsystem(climber);
    SubsystemManager.getInstance().registerSubsystem(conveyor);
  private final Lights lights = new Lights();
  
  private final RobotManager robotManager = new RobotManager(lights);

  public RobotContainer() {

    SubsystemManager.getInstance().registerSubsystem(drivetrain);

    robotManager.enable();

    turret.turnOffLimelight();

    SubsystemManager.getInstance().registerSubsystem(turret);

    configureButtonBindings();

  }


  private void configureButtonBindings() {
    

    //Hood angle
    new POVButton(driverController, 0).whenPressed(new InstantCommand(() -> turret.setHoodPower(0.5))).whenReleased(new InstantCommand(() -> turret.setFlywheelPower(0)));
    new POVButton(driverController, 180).whenPressed(new InstantCommand(() -> turret.setHoodPower(-0.5))).whenReleased(new InstantCommand(() -> turret.setFlywheelPower(0)));

    //Rotary angle
    new POVButton(driverController, 90).whenPressed(new InstantCommand(() -> turret.setRotaryPower(0.5))).whenReleased(new InstantCommand(() -> turret.setRotaryPower(0)));
    new POVButton(driverController, 270).whenPressed(new InstantCommand(() -> turret.setRotaryPower(-0.5))).whenReleased(new InstantCommand(() -> turret.setRotaryPower(0)));

    //Flywheel
    new JoystickButton(driverController, 5).whenPressed(new InstantCommand(() -> turret.setFlywheelPower(0.5))).whenReleased(new InstantCommand(() -> turret.setFlywheelPower(0)));

    //PID Controllers
    new JoystickButton(driverController, 6).whenPressed(new InstantCommand(() -> turret.setFlywheelTargetRPM(5000))).whenReleased(new InstantCommand(() -> turret.setFlywheelTargetRPM(0)));
    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> turret.setHoodTargetAngle(0)));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand(() -> turret.setRotaryTargetAngle(-90)));
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> turret.setRotaryTargetAngle(90)));
    new JoystickButton(driverController, 4).whenPressed(new InstantCommand(() -> turret.setHoodTargetAngle(32)));

  }

    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartIntakeLeft)));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartIntakeRight)));
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> conveyor.setShouldEndIntakeSequence(true)));
    new JoystickButton(driverController, 4).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartShooting)));

    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> lights.requestTransition(lights.getCurrentState().next())));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand( () -> lights.requestTransition(LEDState.Default)));
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand( () -> lights.requestTransition(LEDState.Idle)));
    new JoystickButton(driverController, 4).whenPressed(new InstantCommand( () -> lights.requestTransition(LEDState.Testing)));
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
