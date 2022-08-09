package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Intake;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
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

  private final RobotManager robotManager = new RobotManager();
  
  private final Intake intake = new Intake();

  public RobotContainer() {
    SubsystemManager.getInstance().registerSubsystems(intake);
    robotManager.enable();

    SubsystemManager.getInstance().registerSubsystem(climber);
    SubsystemManager.getInstance().registerSubsystem(conveyor);

    configureButtonBindings();
  }


  private void configureButtonBindings() {
    

    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartIntakeLeft)));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartIntakeRight)));
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> conveyor.setShouldEndIntakeSequence(true)));
    new JoystickButton(driverController, 4).whenPressed(new InstantCommand(() -> conveyor.requestTransition(StartShooting)));

  }

  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

}
