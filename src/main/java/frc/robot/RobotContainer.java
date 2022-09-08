package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RobotManager;
import frc.robot.util.Shambots5907_SMF.SubsystemManager;

public class RobotContainer {

  private final Turret turret = new Turret(); //Instantiated in the constructor because I need Robot.java
  
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);

  private final Drivetrain drivetrain = new Drivetrain(driverController);

  private final RobotManager robotManager = new RobotManager();

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

  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

}
