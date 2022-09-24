package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
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
  // private final Joystick operatorController = new Joystick(1);

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

    new JoystickButton(driverController, 1).whenPressed(new InstantCommand(() -> turret.turnOnLimelight()));
    new JoystickButton(driverController, 2).whenPressed(new InstantCommand(() -> drivetrain.resetGyro()));
    new JoystickButton(driverController, 3).whenPressed(new InstantCommand(() -> drivetrain.resetOdometryPose(new Pose2d())));


  }

  public void runControlLoops() {
    drivetrain.runModuleControlLoops();
  }

  public void determineRobotManagerState() {
    robotManager.enable();
    robotManager.requestTransition(robotManager.getEntryState());
  }

}
