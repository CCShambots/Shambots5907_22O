package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;

import static frc.robot.subsystems.Intake.IntakeState.*;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.*;

public class Intake extends StatedSubsystem<Intake.IntakeState> {

    private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, LEFT_SOLENOID_FORWARD, LEFT_SOLENOID_REVERSE);
    private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, RIGHT_SOLENOID_FORWARD, RIGHT_SOLENOID_REVERSE);

    private final WPI_TalonFX leftMotor = new WPI_TalonFX(LEFT_MOTOR_ID);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(RIGHT_MOTOR_ID);

    public Intake() {
        super(IntakeState.class);

        configureMotor(leftMotor);
        configureMotor(rightMotor);

        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            stopLeftMotor();
            stopRightMotor();
            raiseLeftSide();
            raiseRightSide();
        }));

        addCommutativeTransition(Idle, LeftSideRunning, 
            new InstantCommand(() -> {runLeftMotor(); lowerLeftSide();}), 
            new InstantCommand(() -> {stopLeftMotor(); raiseLeftSide();}));
        addCommutativeTransition(Idle, RightSideRunning, 
            new InstantCommand(() -> {runRightMotor(); lowerRightSide();}), 
            new InstantCommand(() -> {stopRightMotor(); raiseRightSide();}));
        addCommutativeTransition(LeftSideRunning, RightSideRunning, 
            new InstantCommand(() -> {runRightMotor(); lowerRightSide(); stopLeftMotor(); raiseLeftSide();}), 
            new InstantCommand(() -> {runLeftMotor(); lowerLeftSide(); stopRightMotor(); raiseRightSide();}));

        addCommutativeTransition(Idle, Ejecting, 
            new InstantCommand(() -> {lowerLeftSide(); lowerRightSide(); exhaustLeftMotor(); exhaustRightMotor();}),
            new InstantCommand(() -> {raiseLeftSide(); raiseRightSide(); stopLeftMotor(); stopRightMotor();}));
        
        addTransition(LeftSideRunning, Ejecting, new InstantCommand(() -> {exhaustLeftMotor(); lowerRightSide(); exhaustRightMotor();}));
        addTransition(RightSideRunning, Ejecting, new InstantCommand(() -> {exhaustRightMotor(); lowerLeftSide(); exhaustLeftMotor();}));
    }

    private void runLeftMotor() {leftMotor.set(INTAKE_POWER);}
    private void runRightMotor() {rightMotor.set(INTAKE_POWER);}

    private void exhaustLeftMotor() {leftMotor.set(-INTAKE_POWER);}
    private void exhaustRightMotor() {rightMotor.set(-INTAKE_POWER);}

    private void stopLeftMotor() {leftMotor.set(0);}
    private void stopRightMotor() {rightMotor.set(0);}

    private void lowerLeftSide() {leftSolenoid.set(DoubleSolenoid.Value.kReverse);}
    private void lowerRightSide() {rightSolenoid.set(DoubleSolenoid.Value.kReverse);}

    private void raiseLeftSide() {leftSolenoid.set(DoubleSolenoid.Value.kForward);}
    private void raiseRightSide() {rightSolenoid.set(DoubleSolenoid.Value.kForward);}

    public enum IntakeState {
        Undetermined, Idle, LeftSideRunning, RightSideRunning, Ejecting
    }

    @Override
    public void update() {

    }

    @Override
    public String getName() {
        return "Intake";
    }

    @Override
    public void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("left-power", leftMotor::get, null);
        builder.addDoubleProperty("right-power", rightMotor::get, null);
        builder.addStringProperty("left-solenoid", () -> leftSolenoid.get().name(), null);
        builder.addStringProperty("right-solenoid", () -> rightSolenoid.get().name(), null);
    }
}