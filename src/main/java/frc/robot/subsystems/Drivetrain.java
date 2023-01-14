package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StatedSubsystem;
import frc.robot.ShamLib.swerve.ModuleInfo;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.SwerveModule;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.LimeLightHoldAngleCommand;
import frc.robot.util.hardware.Limelight;
import frc.robot.util.hardware.Pose3dSendable;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;
import static frc.robot.subsystems.Drivetrain.*;
import static frc.robot.subsystems.Drivetrain.SwerveState.*;

public class Drivetrain extends StatedSubsystem<SwerveState> {

    private SwerveDrive drive;

    public Drivetrain(CommandXboxController driverController) {
        super(SwerveState.class);

        this.drive = new SwerveDrive(
                PigeonID,
                driveGains,
                turnGains,
                MAX_LINEAR_SPEED,
                MAX_TURN_SPEED,
                MAX_TURN_ACCEL,
                new PIDGains(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE),
                new PIDGains(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO),
                new PIDGains(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION),
                true,
                "Drivetrain",
                "",
                Constants.CURRENT_LIMIT,
                ModuleInfo.getL1Module(MODULE_1_DRIVE_ID, MODULE_1_TURN_ID, MODULE_1_ENCODER_ID, MODULE_1_OFFSET, moduleOffsets[0]),
                ModuleInfo.getL1Module(MODULE_2_DRIVE_ID, MODULE_2_TURN_ID, MODULE_2_ENCODER_ID, MODULE_2_OFFSET, moduleOffsets[1]),
                ModuleInfo.getL1Module(MODULE_3_DRIVE_ID, MODULE_3_TURN_ID, MODULE_3_ENCODER_ID, MODULE_3_OFFSET, moduleOffsets[2]),
                ModuleInfo.getL1Module(MODULE_4_DRIVE_ID, MODULE_4_TURN_ID, MODULE_4_ENCODER_ID, MODULE_4_OFFSET, moduleOffsets[3])
        );


        getOdoPose = () -> getPose();
        getDrivetrainAngle = () -> getCurrentAngle();

        //State machine stuff
        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            setAllModules(new SwerveModuleState(0, new Rotation2d()));
            drive.resetGyro();
            resetHoldAngle();
        }));

        addCommutativeTransition(Idle, Trajectory, new InstantCommand(), new InstantCommand(() -> setAllModules(STOPPED_STATE)));
        addInstanceBasedState(Trajectory);

        addCommutativeTransition(Idle, Teleop, new InstantCommand(), new InstantCommand(() -> setAllModules(STOPPED_STATE)));

        setContinuousCommand(Teleop, new DriveCommand(this, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -driverController.getRightX(), true));

        addCommutativeTransition(Idle, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));
        addCommutativeTransition(Teleop, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));

        setContinuousCommand(
                TeleopLimeLightTracking,
                new ParallelCommandGroup(
                        new DriveCommand(
                                this,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                //disallow driver turning
                                () -> 0,
                                false
                        ),
                        new LimeLightHoldAngleCommand(
                                this
                        )
                )
        );

        addCommutativeTransition(TeleopLimeLightTracking, Teleop, new InstantCommand(), new InstantCommand());
        addCommutativeTransition(TeleopLimeLightTracking, Trajectory, new InstantCommand(), new InstantCommand());
        addCommutativeTransition(TeleopLimeLightTracking, Idle, new InstantCommand(), new InstantCommand());

        driverController.x().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(1, Rotation2d.fromDegrees(90)))));
        driverController.b().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(2, Rotation2d.fromDegrees(-90)))));
        driverController.y().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(0, Rotation2d.fromDegrees(0)))));
    }

    Pose2d visionPoseEstimation = new Pose2d();

    @Override
    public void update() {
        updateOdometry();


    }

    public void updateOdometry() {
        drive.updateOdometry();

        drive.updateField2dObject();
    }

    @Override
    protected void onEnable() {
    }

    @Override
    protected void onDisable() {
        runInstantaneousTransition(Idle, () -> {
            setAllModules(STOPPED_STATE);
        });
    }

    public void drive(ChassisSpeeds speeds, boolean allowHoldAngleChange) {
        drive.drive(speeds, allowHoldAngleChange);
    }

    /**
     * Sets the target state of each swerve module based on the input array
     * @param states array of swerve module states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        drive.setModuleStates(states);
    }

    public void setAllModules(SwerveModuleState state) {
        drive.setAllModules(state);
    }

    /**
     * Finds the angle of the robot in radians (limited -PI to PI)
     * @return Robot angle
     */
    public Rotation2d getCurrentAngle(){
        return drive.getCurrentAngle();
    }

    public void stopModules() {
        drive.stopModules();
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory, boolean resetPose) {
        return drive.getTrajectoryCommand(trajectory, resetPose);
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return getTrajectoryCommand(trajectory, false);
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public double getGyroHeading() {
        return drive.getGyroHeading();
    }


    public boolean isFieldRelative() {return drive.isFieldRelative();}
    public void setFieldRelative(boolean value) {drive.setFieldRelative(value);}


    /* RESET COMMANDS FOR DIFFERENT ASPECTS */

    public void resetGyro(Rotation2d angle) {
        drive.resetGyro(angle);
    }


    public void resetGyro() {
        resetGyro(new Rotation2d());
        resetHoldAngle();
    }


    public void resetHoldAngle(){
        setHoldAngle(new Rotation2d());
    }

    public void setHoldAngle(Rotation2d angle) {
        drive.setHoldAngle(angle);
    }

    public enum SwerveState {
        Undetermined, Idle, Trajectory, PurePursuit, Teleop, XShape, TeleopLimeLightTracking
    }

    @Override
    public String getName() {
        return "drivetrain";
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        // builder.addDoubleProperty("Hold angle", () -> holdAngle.getDegrees(), null);
        builder.addDoubleProperty("Measured Angle", () -> getCurrentAngle().getDegrees(), null);
        // builder.addBooleanProperty("field relative", this::isFieldRelative, null);
//        builder.addDoubleProperty("auto hold target", () -> Math.toDegrees(thetaHoldControllerAuto.getSetpoint()), null);
//        builder.addDoubleProperty("auto hold error", () -> Math.toDegrees(thetaHoldControllerAuto.getPositionError()), null);
        // builder.addDoubleProperty("current angle", () -> thetaHoldControllerAuto.getSetpoint(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        Map<String, Sendable> sendables = new HashMap<>();

       for(SwerveModule m : drive.getModules()) {
            sendables.put(m.getModuleName(), m);
       }

        sendables.put("field", drive.getField());
        // sendables.put("thetaControllerTele", thetaHoldControllerTele);
        // sendables.put("thetaControllerAuto", thetaHoldControllerAuto);
        // sendables.put("xController", xHoldController);
        // sendables.put("yController", yHoldController);

        sendables.put("pose-from-limelight", Limelight.getInstance().getPose());


        return sendables;
    }

    public Command calculateTurnKF(BooleanSupplier interrupt) {
        return drive.calculateKF(interrupt);
    }
    
}
