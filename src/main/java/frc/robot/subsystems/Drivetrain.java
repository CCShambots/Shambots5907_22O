package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.SMF.StatedSubsystem;
import frc.robot.ShamLib.swerve.*;
import frc.robot.ShamLib.vision.PhotonVisionInstance;
import frc.robot.commands.drivetrain.LimeLightHoldAngleCommand;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.SwerveModule.*;
import static frc.robot.subsystems.Drivetrain.*;
import static frc.robot.subsystems.Drivetrain.SwerveState.*;

public class Drivetrain extends StatedSubsystem<SwerveState> {

    private SwerveDrive drive;

    private Pose3d cameraPose = 
    new Pose3d(
        -0.12, 0, 0.81,
        new Rotation3d(
            0, -(4/180) * Math.PI, 0
        )
    );

    private PhotonVisionInstance photonInstance;

    public Drivetrain(CommandXboxController driverController) {
        super(SwerveState.class);

        this.drive = new SwerveDrive(
                PigeonID,
                driveGains,
                turnGains,
                MAX_LINEAR_SPEED,
                MAX_LINEAR_ACCELERATION,
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

        try {
            photonInstance = new PhotonVisionInstance(cameraPose, "Cam");
        } catch (Exception e){
            e.printStackTrace();
        }


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

        setContinuousCommand(Teleop, new DriveCommand(
            drive, 
            () -> -driverController.getLeftY(), 
            () -> -driverController.getLeftX(), 
            () -> -driverController.getRightX(), 
            MAX_LINEAR_SPEED,
            MAX_LINEAR_ACCELERATION, 
            MAX_ROTATION,
            MAX_ROT_ACCEL,
            Constants.ControllerConversions.DEADBAND, 
            Constants.ControllerConversions.conversionFunction, 
            true, 
            this));

        addCommutativeTransition(Idle, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));
        addCommutativeTransition(Teleop, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));

        setContinuousCommand(
                TeleopLimeLightTracking,
                new ParallelCommandGroup(
                        new frc.robot.commands.drivetrain.DriveCommand(
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

        addCommutativeTransition(Teleop, Trajectory, new InstantCommand(), new InstantCommand());

        driverController.x().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(1, Rotation2d.fromDegrees(90)))));
        driverController.b().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(2, Rotation2d.fromDegrees(-90)))));
        driverController.y().onTrue(new InstantCommand(() -> drive.setAllModules(new SwerveModuleState(0, Rotation2d.fromDegrees(0)))));
    }

    public Command runTrajectoryWithEndTracking(PathPlannerTrajectory traj, boolean resetPose) {
        return drive.runTrajectoryWithEndTracking(traj, resetPose, this).andThen(new InstantCommand(() -> requestTransition(Teleop)));
    }

    @Override
    public void update() {
        updateOdometry();
        
        // if(Limelight.getInstance().hasTarget()) {
        //     Pose3d botPose = Limelight.getInstance().getPose(botToLimelight); 

        //     drive.getField().getObject("vision").setPose(botPose.toPose2d());
        //     // drive.addVisionMeasurement(botPose.toPose2d());
        // }

        if(photonInstance.hasTarget()) {
            Pose3d botPose = photonInstance.getPose3d(new Pose3d(getPose()));
            
            if(botPose != null) {
                drive.addVisionMeasurement(botPose.toPose2d());
                drive.getField().getObject("vision").setPose(botPose.toPose2d());
            }
        }

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

    public TrajectoryBuilder buildTrajectory() {
        return drive.buildTrajectory();
    }

    public TrajectoryBuilder buildTrajectory(PathConstraints constraints) {
        return drive.buildTrajectory(constraints);
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
        builder.addDoubleProperty("Measured Angle", () -> getCurrentAngle().getDegrees(), null);

        builder.addDoubleProperty("thetaControllerAuto error", 
            () -> Math.toDegrees(drive.getThetaHoldControllerAuto().getSetpoint() - getCurrentAngle().getRadians()), null);
            
        builder.addDoubleProperty("thetaControllerTele error", 
        () -> Math.toDegrees(drive.getThetaHoldControllerTele().getSetpoint() - getCurrentAngle().getRadians()), null);

        builder.addDoubleProperty("xController error", () -> Math.abs(drive.getxHoldController().getSetpoint() - getPose().getX()), null);
        builder.addDoubleProperty("yController error", () -> Math.abs(drive.getyHoldController().getSetpoint() - getPose().getY()), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        Map<String, Sendable> sendables = new HashMap<>();

       for(SwerveModule m : drive.getModules()) {
            sendables.put(m.getModuleName(), m);
       }

        sendables.put("field", drive.getField());
        sendables.put("thetaControllerTele", drive.getThetaHoldControllerTele());
        sendables.put("thetaControllerAuto", drive.getThetaHoldControllerAuto());
        sendables.put("xController", drive.getxHoldController());
        sendables.put("yController", drive.getyHoldController());

        return sendables;
    }

    public Command calculateTurnKF(BooleanSupplier interrupt) {
        return drive.calculateKF(interrupt);
    }
    
}
