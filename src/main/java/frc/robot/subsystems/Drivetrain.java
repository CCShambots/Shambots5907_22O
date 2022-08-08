package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.util.Shambots5907_SMF.StatedSubsystem;
import frc.robot.util.SwerveModule;
import frc.robot.util.hardware.Limelight;
import frc.robot.util.math.ComputerVisionUtil;
import frc.robot.util.math.Geometry;

import static frc.robot.Constants.SwerveDrivetrain.*;
import static frc.robot.Constants.Turret.*;
import static frc.robot.subsystems.Drivetrain.*;
import static frc.robot.subsystems.Drivetrain.SwerveState.*;

public class Drivetrain extends StatedSubsystem<SwerveState> {

    private Map<String, SwerveModule> modules;
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(1);
    private double rotationOffsetDegrees;
    private Rotation2d holdAngle;

    private PIDController thetaHoldControllerTele = new PIDController(P_HOLDANGLETELE, I_HOLDANGLETELE, D_HOLDANGLETELE);

    private ProfiledPIDController thetaHoldControllerAuto = new ProfiledPIDController(P_HOLDANGLEAUTO, I_HOLDANGLEAUTO, D_HOLDANGLEAUTO, new TrapezoidProfile.Constraints(MAX_ROTATION, MAX_ROT_ACCEL));
    private PIDController xHoldController = new PIDController(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION);
    private PIDController yHoldController = new PIDController(P_HOLDTRANSLATION, I_HOLDTRANSLATION, D_HOLDTRANSLATION);
    private SwerveDrivePoseEstimator odometry;

    private boolean fieldRelative = true;

    private Field2d field;

    public Drivetrain(Joystick driverController) {
        super(SwerveState.class);

        modules = new HashMap<>();
        modules.put("Module 1", new SwerveModule("Module-1", 11, 1, 1, false));
        modules.put("Module 2", new SwerveModule("Module-2", 12, 2, 2, false));
        modules.put("Module 3", new SwerveModule("Module-3", 13, 3, 3, true));
        modules.put("Module 4", new SwerveModule("Module-4", 14, 4, 4, true));

        gyro.configFactoryDefault();

        rotationOffsetDegrees = getGyroHeading();
        holdAngle = new Rotation2d(rotationOffsetDegrees);
        
        odometry = new SwerveDrivePoseEstimator(getCurrentAngle(), new Pose2d(), kDriveKinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0, 0, 0)
                );

        thetaHoldControllerTele.enableContinuousInput(-Math.PI, Math.PI);
        field = new Field2d();

        getOdoPose = () -> getPose();
        getDrivetrainAngle = () -> getCurrentAngle();

        //State machine stuff
        addDetermination(Undetermined, Idle, new InstantCommand(() -> {
            setAllModules(new SwerveModuleState(0, new Rotation2d()));
            resetModuleControlLoops();
            resetGyro();
            resetHoldAngle();
        }));

        addCommutativeTransition(Idle, Trajectory, new InstantCommand(), new InstantCommand(() -> setAllModules(STOPPED_STATE)));
        addInstanceBasedState(Trajectory);

        addCommutativeTransition(Idle, Teleop, new InstantCommand(), new InstantCommand(() -> setAllModules(STOPPED_STATE)));

        //TODO: Fix axes
        setContinuousCommand(Teleop, new DriveCommand(this, () -> driverController.getRawAxis(1), () -> driverController.getRawAxis(4), () -> driverController.getRawAxis(5)));

        addCommutativeTransition(Idle, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));
        addCommutativeTransition(Teleop, XShape, new InstantCommand(() -> setModuleStates(X_SHAPE_ARRAY)), new InstantCommand(() -> setAllModules(STOPPED_STATE)));


    }

    @Override
    public void update() {
        updateOdometry();

        field.setRobotPose(getPose());

        if(Limelight.getInstance().hasTarget()) {
            Pose2d visionPoseEstimation = ComputerVisionUtil.estimateFieldToRobot(
                    LIMELIGHT_HEIGHT, GOAL_HEIGHT, LIMELIGHT_ANGLE, Limelight.getInstance().getYOffset().getRadians(), Limelight.getInstance().getXOffset(),
                    getCurrentAngle(), Geometry.getCurrentTargetPose(getDrivetrainAngle.get(), getRotaryAngle.get(), getLimelightXOffsetAngle.get()),
                    new Transform2d(new Translation2d(), new Rotation2d())
            );

            //TODO: Timer.getFPGATimestamp() might cause some issues?
            odometry.addVisionMeasurement(visionPoseEstimation, Timer.getFPGATimestamp());
        }
    }

    @Override
    protected void onEnable() {
        resetModuleControlLoops();
    }

    @Override
    protected void onDisable() {
        runInstantaneousTransition(Idle, () -> {
            setAllModules(STOPPED_STATE);
        });
    }

    public void drive(ChassisSpeeds speeds) {
        if(speeds.omegaRadiansPerSecond == 0) {
            speeds.omegaRadiansPerSecond += thetaHoldControllerTele.calculate(getCurrentAngle().getRadians(), holdAngle.getRadians());
            if(Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond)) < 4) speeds.omegaRadiansPerSecond = 0;
        } else holdAngle = getCurrentAngle();

        SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_LINEAR_SPEED);

        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the target state of each swerve module based on the input array
     * @param states array of swerve module states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        modules.get("Module 1").setDesiredState(states[0]);
        modules.get("Module 2").setDesiredState(states[1]);
        modules.get("Module 3").setDesiredState(states[2]);
        modules.get("Module 4").setDesiredState(states[3]);
    }

    public void setAllModules(SwerveModuleState state) {
        for(SwerveModule module : modules.values()) {
            module.setDesiredState(state);
        }
    }

    /**
     * Finds the angle of the robot in radians (limited -PI to PI)
     * @return Robot angle
     */
    public Rotation2d getCurrentAngle(){
        double angle = getGyroHeading() - rotationOffsetDegrees;
        while (angle < -180){ angle += 360; }
        while (angle > 180){ angle -= 360; }
        return Rotation2d.fromDegrees(angle);
    }

    public void runModuleControlLoops() {
        modules.forEach((name, module) -> module.run());
    }

    public void stopModules() {
        modules.forEach((name, module) -> module.stop());
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory, boolean resetPose) {
        //TODO: Remove trajectory drawing if we ever take this to comp
        return new ParallelCommandGroup(
            new InstantCommand(() -> {
                field.getObject("traj").setTrajectory(trajectory);
                if(resetPose) resetOdometryPose(trajectory.getInitialPose());
            }),
            new PPSwerveControllerCommand(
                trajectory, this::getPose, kDriveKinematics, 
                xHoldController, yHoldController, thetaHoldControllerAuto, 
                (states) -> setModuleStates(states), this),
            new PrintCommand("Finished PP Swerve command")
        );
        
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return getTrajectoryCommand(trajectory, false);
    }

    public void updateOdometry() {
        odometry.update(getCurrentAngle(), 
            modules.get("Module 1").getCurrentState(), 
            modules.get("Module 2").getCurrentState(), 
            modules.get("Module 3").getCurrentState(), 
            modules.get("Module 4").getCurrentState()
        );
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public double getGyroHeading() {
        return gyro.getYaw();
    }

    public boolean isFieldRelative() {return fieldRelative;}
    public void setFieldRelative(boolean value) {fieldRelative = value;}


    /* RESET COMMANDS FOR DIFFERENT ASPECTS */

    public void resetGyro(Rotation2d angle) {
        gyro.setYaw(angle.getDegrees());
        rotationOffsetDegrees = 0;
        holdAngle = getCurrentAngle();
    }


    public void resetGyro() {
        resetGyro(new Rotation2d());
    }

    public void resetHoldAngle(){
        holdAngle = getCurrentAngle();
    }

    public void resetOdometryPose(Pose2d newPose) {
        odometry.resetPosition(newPose, getCurrentAngle());
        resetGyro(newPose.getRotation());
    }

    public void resetModuleControlLoops() {
        modules.forEach((name, module) -> module.resetControlLoops());
    }

    public enum SwerveState {
        Undetermined, Idle, Trajectory, PurePursuit, Teleop, XShape
    }

    @Override
    public String getName() {
        return "drivetrain";
    }

    @Override
    protected void additionalSendableData(SendableBuilder builder) {
        builder.addDoubleProperty("Hold angle", () -> holdAngle.getDegrees(), null);
        builder.addDoubleProperty("Measured Angle", () -> getCurrentAngle().getDegrees(), null);
    }

    @Override
    public Map<String, Sendable> additionalSendables() {
        Map<String, Sendable> sendables = new HashMap<>();

        for(Entry<String, SwerveModule> e : modules.entrySet()) {
            sendables.put(e.getKey(), e.getValue());
        }

        sendables.put("field", field);
        sendables.put("thetaControllerTele", thetaHoldControllerTele);
        sendables.put("thetaControllerAuto", thetaHoldControllerAuto);
        sendables.put("xController", xHoldController);
        sendables.put("yController", yHoldController);

        return sendables;
    }
    
}
