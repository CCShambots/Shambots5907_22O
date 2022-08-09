package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.util.math.Geometry;
import frc.robot.util.math.InterpLUT;

import java.util.function.Supplier;

import static frc.robot.Constants.GOAL_POS;
import static frc.robot.Constants.Turret.*;
import static frc.robot.commands.turret.ActiveTracking.ActiveTrackingMode.*;
import static frc.robot.subsystems.Turret.RotarySpeed.*;

public class ActiveTracking extends CommandBase {

    private Turret turret;
    private InterpLUT flywheelLUT, hoodLUT;
    private BooleanConsumer indicateLockedInConsumer;
    private Supplier<Pose2d> odoPoseSupplier;

    //Control variables
    private double limelightOffset = 0; //The value of the target off on the limelight (x-axis)
    private double targetAngle = 0; //The shooter's current setpoint
    private ActiveTrackingMode mode;
    private Timer timer;
    private Direction searchDirection;
    private Direction prevSearchDirection;

    public ActiveTracking(Turret turret, InterpLUT flywheelLUT, InterpLUT hoodLUT, BooleanConsumer indicateLockedIn) {
        this.turret = turret;
        this.flywheelLUT = flywheelLUT;
        this.hoodLUT = hoodLUT;
        this.indicateLockedInConsumer = indicateLockedIn;

        this.odoPoseSupplier = Constants.SwerveDrivetrain.getOdoPose;

        this.timer = new Timer();
    }

    /**Turns on the limelight, resets the gyro, and sets the limeLightOffset to whatever is currently on the network table */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        //Reset values so the same command instance can be called again
        mode = Searching;
        limelightOffset = 0;
        targetAngle = 0;

        turret.setRotaryTargetAngle(targetAngle);
    }

    /** */
    @Override
    public void execute() {
        updateLimelight();

        if(mode == Targeting) targetingLoop();
        if(mode == WrapAround) wrapAroundLoop();
        if(mode == Searching) searchingLoop();

        updateLockedIn();
    }

    //Update the value we can trust from the limelight
    private void updateLimelight() {
        limelightOffset = turret.getLimelightXOffsetDegrees(); //Get the limelight offset from the network table

        //Only change the limelight target if the limelight has a target
        if(turret.doesLimelightHaveTarget()) targetAngle = limelightOffset + turret.getPrevRotaryAngle();
    }

    //The loop that runs as the turret is actively targeting
    private void targetingLoop() {
        if(turret.doesLimelightHaveTarget()) {
            //Set the spinner to the target angle only if the limelight should not be deadbanded
            if(Math.abs(targetAngle - turret.getRotaryAngle()) > LIMELIGHT_DEADBAND) turret.setRotaryTargetAngle(targetAngle);

            //Flywheel and hood control
            double distance = turret.getLimelightDistanceFromCenter();

            turret.setHoodTargetAngle(hoodLUT.get(distance));
            turret.setFlywheelTargetRPM(flywheelLUT.get(distance));

            overRotatedCheck();
        } else {
            //Searching for target
            mode = Searching;

            //Slow down the spinner's velocity for searching for the target
            turret.setRotarySpeed(Search);
        }
    }

    private void overRotatedCheck() {
        if(turret.isRotaryOverextended() && false) {
            //Set the wrap-around point to the opposite side of the direction the spinner is over-extended
            double wrapAroundPoint = turret.getRotaryTarget() == ROTARY_CLOCKWISE_LIMIT ? ROTARY_COUNTER_CLOCKWISE_LIMIT : ROTARY_CLOCKWISE_LIMIT;

            turret.setRotaryTargetAngle(wrapAroundPoint);
            mode = WrapAround;
        }
    }

    private void wrapAroundLoop() {
        if(turret.doesLimelightHaveTarget() && Math.abs(turret.getRotaryTarget() - turret.getRotaryAngle()) < 45) {
            mode = Targeting;
        } else if(!turret.isRotaryBusy()) {
            mode = Targeting;
        }
    }

    //The loop that runs as the turret is searching for a new target in a direction
    private void searchingLoop() {

        //Update the direction the turret is moving if that has been indicated by the turret
        if(!turret.isRotaryBusy()) {
            Direction newDirection = calculateSearchDirection();

            if(newDirection == prevSearchDirection && (turret.getRotaryTarget() == ROTARY_CLOCKWISE_LIMIT || turret.getRotaryTarget() == ROTARY_COUNTER_CLOCKWISE_LIMIT)) {
                newDirection = flipSearchDirection(newDirection);
            }

            setSearchDirection(newDirection);
        }

        //End the searching loop if the limelight has a target
        if(turret.doesLimelightHaveTarget()) {
            mode = Targeting;

            //Return the spinner to the original velocity constraints for live targeting of the limelight
            turret.setRotarySpeed(Normal);
            turret.setRotaryTargetAngle(turret.getRotaryAngle());
        }
    }

    /**
     * Update both the current and previous directions
     * @param direction
     */
    private void setSearchDirection(Direction direction) {
        prevSearchDirection = searchDirection;
        searchDirection = direction;
    }

    private Direction flipSearchDirection(Direction d) {
        return d == Direction.CLOCKWISE ? Direction.COUNTERCLOCKWISE : Direction.CLOCKWISE;
    }

    private Direction calculateSearchDirection() {
        Rotation2d currentAngleToGoal = Geometry.angleToBetweenPoints(odoPoseSupplier.get().getTranslation(), GOAL_POS)
                .plus(odoPoseSupplier.get().getRotation())
                .plus(Rotation2d.fromDegrees(turret.getRotaryAngle()));

        if(currentAngleToGoal.getRadians() >= 0) {
            return Direction.CLOCKWISE;
        } else return Direction.COUNTERCLOCKWISE;

    }

    private void updateLockedIn() {
        if(
            turret.doesLimelightHaveTarget() &&
            !turret.isRotaryBusy() &&
            !turret.isFlywheelBusy() &&
            !turret.isHoodBusy()
        ) indicateLockedInConsumer.accept(true);
        else indicateLockedInConsumer.accept(false);
    }

    /**
     * Turn off the limelight and make sure the spinner is set to it's current angle
     * */
    @Override
    public void end(boolean interrupted) {
        turret.setRotaryTargetAngle(turret.getRotaryAngle());

        timer.stop();
    }

    enum ActiveTrackingMode {
        Targeting, WrapAround, Searching, Starting
    }

    private enum Direction {
        CLOCKWISE, COUNTERCLOCKWISE
    }

}
