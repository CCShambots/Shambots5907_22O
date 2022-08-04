package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.math.InterpLUT;

import java.util.function.DoubleSupplier;

public class ActiveTracking extends CommandBase {

    private Turret turret;
    private DoubleSupplier recommendedStartSupplier;
    private InterpLUT flywheelLUT, hoodLUT;

    //Config values
    private double limelightDeadband = 3; //Degrees in which to deadband the limelight

    //Control variables (control the state of the command)
    private Mode mode = Mode.Targeting;
    private Timer timer;
    private double limelightOffset = 0; //The value of the target off on the limelight (x-axis)
    private double targetAngle = 0; //The shooter's current setpoint
    private Direction prevDirection = Direction.Clockwise;

    public ActiveTracking(Turret turret, DoubleSupplier recommendedStartSupplier, InterpLUT flywheelLUT, InterpLUT hoodLUT) {
        this.turret = turret;
        this.recommendedStartSupplier = recommendedStartSupplier;
        this.flywheelLUT = flywheelLUT;
        this.hoodLUT = hoodLUT;
    }

    /**Turns on the limelight, resets the gyro, and sets the limeLightOffset to whatever is currently on the network table */
    @Override
    public void initialize() {
        turret.setLimelightOn();
        timer = new Timer();

        timer.start();


        //Reset values so the same command instance can be called again
        mode = Mode.Starting;
        limelightOffset = 0;
        targetAngle = recommendedStartSupplier.getAsDouble();

        turret.setRotaryTargetAngle(targetAngle);

    }

    /** */
    @Override
    public void execute() {
        updateLimelight();

        if(mode == Mode.Targeting) targetingLoop();
        if(mode == Mode.WrapAround) {if(!turret.isSpinnerBusy(WRAPAROUND_ERROR)) mode = Mode.Targeting;}
        if(mode == Mode.Searching) searchingLoop();

        SmartDashboard.putBoolean("Turret overextended?", turret.isOverRotated());
        SmartDashboard.putNumber("Limelight offset", limelightOffset);
        SmartDashboard.putString("Mode", mode.name());
    }

    //Update the value we can trust from the limelight
    private void updateLimelight() {
        limelightOffset = turret.getLimelightXOffset(); //Get the limelight offset from the network table


        //Only change the limelight target if the limelight has a target
        if(turret.doesLimelightHaveTarget()) targetAngle = limelightOffset + turret.getPreviousSpinnerAngle();

    }

    //The loop that runs as the turret is actively targeting
    private void targetingLoop() {
        if(turret.doesLimelightHaveTarget()) {
            //Set the spinner to the target angle
            if(Math.abs(targetAngle - turret.getSpinnerAngle()) > limelightDeadband) turret.setSpinnerTarget(targetAngle);

            //Flywheel and hood control
            double distance = turret.getLimelightDistanceFromCenter();

            turret.setHoodTargetAngle(hoodLUT.get(distance));
            turret.setFlywheelTargetRPM(flywheelLUT.get(distance));

            overRotatedCheck();
        } else {
            //Searching for target
            mode = Mode.Searching;

            //Slow down the spinner's velocity for searching for the target
            turret.setSpinnerConstraints(new TrapezoidProfile.Constraints(SEARCH_VEL, SPINNER_MAX_ACCEL));
            setSearchTarget(turret.getSearchDirection());

            prevDirection = turret.getSearchDirection();
        }
    }

    private void overRotatedCheck() {
        if(turret.isOverRotated() && false) {
            //Set the wrap-around point to the opposite side of the direction the spinner is over-extended
            double wrapAroundPoint = turret.getOverRotatedDirection() == Direction.Clockwise ? SPINNER_COUNTERCLOCKWISE_LIMIT : SPINNER_CLOCKWISE_LIMIT;

            turret.setSpinnerTarget(wrapAroundPoint);
            mode = Mode.WrapAround;
        }
    }

    //The loop that runs as the turret is searching for a new target in a direction
    private void searchingLoop() {

        //Update the direction the turret is moving if that has been indicated by the turret
        if(turret.getSearchDirection() != prevDirection) {
            setSearchTarget(turret.getSearchDirection());
        }
        prevDirection = turret.getSearchDirection();

        //End the searching loop if the limelight has a target
        if(turret.doesLimelightHaveTarget()) {
            mode = Mode.Targeting;

            //Return the spinner to the original velocity constraints for live targeting of the limelight
            turret.setSpinnerConstraints(new TrapezoidProfile.Constraints(SPINNER_MAX_VEL, SPINNER_MAX_ACCEL));
            turret.setSpinnerTarget(turret.getSpinnerAngle());
        }

        //If the spinner has reached the limit of it's movement and still doesn't have a target, we will switch the tageting direction
        if(!turret.isSpinnerBusy() && !turret.doesLimelightHaveTarget()) {
            turret.toggleSearchDirection();
        }
    }

    /**
     * Sets the spinner's target to the whatever side is indicated by the passed direction
     */
    private void setSearchTarget(Direction d) {
        if(d == Direction.Clockwise) turret.setSpinnerTarget(SPINNER_CLOCKWISE_LIMIT);
        else turret.setSpinnerTarget(SPINNER_COUNTERCLOCKWISE_LIMIT);

        System.out.print("Set search target to " + d.name());

    }

    /**
     *
     * @return true if the turret is fully locked into the target (target visible, spinner fully moved, in targeting mode, limelight is trusted)
     */
    protected boolean isLockedIn() {
        return
                !turret.isSpinnerBusy() &&                              //True if the turret IS close enough to the target
                        turret.doesLimelightHaveTarget() &&                     //True if the limelight currently is tracking the goal
                        Math.abs(limelightOffset) <= ACCEPTABLE_ERROR &&                    //True if the turret is close enough to the goal
                        mode == Mode.Targeting                                  //True if the turret is actively tracking a target (rather than searching)
                ;
    }


    /**
     * Turn off the limelight and make sure the spinner is set to it's current angle
     * */
    @Override
    public void end(boolean interrupted) {
        turret.setRotaryTargetAngle(turret.getRotaryAngle());

        timer.stop();

        setTurretReadyFlag(false);

        additionalCodeInEnd();
    }
    public void setTurretReadyFlag(boolean value) {
        turret.setIsReadyToShoot(value);
    }

    public enum Mode {
        Targeting, WrapAround, Searching, Starting
    }

}
