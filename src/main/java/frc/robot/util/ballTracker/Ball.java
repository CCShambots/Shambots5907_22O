package frc.robot.util.ballTracker;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ball implements Sendable{
    //TODO: Rename "Conveyor" Branch to "Conveyor-base"

    private BallColorType color;
    private BallPosition position;

    public Ball(BallColorType color, BallPosition position) {
        this.color = color;
        this.position = position;
    }

    //TODO: Change access to "void" instead of "public"

    public void advancePosition() {position = position.next();}
    public void regressPosition() {position = position.previous();}
    public void setPosition(BallPosition pos) {position = pos;}
    public BallPosition getPosition() {return position;}
    public BallColorType getColor() {return color;}
    void setColor(BallColorType color) {this.color = color;}
    
    /**
     * @param pos The position that we're trying to move the ball to
     * @return whether that is indeed the next position that should next be moved to
     */
    boolean isNextPosition(BallPosition pos) {return position.next().equals(pos);}

    /**
     * @param pos The position that we're trying to move the ball to
     * @return whether that is indeed the previous position that the ball should go to
     */
    boolean isPrevPosition(BallPosition pos) {return position.previous().equals(pos);}

    public static enum BallPosition {
        PastLeft, //Held in the left compactor
        Left, //Left sensor is tripped
        BetweenLeftAndCenter, //Left and center tripped
        Center, //Only center tripped
        BetweenRightAndCenter, //Right and center tripped
        Right, //Right sensor tripped
        PastRight, //Held in the right compactor
        NotInBot; //The ball has left the robot


        private static BallPosition[] vals = values();
        public BallPosition next() { return vals[(this.ordinal() +1) % vals.length];}
        public BallPosition previous() { 
            if(this.ordinal() - 1 == -1) return BallPosition.NotInBot;
            return vals[(this.ordinal() -1) % vals.length];
        }

    }

    public Ball copy() {
        return new Ball(getColor(), getPosition());
    }

    /**
     * Set the position and color of this ball to that of `other`
     * @param other
     */
    public void conformTo(Ball other) {
        setPosition(other.getPosition());
        setColor(other.getColor());
    }

    public enum BallColorType {
        Ours, //Our alliance's colored balls
        Opposing, //The opposing alliance's colored balls
        NotInBot, //The ball is not in the robot 
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Ball");
        builder.addStringProperty("Position", () -> getPosition().name(), null);
        builder.addStringProperty("Color", () -> getColor().name(), null);
        
    }
}
