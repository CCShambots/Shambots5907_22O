package frc.robot.util.ballTracker;

import frc.robot.Constants;
import frc.robot.Constants.AllianceColor;
import frc.robot.subsystems.Conveyor;
import frc.robot.util.ballTracker.Ball.BallColorType;
import frc.robot.util.ballTracker.Ball.BallPosition;
import frc.robot.util.hardware.ColorSensor;
import frc.robot.util.hardware.ColorSensor.ColorSensorOutput;
import frc.robot.util.hardware.ProximitySensor;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static frc.robot.subsystems.Conveyor.MotorState.*;
import static frc.robot.util.ballTracker.Ball.BallPosition.*;
import static frc.robot.util.hardware.ColorSensor.ColorSensorOutput.*;

public class BallTracker {
    private boolean colorDetectionEnabled = false;

    //The balls that can exist that are currently in the robot
    private Ball emptyBall = new Ball(BallColorType.NotInBot, NotInBot);
    List<Ball> balls = List.of(
            emptyBall.copy(),
            emptyBall.copy(),
            emptyBall.copy()
    );

    public BallTracker(ColorSensor leftColor, ColorSensor rightColor, ProximitySensor leftProx, ProximitySensor rightProx, ProximitySensor centerProx, Conveyor conveyor) {

        leftProx.registerTrigger(true, () -> {
                    if(findBall(PastLeft) != null) {
                        safeSetBallPos(Left, PastLeft);
                    } else if(conveyor.getLeftConveyorState() == Exhausting) {
                        safeSetBallPos(BetweenLeftAndCenter, Center);
                    } else if(conveyor.getLeftConveyorState() == Intaking) {
                        createNewBall(evaluateBallColorType(leftColor.getColor()), Left);
                    }
            }
        );

        leftProx.registerTrigger(false, () -> {
            if(conveyor.getLeftConveyorState() == Intaking) {
                safeSetBallPos(Center, BetweenLeftAndCenter);
            } else if(conveyor.getLeftConveyorState() == Exhausting) {
                safeSetBallPos(PastLeft, Left);
            }
        });

        centerProx.registerTrigger(true, () -> {
            if(conveyor.getLeftConveyorState() == Intaking) {
                if(leftProx.isActivated()) {
                    safeSetBallPos(BetweenLeftAndCenter, Left);
                    return;
                }
            }
            if(conveyor.getRightConveyorState() == Intaking) {
                if(rightProx.isActivated()) {
                    safeSetBallPos(BetweenRightAndCenter, Right);
                }
            }
        });

        centerProx.registerTrigger(false, () -> {
           if(conveyor.getRightConveyorState() == Exhausting) {
               safeSetBallPos(Right, BetweenRightAndCenter);
           } else if(conveyor.getLeftConveyorState() == Exhausting) {
               safeSetBallPos(Left, BetweenLeftAndCenter);
           } else if(conveyor.getLeftConveyorState() == Intaking && conveyor.getRightConveyorState() == Intaking) {
               removeBall(findBall(Center));
           }
        });

        rightProx.registerTrigger(true, () -> {
            if(findBall(PastRight) != null) {
                safeSetBallPos(Right, PastRight);
            } else if(conveyor.getRightConveyorState() == Exhausting) {
                    safeSetBallPos(BetweenRightAndCenter, Center);
            } else if(conveyor.getRightConveyorState() == Intaking){
                createNewBall(evaluateBallColorType(rightColor.getColor()), Right);
            }
        });

        rightProx.registerTrigger(false, () -> {
            if(conveyor.getRightConveyorState() == Intaking) {
                safeSetBallPos(Center, BetweenRightAndCenter);
            } else if(conveyor.getRightConveyorState() == Exhausting) {
                safeSetBallPos(PastRight, Right);
            }
        });
    }

    public BallColorType evaluateBallColorType(ColorSensorOutput colorSensorOutput) {
        //Only evaluate color if that feature is enabled
        if(colorDetectionEnabled) {
            AllianceColor allianceColor = Constants.alliance;
            if(colorSensorOutput == Red && allianceColor == Constants.AllianceColor.Red) {
                return BallColorType.Ours;
            } else if(colorSensorOutput == Red && allianceColor == Constants.AllianceColor.Blue) {
                return BallColorType.Opposing;
            } else if(colorSensorOutput == Blue && allianceColor == Constants.AllianceColor.Blue) {
                return BallColorType.Ours;
            } else if(colorSensorOutput == Blue && allianceColor == Constants.AllianceColor.Red) {
                return BallColorType.Opposing;
            } else {
                //Something has gone terribly wrong, so we'll just assume the ball is ours
                return BallColorType.Ours;
            }
        } else return BallColorType.Ours;
        
    }

    private void safeSetBallPos(BallPosition targetPos, BallPosition searchPos) {
        Ball ball = findBall(searchPos);

        if(ball == null) {
            ball = guessBall(targetPos, searchPos); //If no ball was found, try to guess one
        }

        if(ball != null) {
            ball.setPosition(targetPos);

            //If the ball is no longer in the bot, we want to remove it from the list
            if(targetPos == NotInBot) removeBall(ball);
        }
    }

    public Ball findBall(BallPosition pos) {
        return getActualBalls().stream().filter((e) -> e.getPosition() == pos).findFirst().orElse(null);
    }

    public Ball findBall(BallPosition... pos) {
        for(BallPosition specificPos : pos) {
            Ball found = findBall(pos);

            if(found != null) return found;
        }

        return null;
    }

    public Ball guessBall(BallPosition targetPos, BallPosition originalSearchPos) {
        Direction searchDirection = evaluateDirection(targetPos, originalSearchPos);

        //Find all balls to the correct direction of the ball
        List<Ball> potentials = getActualBalls().stream().filter((e) -> evaluateDirection(targetPos, e.getPosition()) == searchDirection)
                .collect(Collectors.toList());

        if(potentials.size() > 0) {
            Comparator<Ball> comparator =
                    (e1, e2) -> evaluateDistance(targetPos, e1.getPosition()) - evaluateDistance(targetPos, e1.getPosition());
            potentials.sort(comparator);

            return potentials.get(0); //First element in the list will be the one with the least distance (the best solution)
        }

        return null;
    }

    private int evaluateDistance(BallPosition startingPos, BallPosition endPos) {
        return Math.abs(endPos.ordinal() - startingPos.ordinal());
    }


    private Direction evaluateDirection(BallPosition startingPos, BallPosition endPos) {
        return endPos.ordinal() > startingPos.ordinal() ? Direction.Right : Direction.Left;

//        //No need to account for starting and end pos being the same because that should never happen
//        switch (startingPos) {
//            case PastLeft: return Direction.Right;
//            case PastRight: return Direction.Left;
//            case Left: return endPos == PastLeft ? Direction.Left : Direction.Right;
//            case Right: return endPos == PastRight ? Direction.Right : Direction.Left;
//            case BetweenLeftAndCenter: return equalAnyPos(endPos, PastLeft, Left) ? Direction.Left : Direction.Right;
//            case BetweenRightAndCenter: return equalAnyPos(endPos, PastRight, Right) ? Direction.Right : Direction.Left;
//            case Center: return equalAnyPos(endPos, PastRight, Right, BetweenRightAndCenter) ? Direction.Right : Direction.Left;
//            case NotInBot: return null; //Never would be searching for a ball starting not in bot
//        }
//
//        //Null return should never get called because there's a switch for every possibility
//        return null;
    }

    private boolean equalAnyPos(BallPosition startingPos, BallPosition... otherPos) {
        for(BallPosition other : otherPos) {
            if (startingPos == other) return true;
        }

        return false;
    }

    private enum Direction {
        Left, Right
    }

    /**
     * Finds all the balls at any of the following positions
     * @param positions
     * @return
     */
    public List<Ball> findBallsAtPositions(BallPosition... positions) {
        return getActualBalls().stream().filter((e) -> {
            boolean match = false;
            for(BallPosition pos : positions) {
                if(e.getPosition() == pos) match = true;
            }
            return match;
        }).collect(Collectors.toList());
    }

    /**
     * Find a ball at given position with protection for null values
     * @return
     */
    public Ball findBallSafe(BallPosition pos) {
        Ball foundBall = findBall(pos);

        return foundBall != null ? foundBall : emptyBall;
    }


    public boolean isBallAtPos(BallPosition pos) {
        return findBall(pos) != null;
    }

    public int getNumberOfBalls() {
        return getActualBalls().size();
    }

    private List<Ball> getActualBalls() {
        return balls.stream().filter((e) -> e.getPosition() != NotInBot).collect(Collectors.toList());
    }

    public boolean doesBotHaveBall(Ball ball) {
        return balls.contains(ball);
    }

    public List<Ball> getBallsOfColor(BallColorType color) {
        return balls.stream().filter((e) -> e.getColor() == color && e.getPosition() != NotInBot).collect(Collectors.toList());
    }

    public void createNewBall(BallColorType color, BallPosition position) {
        Ball originalBall = getEmptyBall();
        if(originalBall == null) {
            originalBall = emptyBall.copy();
            balls.add(originalBall);
        }

        originalBall.setColor(color);
        originalBall.setPosition(position);
    }

    public void removeBall(Ball ball) {
        if(ball != null) {
            ball.setColor(BallColorType.NotInBot);
            ball.setPosition(NotInBot);
        }
    }

    public Ball getEmptyBall() {
        return balls.stream().filter((e) -> e.getPosition() == NotInBot).findFirst().orElse(null);
    }

    public Ball getBall1() {
        return balls.get(0);
    }

    public Ball getBall2() {
        return balls.get(1);
    }

    public Ball getBall3() {
        return balls.get(2);
    }

    public void disableColorDetection() {
        colorDetectionEnabled = false;
    }

    public void enableColorDetection() {
        colorDetectionEnabled = true;
    }

    public boolean isColorDetectionEnabled() {
        return colorDetectionEnabled;
    }


    /**
     * This method should only be called once the balls in the robot are not moving at all, as that could cause some issues with the sensors
     * @param newBallList
     */
    public void resetTracker(List<Ball> newBallList) {
        for(int i = 0; i<newBallList.size(); i++) {
            balls.get(i).conformTo(newBallList.get(i));
        }
    }


}
