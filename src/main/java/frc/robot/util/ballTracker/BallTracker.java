package frc.robot.util.ballTracker;

import frc.robot.Constants;
import frc.robot.Constants.AllianceColor;
import frc.robot.subsystems.Conveyor;
import frc.robot.util.ballTracker.Ball.BallColorType;
import frc.robot.util.ballTracker.Ball.BallPosition;
import frc.robot.util.hardware.ColorSensor;
import frc.robot.util.hardware.ColorSensor.ColorSensorOutput;
import frc.robot.util.hardware.ProximitySensor;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import static frc.robot.subsystems.Conveyor.MotorState.*;
import static frc.robot.util.ballTracker.Ball.BallPosition.*;
import static frc.robot.util.hardware.ColorSensor.ColorSensorOutput.*;

public class BallTracker {
    private ColorSensor leftColor;
    private ColorSensor rightColor;
    private ProximitySensor rightProx;
    private ProximitySensor leftProx;
    private ProximitySensor centerProx;
    private Conveyor conveyor;

    //The balls that can exist that are currently in the robot
    private Ball emptyBall = new Ball(BallColorType.NotInBot, NotInBot);
    List<Ball> balls = List.of(
            emptyBall.copy(),
            emptyBall.copy(),
            emptyBall.copy()
    );

    public BallTracker(ColorSensor leftColor, ColorSensor rightColor, ProximitySensor leftProx, ProximitySensor rightProx, ProximitySensor centerProx, Conveyor conveyor) {
        this.leftColor = leftColor;
        this.rightColor = rightColor;
        this.rightProx = rightProx;
        this.leftProx = leftProx;
        this.centerProx = centerProx;
        this.conveyor = conveyor;

        leftProx.registerTrigger(true, () -> {
                if(conveyor.getLeftConveyorState() == Intaking) {
                    if(findBall(PastLeft) != null) {
                        safeSetBallPos(Left, PastLeft);
                    } else {
                        createNewBall(evaluateBallColorType(leftColor.getColor()), Left);
                    }
                } else if(conveyor.getLeftConveyorState() == Exhausting) {
                    safeSetBallPos(BetweenLeftAndCenter, Center);
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
            if(conveyor.getRightConveyorState() == Intaking) {
                if(findBall(PastRight) != null) {
                    safeSetBallPos(Right, PastRight);
                } else {
                    createNewBall(evaluateBallColorType(rightColor.getColor()), Right);
                }
            } else if(conveyor.getRightConveyorState() == Exhausting) {
                safeSetBallPos(BetweenRightAndCenter, Center);
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
        AllianceColor color = Constants.alliance;
        if(colorSensorOutput == Red && Constants.alliance == Constants.AllianceColor.Red) {
            return BallColorType.Ours;
        } else if(colorSensorOutput == Red && Constants.alliance == Constants.AllianceColor.Blue) {
            return BallColorType.Opposing;
        } else if(colorSensorOutput == Blue && Constants.alliance == Constants.AllianceColor.Blue) {
            return BallColorType.Ours;
        } else if(colorSensorOutput == Blue && Constants.alliance == Constants.AllianceColor.Red) {
            return BallColorType.Opposing;
        } else {
            //Something has gone terribly wrong, so we'll just assume the ball is ours
            return BallColorType.Ours;
        }
    }

    //TODO: Implement tracker error if needed (hopefully not needed)

    private void safeSetBallPos(BallPosition targetPos, BallPosition searchPos) {
        Ball ball = findBall(searchPos);
        if(ball != null) {
            ball.setPosition(targetPos);

            //If the ball is no longer in the bot, we want to remove it from the list
            if(targetPos == NotInBot) removeBall(ball);
        }
    }

    public Ball findBall(BallPosition pos) {
        return getActualBalls().stream().filter((e) -> e.getPosition() == pos).findFirst().orElse(null);
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
