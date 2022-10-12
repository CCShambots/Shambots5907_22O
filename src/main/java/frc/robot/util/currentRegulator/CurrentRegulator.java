package frc.robot.util.currentRegulator;

import java.util.ArrayList;

public class CurrentRegulator {
    private CurrentRegulator instance = null;
    private ArrayList<RegulatedCommand> commands;

    private double maxCurrentDraw;

    public CurrentRegulator() {
        commands = new ArrayList<>();
    }

    public CurrentRegulator getInstance() {
        instance = instance == null ? new CurrentRegulator() : instance;
        return instance;
    }

    public void registerCommand(RegulatedCommand command) {
        commands.add(command);
    }

    public void updateCommands() {
        ArrayList<RegulatedCommand> newCommands = new ArrayList<>();

        for (RegulatedCommand command : commands) {
            if (!command.isFinished()) {
                newCommands.add(command);
            }
        }

        commands = newCommands;
    }

    public void setCurrentLimits() {
        double totalMin = 0;
        for (RegulatedCommand command : commands) {
            totalMin += command.getMinCurrentTotal();
        }

        if (totalMin > maxCurrentDraw) {
            //do something
        }
        else {
            //do something else
        }
    }
}
