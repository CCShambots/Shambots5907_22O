package frc.robot.util.currentRegulator;

import java.util.ArrayList;

public class CurrentRegulator {
    private static CurrentRegulator instance = null;
    private ArrayList<RegulatedCommand> commands;

    private double maxCurrentDraw;

    public CurrentRegulator() {
        commands = new ArrayList<>();
    }

    public static CurrentRegulator getInstance() {
        instance = instance == null ? new CurrentRegulator() : instance;
        return instance;
    }

    public void registerCommand(RegulatedCommand command) {
        commands.add(command);
        updateCommandCurrentLimits();
    }

    private void updateCommands() {
        ArrayList<RegulatedCommand> newCommands = new ArrayList<>();

        for (RegulatedCommand command : commands) {
            if (!command.isFinished()) {
                newCommands.add(command);
            }
        }

        commands = newCommands;
    }

    public void updateCommandCurrentLimits() {
        updateCommands();
        limitTotalCurrent();

        double increaseFactor = maxCurrentDraw/getTotalCurrentMin();

        for (RegulatedCommand command : commands) {
            command.setCurrent(command.getMinCurrentTotal() * increaseFactor);
        }
    }

    private void limitTotalCurrent() {
        while (getTotalCurrentMin() > maxCurrentDraw) {
            RegulatedCommand toCancel = getLeastImportantCommand();
            toCancel.cancel();
            commands.remove(toCancel);
        }
    }

    private double getTotalCurrentMin() {
        double totalMin = 0;
        for (RegulatedCommand command : commands) {
            totalMin += command.getMinCurrentTotal();
        }

        return totalMin;
    }

    private RegulatedCommand getLeastImportantCommand() {
        RegulatedCommand out = commands.get(0);

        for (RegulatedCommand command : commands) {
            if (command.getPriority() < out.getPriority()) {
                out = command;
            }
        }

        return out;
    }

    public void update() {
        //TODO: log ig idk
    }
}
