package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardInterface {

    private static DashboardInterface instance;

    private DashboardInterface() {
        this.setTab(DashboardInterface.Tab.Auto);
    }

    public static DashboardInterface getInstance() {
        if(instance == null) instance = new DashboardInterface();
        return instance;
    }

    public void setTab(DashboardInterface.Tab t) {
        SmartDashboard.putString("Tab", t.name());
    }

    public static enum Tab {
        Auto,
        Teleop,
        Test,
        Debug,
        Pathing
    }
}
