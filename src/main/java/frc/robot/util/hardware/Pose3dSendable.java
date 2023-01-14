package frc.robot.util.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Pose3dSendable extends Pose3d implements Sendable {

    public Pose3dSendable() {
        super(0, 0, 0, new Rotation3d());
    }

    public Pose3dSendable(Translation3d translation, Rotation3d rotation) {
        super(translation, rotation);
    }

    public Pose3dSendable(double x, double y, double z, Rotation3d rotation) {
        super(x, y, z, rotation);
    }

    public Pose3dSendable(Pose2d pose) {
        super(pose);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("translation/x", this::getX, null);
        builder.addDoubleProperty("translation/y", this::getY, null);
        builder.addDoubleProperty("translation/z", this::getZ, null);
        builder.addDoubleProperty("rotation/roll", () -> this.getRotation().getX(), null);
        builder.addDoubleProperty("rotation/pitch", () -> this.getRotation().getY(), null);
        builder.addDoubleProperty("rotation/yaw", () -> this.getRotation().getZ(), null);
    }
}
