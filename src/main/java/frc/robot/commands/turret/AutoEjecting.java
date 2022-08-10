package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.util.math.Geometry;
import frc.robot.util.math.InterpLUT;

import java.util.function.Supplier;

import static frc.robot.Constants.Turret.*;

public class AutoEjecting extends CommandBase {
    private Turret t;
    private InterpLUT RPMLUT;
    private Supplier<Pose2d> odoPoseSupplier;

    public AutoEjecting(Turret t, InterpLUT RPMLUT, Supplier<Pose2d> odoPoseSupplier) {
        this.t = t;
        this.RPMLUT = RPMLUT;
        this.odoPoseSupplier = odoPoseSupplier;

        addRequirements(t);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //Calculate target angle
        Pose2d currentPose = odoPoseSupplier.get();

        Translation2d targetPoint = Geometry.getEjectionTargetPoint(currentPose);

        t.setRotaryTargetAngle(Geometry.getTurretTarget(currentPose, targetPoint).getDegrees());

        //Update flywheel velo
        t.setFlywheelTargetRPM(RPMLUT.get(targetPoint.getDistance(currentPose.getTranslation())));
    }

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {
    }
}
