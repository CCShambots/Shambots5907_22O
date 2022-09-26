package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SwerveDrivetrain.*;

public class DriveCommand extends CommandBase{
    private Drivetrain drivetrain;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier turnSupplier;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_LINEAR_ACCELERATION);
    private SlewRateLimiter thetaLimiter = new SlewRateLimiter(MAX_ROT_ACCEL);

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turnSupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      double correctedX =  convertRawInput(xSupplier.getAsDouble(), MAX_LINEAR_SPEED);
      double correctedY =  convertRawInput(ySupplier.getAsDouble(), MAX_LINEAR_SPEED);
      double correctedRot =  convertRawInput(turnSupplier.getAsDouble(), MAX_ROTATION);

      ChassisSpeeds speeds;

      if(drivetrain.isFieldRelative()) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          correctedX, correctedY, correctedRot,
          drivetrain.getCurrentAngle()
        );
      } else {
        speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
      }

      speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
      speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
      speeds.omegaRadiansPerSecond = thetaLimiter.calculate(speeds.omegaRadiansPerSecond);
      
      drivetrain.drive(speeds);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    
    private double convertRawInput(double rawInput, double maxOutput){
      double deadbandInput = deadband(rawInput, Constants.ControllerConversions.DEADBAND);
      double scaledOutput = Constants.ControllerConversions.conversionFunction.apply(Double.valueOf(deadbandInput))
      * maxOutput;
      return scaledOutput;
    }
    
    private double deadband(double rawInput, double deadband){
      if (Math.abs(rawInput) > deadband) {
        if (rawInput > 0.0) return (rawInput - deadband) / (1.0 - deadband);
        else return (rawInput + deadband) / (1.0 - deadband);
      } else return 0;
    }
}
