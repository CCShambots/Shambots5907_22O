package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SwerveDrivetrain.*;

public class DriveCommand extends CommandBase{
    private Drivetrain drivetrain;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier turnSupplier;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(4);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(4);
    private SlewRateLimiter thetaLimiter = new SlewRateLimiter(4);


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
      double correctedX =  convertRawInput(xLimiter.calculate(xSupplier.getAsDouble()), MAX_LINEAR_SPEED);
      double correctedY =  convertRawInput(yLimiter.calculate(ySupplier.getAsDouble()), MAX_LINEAR_SPEED);
      double correctedRot =  convertRawInput(thetaLimiter.calculate(turnSupplier.getAsDouble()), MAX_ROTATION);

      ChassisSpeeds speeds;

      if(drivetrain.isFieldRelative()) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          correctedX, correctedY, correctedRot,
          drivetrain.getCurrentAngle()
        );
      } else {
        speeds = new ChassisSpeeds(correctedX, correctedY, correctedRot);
      }

      //TODO: Remove
      SmartDashboard.putNumber("drivetrain/Target chassis speed x", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("drivetrain/Target chassis speed y", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("drivetrain/Target chassis speed theta", Math.toDegrees(speeds.omegaRadiansPerSecond));


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
