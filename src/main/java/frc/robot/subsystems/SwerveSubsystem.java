package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive drive;
  private boolean fieldRelative = true;

  public SwerveSubsystem() {
    try {
      File folder = new File(Filesystem.getDeployDirectory(), "swerve");
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      drive = new SwerveParser(folder).createSwerveDrive(Constants.MAX_SPEED_METERS_PER_SECOND);
    } catch (Exception e) {
      throw new RuntimeException("Could not initialize YAGSL. Check deploy/swerve JSON and hardware.", e);
    }
  }

  public void driveWithJoysticks(double forward, double left, double turn) {
    Translation2d translation = new Translation2d(
        forward * Constants.MAX_SPEED_METERS_PER_SECOND * Constants.DRIVE_SCALE,
        left * Constants.MAX_SPEED_METERS_PER_SECOND * Constants.DRIVE_SCALE);
    double rotation = turn * drive.getMaximumChassisAngularVelocity() * Constants.TURN_SCALE;
    drive.drive(translation, rotation, fieldRelative, false);
  }
  public void toggleFieldRelative() { fieldRelative = !fieldRelative; }
  public boolean isFieldRelative() { return fieldRelative; }
  public void zeroGyro() { drive.zeroGyro(); }
  public void lock() { drive.lockPose(); }
  public void stop() { drive.drive(new Translation2d(), 0, false, false); }
  public SwerveDrive getSwerveDrive() { return drive; }
}
