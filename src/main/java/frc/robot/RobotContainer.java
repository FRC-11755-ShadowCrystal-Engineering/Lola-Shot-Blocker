package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Dozer;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController xbox = new CommandXboxController(Constants.DRIVER_PORT);
  private final Dozer dozer = new Dozer();
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.run(() -> {
      double forward = MathUtil.applyDeadband(xbox.getLeftY(), Constants.DEADBAND);
      double left = MathUtil.applyDeadband(xbox.getLeftX(), Constants.DEADBAND);
      double turn = MathUtil.applyDeadband(xbox.getRightX(), Constants.DEADBAND);
      swerve.driveWithJoysticks(forward, left, turn);
      SmartDashboard.putBoolean("Field Relative", swerve.isFieldRelative());
    }));
    xbox.y().onTrue(Commands.runOnce(swerve::zeroGyro, swerve));
    xbox.a().onTrue(dozer.toggleArmSetpoint());
  }

  // Safe baseline auto: moves forward, then backward using robot-oriented control.
  // WARNING: Start with robot raised off the ground and verify direction.
  public Command getAutonomousCommand() {
    return Commands.sequence(
      swerve.run(() -> swerve.getSwerveDrive().drive(
          new edu.wpi.first.math.geometry.Translation2d(0.4, 0), 0, false, false)).withTimeout(1.0),
      swerve.run(() -> swerve.getSwerveDrive().drive(
          new edu.wpi.first.math.geometry.Translation2d(-0.4, 0), 0, false, false)).withTimeout(1.0),
      Commands.runOnce(swerve::stop, swerve)
    );
  }
}
