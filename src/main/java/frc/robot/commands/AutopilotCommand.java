// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */
public class AutopilotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;

private final Autopilot autopilot = new Autopilot(kProfile);

  private static final APConstraints kConstraints = new APConstraints()
    .withAcceleration(15.0)
    .withJerk(2.0);

    private static final APProfile kProfile = new APProfile(kConstraints)
    .withErrorXY(Units.Centimeters.of(2))
    .withErrorTheta(Units.Degrees.of(200))
    .withBeelineRadius(Units.Centimeters.of(8));

    APTarget target;

    private SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withHeadingPID(0.02, 0, 0.0); /* change these values for your robot */

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutopilotCommand(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    target = new APTarget(new Pose2d(5.0, 5.0, new Rotation2d(180.0)))
    .withEntryAngle(new Rotation2d(180.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var driveState = m_drivetrain.getState();
    var fieldOrientedRobotSpeed = driveState.Speeds;

            var output = autopilot.calculate(
                driveState.Pose,
                fieldOrientedRobotSpeed,
                target);
            
                var veloX = output.vx();
                var veloY = output.vy();
                Rotation2d headingReference = output.targetAngle();
                
                m_drivetrain.setControl(m_request
                   .withVelocityX(veloX)
                   .withVelocityY(veloY)
                   .withTargetDirection(headingReference));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autopilot.atTarget(m_drivetrain.getState().Pose, target);
  }
}