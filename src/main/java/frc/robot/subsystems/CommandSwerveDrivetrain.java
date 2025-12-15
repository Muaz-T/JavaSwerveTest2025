package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.DriveCommands;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public static final Translation2d blueReefCenter = new Translation2d(4.489, 4.026);
    private static final double reefOffset = 1.281;
    private static final double startOffset = 1.0;

    boolean lastStageAuto;
    public static AprilTagFieldLayout aprilTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final Pose2d[] blueTagPoses = {
        aprilTagLayout.getTagPose(17).get().toPose2d(), // frontRight
        aprilTagLayout.getTagPose(18).get().toPose2d(), // front
        aprilTagLayout.getTagPose(19).get().toPose2d(), // frontLeft
        aprilTagLayout.getTagPose(20).get().toPose2d(), // backLeft
        aprilTagLayout.getTagPose(21).get().toPose2d(), // back
        aprilTagLayout.getTagPose(22).get().toPose2d(), // backRight
    };

    private static final Pose2d[] redTagPoses = {
        aprilTagLayout.getTagPose(8).get().toPose2d(), // frontRight
        aprilTagLayout.getTagPose(7).get().toPose2d(), // front
        aprilTagLayout.getTagPose(6).get().toPose2d(), // frontLeft
        aprilTagLayout.getTagPose(11).get().toPose2d(), // backLeft
        aprilTagLayout.getTagPose(10).get().toPose2d(), // back
        aprilTagLayout.getTagPose(9).get().toPose2d() // backRight
    };

    // private static double getSquaredDistance(Pose2d pose1, Pose2d pose2) {
    //     return (Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    // }

    // public static HashMap<String, Pose2d> getScoreingLocations(
    //     Translation2d center, double offset, double angle, boolean flip) {
    //   HashMap<String, Pose2d> positions = new HashMap<String, Pose2d>();
    //   // positions.put("center", new Pose2d(center, Rotation2d.fromDegrees(angle)));
    //   Translation2d middle = center.minus(new Translation2d(offset, 0));
    //   Translation2d left;
    //   Translation2d right;
    //   if (flip) {
    //     left = middle.plus(new Translation2d(0, 0.1651));
    //     right = middle.minus(new Translation2d(0, 0.1651));
    //   } else {
    //     left = middle.minus(new Translation2d(0, 0.1651));
    //     right = middle.plus(new Translation2d(0, 0.1651));
    //   }
    //   middle = middle.rotateAround(center, Rotation2d.fromDegrees(angle));
    //   left = left.rotateAround(center, Rotation2d.fromDegrees(angle));
    //   right = right.rotateAround(center, Rotation2d.fromDegrees(angle));
    //   positions.put("middle", new Pose2d(middle, Rotation2d.fromDegrees(angle)));
    //   positions.put("left", new Pose2d(left, Rotation2d.fromDegrees(angle)));
    //   positions.put("right", new Pose2d(right, Rotation2d.fromDegrees(angle)));
    //   return positions;
    // }

    // private static final HashMap<Integer, HashMap<String, Pose2d>> blueReefStartTargets =
    //     new HashMap<Integer, HashMap<String, Pose2d>>();

    // static {
    //     blueReefStartTargets.put(
    //         0, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 60, true));
    //     blueReefStartTargets.put(
    //         1, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 0, true));
    //     blueReefStartTargets.put(
    //         2, getScoreingLocations(blueReefCenter, reefOffset + startOffset, -60, true));
    //     blueReefStartTargets.put(
    //         3, getScoreingLocations(blueReefCenter, reefOffset + startOffset, -120, false));
    //     blueReefStartTargets.put(
    //         4, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 180, false));
    //     blueReefStartTargets.put(
    //         5, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 120, false));
    // }

    // private static final HashMap<Integer, HashMap<String, Pose2d>> blueReefEndTargets =
    // new HashMap<Integer, HashMap<String, Pose2d>>();

//     static {
//         blueReefEndTargets.put(0, getScoreingLocations(blueReefCenter, reefOffset, 60, true));
//         blueReefEndTargets.put(1, getScoreingLocations(blueReefCenter, reefOffset, 0, true));
//         blueReefEndTargets.put(2, getScoreingLocations(blueReefCenter, reefOffset, -60, true));
//         blueReefEndTargets.put(3, getScoreingLocations(blueReefCenter, reefOffset, -120, false));
//         blueReefEndTargets.put(4, getScoreingLocations(blueReefCenter, reefOffset, 180, false));
//         blueReefEndTargets.put(5, getScoreingLocations(blueReefCenter, reefOffset, 120, false));
//     }

//     private static final HashMap<Integer, HashMap<String, Pose2d>> redReefStartTargets =
//     new HashMap<Integer, HashMap<String, Pose2d>>();

//     static {
//     redReefStartTargets.put(
//       0, getScoreingLocations(redReefCenter, startOffset + reefOffset, -120, true));
//     redReefStartTargets.put(
//       1, getScoreingLocations(redReefCenter, startOffset + reefOffset, 180, true));
//     redReefStartTargets.put(
//       2, getScoreingLocations(redReefCenter, startOffset + reefOffset, 120, true));
//     redReefStartTargets.put(
//       3, getScoreingLocations(redReefCenter, startOffset + reefOffset, 60, false));
//     redReefStartTargets.put(
//       4, getScoreingLocations(redReefCenter, startOffset + reefOffset, 0, false));
//     redReefStartTargets.put(
//       5, getScoreingLocations(redReefCenter, startOffset + reefOffset, -60, false));
// }



//     private static Pose2d[] getClosestPath(Pose2d currentPose, boolean isBlue, String offset) {
//         Pose2d[] tagPoses;
//         if (isBlue) {
//             tagPoses = blueTagPoses;
//         } else {
//         tagPoses = redTagPoses;
//     }
//     int closestIndex = 0;
//     double closestDistance = getSquaredDistance(currentPose, tagPoses[0]);
//     for (int i = 1; i < tagPoses.length; i++) {
//       double distance = getSquaredDistance(currentPose, tagPoses[i]);
//       if (distance < closestDistance) {
//         closestIndex = i;
//         closestDistance = distance;
//       }
//     }
//     if (isBlue) {
//       Pose2d[] targets = {
//         blueReefStartTargets.get(closestIndex).get(offset),
//         blueReefEndTargets.get(closestIndex).get(offset)
//       };
//       return targets;
//     } else {
//       Pose2d[] targets = {
//         redReefStartTargets.get(closestIndex).get(offset),
//         redReefEndTargets.get(closestIndex).get(offset)
//       };
//       return targets;
//     }
//   }


private static final PathConstraints teleopPathConstraints =
new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

public Command getLeftCoralDriveCommand() {

    Supplier<Command> pathfindingCommand =
        () -> AutoBuilder.pathfindToPose(new Pose2d(5.0, 5.0, Rotation2d.k180deg), teleopPathConstraints);
    Command command =
        defer(pathfindingCommand)
            .andThen(() -> lastStageAuto = true)
            .andThen(
                defer(
                        () ->
                            DriveCommands.driveToPoseCommand(
                                this, new Pose2d(6.0, 5.0, Rotation2d.kPi)))
                    .beforeStarting(() -> lastStageAuto = true)
                    .finallyDo(() -> lastStageAuto = false));
    return command;
  }
}
