package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.config.SwerveModuleConstants;
import static edu.wpi.first.units.Units.*;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final Measure<Distance> trackWidth = Inches.of(22.125);
    public static final Measure<Distance> wheelBase = Inches.of(27.25);
    public static final Measure<Distance> wheelDiameter = Meters.of(Meters.convertFrom(4.0, Inches));
    public static final Measure<Distance> wheelCircumference = Meters.of(wheelDiameter.magnitude() * Math.PI);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static double driveGearRatio = mk4iL1DriveGearRatio;

    public static double angleGearRatio = mk4iL1TurnGearRatio;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase.magnitude() / 2.0, trackWidth.magnitude() / 2.0),
        new Translation2d(wheelBase.magnitude() / 2.0, -trackWidth.magnitude() / 2.0),
        new Translation2d(-wheelBase.magnitude() / 2.0, trackWidth.magnitude() / 2.0),
        new Translation2d(-wheelBase.magnitude() / 2.0, -trackWidth.magnitude() / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 30;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3.25; // meters per second
    public static final double maxAngularVelocity = 2.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = .9 / maxSpeed;//90% feed forward

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 3.04;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int cancoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(253);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int cancoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(108);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int cancoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(207);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 14;
      public static final int cancoderID = 15;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(239);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.05;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
