package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
   // The gyro sensor

   private final AHRS gyro;

  private SwerveDrivePoseEstimator swervePoseEstimator;
  
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new AHRS(SPI.Port.kMXP, (byte) 100);
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    

      swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    resetModuleEncoders();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    SmartDashboard.putNumber("DRIVE",translation.getX());
    
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetModuleEncoders() {
    mSwerveMods[0].resetAngleToAbsolute();
    mSwerveMods[1].resetAngleToAbsolute();
    mSwerveMods[2].resetAngleToAbsolute();
    mSwerveMods[3].resetAngleToAbsolute();
  }

  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void resetPoseEstimator(Pose2d pose) {
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder((gyro.getAngle()), 360);
  }

  @Override
  public void periodic() {
    swervePoseEstimator.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("X Meters", swervePoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Meters", swervePoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Yaw",getHeadingDegrees());
  }
}
