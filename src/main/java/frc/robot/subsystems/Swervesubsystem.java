package frc.robot.subsystems;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FalconSwerveModule;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Libs.SwerveDrive;
import frc.robot.Libs.SwerveUtils;

public class Swervesubsystem extends SubsystemBase {
  private WPI_TalonFX RightFront;
  private WPI_TalonFX LeftFront;
  private WPI_TalonFX RightBack;
  private WPI_TalonFX LeftBack;

  private WPI_TalonFX rRightFront;
  private WPI_TalonFX rLeftFront;
  private WPI_TalonFX rRightBack;
  private WPI_TalonFX rLeftBack;

  private WPI_CANCoder RightFrontEncoder;
  private WPI_CANCoder LeftFrontEncoder;
  private WPI_CANCoder RightBackEncoder;
  private WPI_CANCoder LeftBackEncoder;

  private FalconSwerveModule TopLeft;
  private FalconSwerveModule TopRight;
  private FalconSwerveModule BottomLeft;
  private FalconSwerveModule BottomRight;
  private FalconSwerveModule[] modules;

  private Pigeon2 gyro;

  private SwerveDriveKinematics kinematics;

  private SwerveDrive swerveDrive;
  private SwerveUtils swerveUtils;

  private boolean field;

  public Swervesubsystem() {
      LeftFront = new WPI_TalonFX(DRIVETRAIN.FL_DRIVE_ID);
      RightFront = new WPI_TalonFX(DRIVETRAIN.FR_DRIVE_ID);
      LeftBack = new WPI_TalonFX(DRIVETRAIN.BL_DRIVE_ID);
      RightBack = new WPI_TalonFX(DRIVETRAIN.BR_DRIVE_ID);

      rLeftFront = new WPI_TalonFX(DRIVETRAIN.FL_AZIMUTH_ID);
      rRightFront = new WPI_TalonFX(DRIVETRAIN.FR_AZIMUTH_ID);
      rLeftBack = new WPI_TalonFX(DRIVETRAIN.BL_AZIMUTH_ID);
      rRightBack = new WPI_TalonFX(DRIVETRAIN.BR_AZIMUTH_ID);

      LeftFrontEncoder = new WPI_CANCoder(DRIVETRAIN.FL_CANCODER_ID);
      RightFrontEncoder = new WPI_CANCoder(DRIVETRAIN.FR_CANCODER_ID);
      LeftBackEncoder = new WPI_CANCoder(DRIVETRAIN.BL_CANCODER_ID);
      RightBackEncoder = new WPI_CANCoder(DRIVETRAIN.BR_CANCODER_ID);

      gyro = new Pigeon2(0);
      gyro.setYaw(0);

      TopLeft = new FalconSwerveModule(LeftFront, rLeftFront, 
                LeftFrontEncoder, DRIVETRAIN.FL_ECODER_OFFSET);
      TopRight = new FalconSwerveModule(RightFront, rRightFront, 
                  RightFrontEncoder ,DRIVETRAIN.FR_ECODER_OFFSET);
      BottomLeft = new FalconSwerveModule(LeftBack, rLeftBack, 
                  LeftBackEncoder, DRIVETRAIN.BL_ECODER_OFFSET);
      BottomRight = new FalconSwerveModule(RightBack, rRightBack, 
                  RightBackEncoder, DRIVETRAIN.BR_ECODER_OFFSET);

      kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN.ROBOT_WIDTH/2, DRIVETRAIN.ROBOT_WIDTH/2),
        new Translation2d(DRIVETRAIN.ROBOT_WIDTH/2, -DRIVETRAIN.ROBOT_WIDTH/2),
        new Translation2d(-DRIVETRAIN.ROBOT_WIDTH/2, DRIVETRAIN.ROBOT_WIDTH/2),
        new Translation2d(-DRIVETRAIN.ROBOT_WIDTH/2, -DRIVETRAIN.ROBOT_WIDTH/2)
      );

      modules = new FalconSwerveModule[] {TopLeft, TopRight, BottomLeft, BottomRight};

      swerveDrive = new SwerveDrive(
        modules, 
        gyro, 
        kinematics, 
        DRIVETRAIN.MAX_LINEAR_SPEED
        );

      swerveUtils = new SwerveUtils(
        new PIDConstants(DRIVETRAIN._translationKp, DRIVETRAIN._translationKi, DRIVETRAIN._translationKd),
        new PIDConstants(DRIVETRAIN._rotationKp, DRIVETRAIN._rotationKi, DRIVETRAIN._rotationKd),
        swerveDrive
        );

        field = false;
  }

  public void swerveDrive(double x_speed, double y_speed, double orientation) {
    Translation2d translation = new Translation2d(x_speed, y_speed);

    swerveDrive.drive(translation, orientation, getField().getAsBoolean(), false);
  }

  public void stop() {}

  public Command getAuton() {
    return swerveUtils.followPath("Holonomic path",
                          new HashMap<String,Command>(),
                          false,
                          this
                          );
  }

  public Command toggleField() {
    return new InstantCommand(() -> field ^= field);
  }

  public BooleanSupplier getField() {
    return () -> field;
  }

  @Override
  public void periodic() {
    swerveUtils.updateOdometry();

    for(int i = 0; i <= modules.length - 1; i++) {
      SmartDashboard.putNumber("Module " + i + " Angle", modules[i].getAngleRads().getDegrees());
      SmartDashboard.putNumber("Module " + i + " Meters", modules[i].getDriveMeters());
      SmartDashboard.putNumber("Module " + i + " Velocity", modules[i].getDriveVelocity());
    }

    SmartDashboard.putBoolean("Field", field);
  }

  @Override
  public void simulationPeriodic() {}
}