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
import frc.robot.Libs.HolonomicDrive;
import frc.robot.Libs.SwerveUtils;

public class Swervesubsystem extends SubsystemBase {
  private WPI_TalonFX FrontRight;
  private WPI_TalonFX FrontLeft;
  private WPI_TalonFX BackRight;
  private WPI_TalonFX BackLeft;

  private WPI_TalonFX rFrontRight;
  private WPI_TalonFX rFrontLeft;
  private WPI_TalonFX rBackRight;
  private WPI_TalonFX rBackLeft;

  private WPI_CANCoder FrontRightEncoder;
  private WPI_CANCoder FrontLeftEncoder;
  private WPI_CANCoder BackRightEncoder;
  private WPI_CANCoder BackLeftEncoder;

  private FalconSwerveModule TopLeft;
  private FalconSwerveModule TopRight;
  private FalconSwerveModule BottomLeft;
  private FalconSwerveModule BottomRight;
  private FalconSwerveModule[] modules;

  private Pigeon2 gyro;

  private SwerveDriveKinematics kinematics;

  private HolonomicDrive swerveDrive;
  private SwerveUtils swerveUtils;

  private boolean field;

  public Swervesubsystem() {
      FrontLeft = new WPI_TalonFX(DRIVETRAIN.FL_DRIVE_ID);
      FrontRight = new WPI_TalonFX(DRIVETRAIN.FR_DRIVE_ID);
      BackLeft = new WPI_TalonFX(DRIVETRAIN.BL_DRIVE_ID);
      BackRight = new WPI_TalonFX(DRIVETRAIN.BR_DRIVE_ID);

      rFrontLeft = new WPI_TalonFX(DRIVETRAIN.FL_AZIMUTH_ID);
      rFrontRight = new WPI_TalonFX(DRIVETRAIN.FR_AZIMUTH_ID);
      rBackLeft = new WPI_TalonFX(DRIVETRAIN.BL_AZIMUTH_ID);
      rBackRight = new WPI_TalonFX(DRIVETRAIN.BR_AZIMUTH_ID);

      FrontLeftEncoder = new WPI_CANCoder(DRIVETRAIN.FL_CANCODER_ID);
      FrontRightEncoder = new WPI_CANCoder(DRIVETRAIN.FR_CANCODER_ID);
      BackLeftEncoder = new WPI_CANCoder(DRIVETRAIN.BL_CANCODER_ID);
      BackRightEncoder = new WPI_CANCoder(DRIVETRAIN.BR_CANCODER_ID);

      gyro = new Pigeon2(0);
      gyro.setYaw(0);

      TopLeft = new FalconSwerveModule(FrontLeft, rFrontLeft, 
                FrontLeftEncoder, DRIVETRAIN.FL_ECODER_OFFSET);
      TopRight = new FalconSwerveModule(FrontRight, rFrontRight, 
                  FrontRightEncoder ,DRIVETRAIN.FR_ECODER_OFFSET);
      BottomLeft = new FalconSwerveModule(BackLeft, rBackLeft, 
                  BackLeftEncoder, DRIVETRAIN.BL_ECODER_OFFSET);
      BottomRight = new FalconSwerveModule(BackRight, rBackRight, 
                  BackRightEncoder, DRIVETRAIN.BR_ECODER_OFFSET);

      kinematics = SwerveUtils.createSquareKinematics(DRIVETRAIN.ROBOT_WIDTH);

      modules = new FalconSwerveModule[] {TopLeft, TopRight, BottomLeft, BottomRight};

      swerveDrive = new HolonomicDrive(
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

  public void xLock() {
    
  }

  public Command getAuton() {
    return swerveUtils.followPath("Holonomic path", new HashMap<String,Command>(), false, this);
  }

  public Command toggleField() {
    return new InstantCommand(() -> field = !field);
  }

  public BooleanSupplier getField() {
    return () -> field;
  }

  @Override
  public void periodic() {
    swerveUtils.updateOdometry();

    for(int i = 0; i <= modules.length - 1; i++) {
      SmartDashboard.putNumber("Module " + i + " Degrees", modules[i].getAngleRads().getDegrees());
      SmartDashboard.putNumber("Module " + i + " Meters", modules[i].getDriveMeters());
      SmartDashboard.putNumber("Module " + i + " Velocity", modules[i].getDriveVelocity());
    }

    SmartDashboard.putBoolean("Field", field);
  }

  @Override
  public void simulationPeriodic() {}
}