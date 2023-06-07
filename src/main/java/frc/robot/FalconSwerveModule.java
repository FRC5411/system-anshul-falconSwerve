package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Libs.Conversions;
import frc.robot.Libs.SwerveModuleInterface;
import frc.robot.Constants.*;


public class FalconSwerveModule implements SwerveModuleInterface {
    private WPI_TalonFX m_speed;
    private WPI_TalonFX m_rotation;
    private WPI_CANCoder rot_encoder;
    private Rotation2d lastAngle;

    public FalconSwerveModule(WPI_TalonFX speed, WPI_TalonFX rotation, WPI_CANCoder encoder, double offset) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = encoder;

        lastAngle = new Rotation2d();

        SwerveConfigs.configDrive(m_speed);
        SwerveConfigs.configPosition(rot_encoder, offset);
        SwerveConfigs.configAzimuth(m_rotation, rot_encoder);
    }

    @Override
    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngleRads());
        
        setDriveMPS(optimizedState, openLoop);
        setAngleDegrees(optimizedState);
    }

    @Override
    public void setDriveMPS(SwerveModuleState state, boolean openLoop) {
        double speed = state.speedMetersPerSecond;

        if (openLoop) {
            m_speed.set(speed/ 5.4);
        } else {
            m_speed.set(ControlMode.Velocity, 
            Conversions.MPSToFalcon(speed, DRIVETRAIN.WHEEL_PERIMETER, DRIVETRAIN.DRIVE_GEAR_RATIO));
        }
    }

    @Override
    public void setAngleDegrees(SwerveModuleState state) {
        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DRIVETRAIN.MAX_LINEAR_SPEED * 0.01)) ? lastAngle : state.angle;
        double angleDegrees = angle.getDegrees();

        m_rotation.set(ControlMode.Position, 
        (360
        /
        (DRIVETRAIN.DRIVE_GEAR_RATIO * 4096)) 
        *
        angleDegrees
        );

        lastAngle = angle;
    }

    @Override
    public WPI_CANCoder getEncoder() {
        return rot_encoder;
    }

    @Override
    public double getDriveMeters() {
        return 
        DRIVETRAIN.DRIVE_GEAR_RATIO 
        *
        DRIVETRAIN.WHEEL_PERIMETER
        *
        (m_speed.getSelectedSensorPosition()/2048);
    }

    public double getDriveVelocity() {
        return 
        DRIVETRAIN.DRIVE_GEAR_RATIO 
        *
        DRIVETRAIN.WHEEL_PERIMETER
        *
        (m_speed.getSelectedSensorVelocity()/2048);
    }

    @Override
    public double getAnshulFactor() {
        return DRIVETRAIN.SCALE_FACTOR;
    }

    @Override
    public Rotation2d getAngleRads() {
        return 
        new Rotation2d(
            getEncoder().getAbsolutePosition() 
            * 
            (
            (2 * Math.PI) 
            / 
            (DRIVETRAIN.DRIVE_GEAR_RATIO * 4096)
            )
        );
    }

    @Override
    public void resetToAbsolute() {
        rot_encoder.setPosition(0);
    }

    @Override
    public void resetToZero() {
        m_speed.setSelectedSensorPosition(0);
    }
}