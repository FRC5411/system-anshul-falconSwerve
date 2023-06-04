package frc.robot.Libs;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
    private Pigeon2 gyro;
    private SwerveModuleInterface modules[];
    private SwerveDriveKinematics kinematics;
    private double maxSpeed;
    private boolean invertGyro;

    public SwerveDrive(SwerveModuleInterface[] modules, Pigeon2 gyro,
    SwerveDriveKinematics kinematics, double maxSpeed) {
        invertGyro = false;
        this.gyro = gyro;

        this.kinematics = kinematics;

        this.modules = modules;

        this.maxSpeed = maxSpeed;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for(int i = 0; i <= modules.length - 1; i++) {
            SmartDashboard.putNumber("Module " + i + " Angle", swerveModuleStates[i].angle.getDegrees());
            SmartDashboard.putNumber("Module " + i + " Speed", swerveModuleStates[i].speedMetersPerSecond);
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for(int i = 0; i < modules.length - 1; i++) {
            modules[i].setDesiredState(desiredStates[i], false);
        }
    }

    public void invertGyro(boolean invert) {
        gyro.setYaw(-gyro.getYaw());
    }

    public Rotation2d getYaw() {
        return invertGyro ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    public SwerveModuleInterface[] getModules() {
        return modules;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}