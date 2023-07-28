package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Libs.SwerveUtils;
import frc.robot.subsystems.SwerveSim.QuadSwerveSim;
import frc.robot.subsystems.SwerveSim.SwerveModuleSim;

public class SwerveSimManager {
    SwerveModuleSim m_frontLeftSim;;
    SwerveModuleSim m_frontRightSim;
    SwerveModuleSim m_backLeftSim;
    SwerveModuleSim m_backRightSim;

    List<SwerveModuleSim> m_modules;

    QuadSwerveSim m_swerveSim;

    /*
        DCMotor azimuthMotor,
        DCMotor wheelMotor,
        double wheelRadiusM,
        double azimuthGearRatio,    
        double wheelGearRatio,      
        double azimuthEncGearRatio, 
        double wheelEncGearRatio,   
        double treadStaticCoefFric,
        double treadKineticCoefFric,
        double moduleNormalForce,
        double azimuthEffectiveMOI
    */
    
    public SwerveSimManager() {
        m_frontLeftSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            0.0, // treadStaticCoefFric
            0.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            0.0 // azimuthEffectiveMOI
        );

        m_frontRightSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            0.0, // treadStaticCoefFric
            0.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            0.0 // azimuthEffectiveMOI
        );

        m_backLeftSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            0.0, // treadStaticCoefFric
            0.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            0.0 // azimuthEffectiveMOI
        );

        m_backRightSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            0.0, // treadStaticCoefFric
            0.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            0.0 // azimuthEffectiveMOI
        );
        
        m_modules = List.of(m_frontLeftSim, m_frontRightSim, m_backLeftSim, m_backRightSim);

        m_swerveSim = new QuadSwerveSim(
            DRIVETRAIN.ROBOT_WIDTH,
            DRIVETRAIN.ROBOT_WIDTH,
            125.0,
            0.0,
            m_modules);

    }


}