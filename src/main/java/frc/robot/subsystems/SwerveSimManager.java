package frc.robot.subsystems;
import java.util.List;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Libs.Conversions;
import frc.robot.subsystems.SwerveSim.QuadSwerveSim;
import frc.robot.subsystems.SwerveSim.SwerveModuleSim;

public class SwerveSimManager {
    SwerveModuleSim m_frontLeftSim;;
    SwerveModuleSim m_frontRightSim;
    SwerveModuleSim m_backLeftSim;
    SwerveModuleSim m_backRightSim;

    List<SwerveModuleSim> m_modules;

    QuadSwerveSim m_swerveSim;

    List<DoubleSupplier> FL;
    List<DoubleSupplier> FR;
    List<DoubleSupplier> BL;
    List<DoubleSupplier> BR;

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
    
    public SwerveSimManager(List<DoubleSupplier> FL, List<DoubleSupplier> FR, List<DoubleSupplier> BL, List<DoubleSupplier> BR) {
        m_frontLeftSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            1.0, // treadStaticCoefFric
            1.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            DRIVETRAIN.AZIMUTH_MOI // azimuthEffectiveMOI
        );

        m_frontRightSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            1.0, // treadStaticCoefFric
            1.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            DRIVETRAIN.AZIMUTH_MOI // azimuthEffectiveMOI
        );

        m_backLeftSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            1.0, // wheelEncGearRatio
            1.0, // treadStaticCoefFric
            1.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            DRIVETRAIN.AZIMUTH_MOI // azimuthEffectiveMOI
        );

        m_backRightSim = new SwerveModuleSim(
            DCMotor.getFalcon500(1),
            DCMotor.getFalcon500(1),
            DRIVETRAIN.WHEEL_DIAMETER / 2.0, // wheelRadiusM
            DRIVETRAIN.AZIMUTH_GEAR_RATIO, // azimuthGearRatio
            DRIVETRAIN.DRIVE_GEAR_RATIO, // wheelGearRatio
            1.0, // azimuthEncGearRatio
            Conversions.falconToMPS(1, DRIVETRAIN.WHEEL_PERIMETER, 6.75), // wheelEncGearRatio
            1.0, // treadStaticCoefFric
            1.0, // treadKineticCoefFric
            1.0, // moduleNormalForce
            DRIVETRAIN.AZIMUTH_MOI // azimuthEffectiveMOI
        );
        
        m_modules = List.of(m_frontLeftSim, m_frontRightSim, m_backLeftSim, m_backRightSim);

        m_swerveSim = new QuadSwerveSim(
            DRIVETRAIN.ROBOT_WIDTH,
            DRIVETRAIN.ROBOT_WIDTH,
            125.0,
            DRIVETRAIN.DRIVE_MOI,
            m_modules);

        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;
    }

    public List<SwerveModuleSim> getModules() {
        return m_modules;
    }

    public void update(double dt) {
        m_frontLeftSim.setInputVoltages(FL.get(0).getAsDouble(), FL.get(1).getAsDouble());
        m_frontRightSim.setInputVoltages(FR.get(0).getAsDouble(), FR.get(1).getAsDouble());
        m_backLeftSim.setInputVoltages(BL.get(0).getAsDouble(), BL.get(1).getAsDouble());
        m_backRightSim.setInputVoltages(BR.get(0).getAsDouble(), BR.get(1).getAsDouble());

        m_swerveSim.update(dt);
    }

    public double getDriveEncoderPos(int module) {
        return m_modules.get(module).getWheelEncoderPositionRev();
    }

    public double getAzimuthEncoderPos(int module) {
        return m_modules.get(module).getAzimuthEncoderPositionRev();
    }

    public QuadSwerveSim getSim() {
        return m_swerveSim;
    }

    public void telemetry() {
    }
}