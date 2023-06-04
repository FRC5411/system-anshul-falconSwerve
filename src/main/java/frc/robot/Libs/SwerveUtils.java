package frc.robot.Libs;
import java.util.HashMap;
import java.util.List;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveUtils {
    public SwerveDriveKinematics kinematics;
    public SwerveDrivePoseEstimator odometry;
    public SwerveModuleInterface[] modules;
    public SwerveModulePosition[] positions;
    public SwerveAutoBuilder builder;
    public PIDConstants tranPID;
    public PIDConstants rotPID;
    public HashMap <String, Commands> eventMap;
    public Pigeon2 gyro;
    public SwerveDrive drive;

    public SwerveUtils( PIDConstants tranPID, PIDConstants rotPID, SwerveDrive drive) {
        this.modules = drive.getModules();
        this.kinematics = drive.getKinematics();

        this.positions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
        };

        this.tranPID = tranPID;
        this.rotPID = rotPID;

        this.gyro = drive.getGyro();

        this.drive = drive;

        this.odometry = createOdometry();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public SwerveDrivePoseEstimator createOdometry() {
        return new SwerveDrivePoseEstimator(
            kinematics,
            getRotation2d(),
            positions,
            new Pose2d()
        );
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), positions, pose);
    }

    public void updateOdometry(Pose2d vision) {
        odometry.addVisionMeasurement(vision, Timer.getFPGATimestamp());
        updateOdometry();
    }

    public void updateOdometry() {
        int i = 0;
        while(i <= positions.length - 1) {
            double pos = modules[i].getDriveMeters() * modules[i].getAnshulFactor();
            positions[i] = new SwerveModulePosition(pos, modules[i].getAngleRads());
            i++;
        }
        odometry.update(getRotation2d(), positions);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Command followPath(String path, HashMap<String, Command> eventMap, 
                                boolean isRed, Subsystem requirements) {

        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(path, PathPlanner.getConstraintsFromPath(path));

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            this::getPose,
            this::resetOdometry,
            kinematics,
            tranPID,
            rotPID,
            drive::setModuleStates,
            eventMap,
            isRed,
            requirements);

        return builder.fullAuto(trajectory);
    }

}