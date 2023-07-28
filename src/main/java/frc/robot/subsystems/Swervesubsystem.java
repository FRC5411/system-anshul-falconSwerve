package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FalconSwerveModule;
import frc.robot.RobotContainer;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Libs.HolonomicDrive;
import frc.robot.Libs.Limelight;
import frc.robot.Libs.SwerveUtils;

public class Swervesubsystem extends SubsystemBase {
  // Speed Motors
  private WPI_TalonFX RightFront;
  private WPI_TalonFX LeftFront;
  private WPI_TalonFX RightBack;
  private WPI_TalonFX LeftBack;

  private TalonFXSimCollection RightFrontSim;
  private TalonFXSimCollection LeftFrontSim;
  private TalonFXSimCollection RightBackSim;
  private TalonFXSimCollection LeftBackSim;

  // Azimuths
  private WPI_TalonFX rRightFront;
  private WPI_TalonFX rLeftFront;
  private WPI_TalonFX rRightBack;
  private WPI_TalonFX rLeftBack;

  private TalonFXSimCollection rRightFrontSim;
  private TalonFXSimCollection rLeftFrontSim;
  private TalonFXSimCollection rRightBackSim;
  private TalonFXSimCollection rLeftBackSim;
  
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

  private HolonomicDrive swerveDrive;
  private SwerveUtils swerveUtils;

  private boolean field;

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private PIDController tPID;
  private PIDController rPID;

  private Limelight limelight;

  public Swervesubsystem() {
      ///////////////// DEVICE INITIALIZATION \\\\\\\\\\\\\\\\\
      LeftFront = new WPI_TalonFX(DRIVETRAIN.FL_DRIVE_ID, "drivetrain");
      RightFront = new WPI_TalonFX(DRIVETRAIN.FR_DRIVE_ID, "drivetrain");
      LeftBack = new WPI_TalonFX(DRIVETRAIN.BL_DRIVE_ID, "drivetrain");
      RightBack = new WPI_TalonFX(DRIVETRAIN.BR_DRIVE_ID, "drivetrain");

      LeftFrontSim = new TalonFXSimCollection(LeftFront);
      RightFrontSim = new TalonFXSimCollection(RightFront);
      LeftBackSim = new TalonFXSimCollection(LeftBack);
      RightBackSim = new TalonFXSimCollection(RightBack);

      rLeftFront = new WPI_TalonFX(DRIVETRAIN.FL_AZIMUTH_ID, "drivetrain");
      rRightFront = new WPI_TalonFX(DRIVETRAIN.FR_AZIMUTH_ID, "drivetrain");
      rLeftBack = new WPI_TalonFX(DRIVETRAIN.BL_AZIMUTH_ID, "drivetrain");
      rRightBack = new WPI_TalonFX(DRIVETRAIN.BR_AZIMUTH_ID, "drivetrain");

      rLeftFrontSim = new TalonFXSimCollection(rLeftFront);
      rRightFrontSim = new TalonFXSimCollection(rRightFront);
      rLeftBackSim = new TalonFXSimCollection(rLeftBack);
      rRightBackSim = new TalonFXSimCollection(rRightBack);

      RightFrontEncoder = new WPI_CANCoder(DRIVETRAIN.FR_CANCODER_ID, "drivetrain");
      LeftFrontEncoder = new WPI_CANCoder(DRIVETRAIN.FL_CANCODER_ID, "drivetrain");
      RightBackEncoder = new WPI_CANCoder(DRIVETRAIN.BR_CANCODER_ID, "drivetrain");
      LeftBackEncoder = new WPI_CANCoder(DRIVETRAIN.BL_CANCODER_ID, "drivetrain");

      gyro = new Pigeon2(DRIVETRAIN.PIGEON_ID, "drivetrain");
      gyro.setYaw(0);

      limelight = new Limelight("limelight", new Pose2d());

      TopLeft = new FalconSwerveModule(LeftFront, rLeftFront,
                LeftFrontEncoder, DRIVETRAIN.FL_ECODER_OFFSET);
      TopRight = new FalconSwerveModule(RightFront, rRightFront, 
                  RightFrontEncoder ,DRIVETRAIN.FR_ECODER_OFFSET);
      BottomLeft = new FalconSwerveModule(LeftBack, rLeftBack, 
                  LeftBackEncoder, DRIVETRAIN.BL_ECODER_OFFSET);
      BottomRight = new FalconSwerveModule(RightBack, rRightBack, 
                  RightBackEncoder, DRIVETRAIN.BR_ECODER_OFFSET);

      ////// DRIVE INITIZALIZATION \\\\\\\
      kinematics = SwerveUtils.createSquareKinematics(DRIVETRAIN.ROBOT_WIDTH);

      modules = new FalconSwerveModule[] {TopLeft, TopRight, BottomLeft, BottomRight};

      swerveDrive = new HolonomicDrive(
        modules, 
        gyro, 
        kinematics, 
        DRIVETRAIN.MAX_LINEAR_SPEED
      );

      field = false;

      ////// PATH PLANNER INITIALIZATION \\\\\\\
      tPID = new PIDController(DRIVETRAIN._translationKp, DRIVETRAIN._translationKi, DRIVETRAIN._translationKd);
        tPID.setTolerance(0);
        tPID.setIntegratorRange(-0, 0);
    
      rPID = new PIDController(DRIVETRAIN._rotationKp, DRIVETRAIN._rotationKi, DRIVETRAIN._rotationKd);
        rPID.setTolerance(0);
        rPID.setIntegratorRange(-0, 0);

      swerveUtils = new SwerveUtils(
        tPID,
        rPID,
        swerveDrive
      );

      swerveUtils.setVisionVaritation(VecBuilder.fill(0.09, 0.09, Math.toRadians(180)));

      ////// ALIGNMENT WAYPOINT INITIALIZATION \\\\\\\
    if (RobotContainer.getDriverAlliance() == DriverStation.Alliance.Red) {  
      _coneWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(14.83, 5.15, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.83, 3.94, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.83, 3.38, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.83, 2.28, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.83, 1.70, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(14.83, 0.57, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.83, 1.13, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.83, 2.95, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(14.83, 4.52, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(0.76, 6.13, new Rotation2d((Math.PI))));
      _cubeWaypoints.add(new Pose2d(0.76, 7.49, new Rotation2d((Math.PI))));
    } else if (RobotContainer.getDriverAlliance() == DriverStation.Alliance.Blue) {
      _coneWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d(Math.PI)));
      _coneWaypoints.add(new Pose2d(1.84, 5.05, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.84, 3.84, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.84, 3.28, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.84, 2.18, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.84, 1.60, new Rotation2d(0)));
      _coneWaypoints.add(new Pose2d(1.84, 0.47, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.84, 1.03, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.84, 2.75, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(1.84, 4.42, new Rotation2d(0)));
      _cubeWaypoints.add(new Pose2d(15.79, 7.33, new Rotation2d((Math.PI))));
      _cubeWaypoints.add(new Pose2d(15.79, 6.00, new Rotation2d((Math.PI))));
    }
  }

  ////// DRIVE FUNCTIONS \\\\\\\
  public void swerveDrive(double x_speed, double y_speed, double orientation) {
    swerveDrive.drive(new Translation2d(x_speed, y_speed), orientation, field, false);
  }

  public void xLock() {
    swerveDrive.xLock();
  }

  public Command toggleField() {
    return new InstantCommand(() -> field = !field);
  }

  ////// PERIODIC FUNCTIONS\\\\\\\
  @Override
  public void periodic() {
    swerveUtils.updateOdometry();
    if(limelight.hastarget()) {swerveUtils.addVisionMeasurement(limelight.getPose(), 
                               Timer.getFPGATimestamp() - limelight.getLatency());}
    limelight.periodic();

    for(int i = 0; i <= modules.length - 1; i++) {
      modules[i].setTelemetry(i);
    }

    SmartDashboard.putBoolean("Field Oriented", field);
  }

  @Override
  public void simulationPeriodic() {}


  ////// PATH PLANNER FUNCTIONS \\\\\\\
  public Command getAuton() {
    return swerveUtils.followPath("Holonomic path", new HashMap<String,Command>(), true, this);
  }

  public Command PPmoveToPositionCommand (boolean wantCone) {  
    Pose2d closest = getOdometry().nearest(wantCone ? _coneWaypoints : _cubeWaypoints);
    if (closest == null) return new InstantCommand();
    return PPpathToCommand( closest );
  }

  public Command PPpathToCommand (Pose2d target) {
    swerveUtils.resetControllers();

    PathPlannerTrajectory _alignToTarget = PathPlanner.generatePath(
      DRIVETRAIN.alignConstraints,
      new PathPoint(
        getOdometry().getTranslation(), 
        new Rotation2d(Math.toRadians(gyro.getYaw()))),

      new PathPoint(
        new Translation2d( getOdometry().getX(), target.getY()) , 
        target.getRotation()
        )        
    );

    PathPlannerTrajectory _toTarget = PathPlanner.generatePath(
      DRIVETRAIN.alignConstraints,
      new PathPoint( new Translation2d( getOdometry().getX(), target.getY() ), 
      target.getRotation()),

      new PathPoint(
        target.getTranslation(),
        target.getRotation()
      )
    );

    Command align = swerveUtils.followTrajectoryCommand(_alignToTarget, false, this);
    Command toGoal = swerveUtils.followTrajectoryCommand(_toTarget, false, this);

    return new SequentialCommandGroup(align, toGoal);
  }

  ///// ODOMETRY FUNCTIONS \\\\\\
  public void resetOdometry(Pose2d pose) {
    swerveUtils.resetOdometry(pose);
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getOdometry() {
    return swerveUtils.getPose();
  }

  ////// SIMULATOR VALUES \\\\\\\
  public List<DoubleSupplier> getFL() {
    DoubleSupplier driveLF = () -> LeftFrontSim.getMotorOutputLeadVoltage();
    DoubleSupplier azimuthLF = () -> rLeftFrontSim.getMotorOutputLeadVoltage();

    return List.of(driveLF, azimuthLF);
  }

  public List<DoubleSupplier> getFR() {
    DoubleSupplier driveRF = () -> RightFrontSim.getMotorOutputLeadVoltage();
    DoubleSupplier azimuthRF = () -> rRightFrontSim.getMotorOutputLeadVoltage();

    return List.of(driveRF, azimuthRF);
  }

  public List<DoubleSupplier> getBL() {
    DoubleSupplier driveLB = () -> LeftBackSim.getMotorOutputLeadVoltage();
    DoubleSupplier azimuthLB = () -> rLeftBackSim.getMotorOutputLeadVoltage();

    return List.of(driveLB, azimuthLB);
  }

  public List<DoubleSupplier> getBR() {
    DoubleSupplier driveRB = () -> RightBackSim.getMotorOutputLeadVoltage();
    DoubleSupplier azimuthRB = () -> rRightBackSim.getMotorOutputLeadVoltage();

    return List.of(driveRB, azimuthRB);
  }
}