package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FalconSwerveModule;
import frc.robot.RobotContainer;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Libs.HolonomicDrive;
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

  private HolonomicDrive swerveDrive;
  private SwerveUtils swerveUtils;

  private boolean field;

  private List<Pose2d> _coneWaypoints = new ArrayList<Pose2d>();
  private List<Pose2d> _cubeWaypoints = new ArrayList<Pose2d>();

  private PIDController tPID;
  private PIDController rPID;

  public Swervesubsystem() {
      LeftFront = new WPI_TalonFX(11);
      RightFront = new WPI_TalonFX(12);
      LeftBack = new WPI_TalonFX(13);
      RightBack = new WPI_TalonFX(14);

      rLeftFront = new WPI_TalonFX(21);
      rRightFront = new WPI_TalonFX(22);
      rLeftBack = new WPI_TalonFX(23);
      rRightBack = new WPI_TalonFX(24);

      RightFrontEncoder = new WPI_CANCoder(0);
      LeftFrontEncoder = new WPI_CANCoder(1);
      RightBackEncoder = new WPI_CANCoder(2);
      LeftBackEncoder = new WPI_CANCoder(3);

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

      kinematics = SwerveUtils.createSquareKinematics(DRIVETRAIN.ROBOT_WIDTH);

      modules = new FalconSwerveModule[] {TopLeft, TopRight, BottomLeft, BottomRight};

      swerveDrive = new HolonomicDrive(
        modules, 
        gyro, 
        kinematics, 
        DRIVETRAIN.MAX_LINEAR_SPEED
        );

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

      field = false;

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

  public void swerveDrive(double x_speed, double y_speed, double orientation) {
    Translation2d translation = new Translation2d(x_speed, y_speed);

    swerveDrive.drive(translation, orientation, true, false);
  }

  public void xLock() {
    swerveDrive.xLock();
  }

  public Command getAuton() {
    return swerveUtils.followPath("Holonomic path", new HashMap<String,Command>(), true, this);
  }

  public Command toggleField() {
    return new InstantCommand(() -> field = !field);
  }

  @Override
  public void periodic() {
    swerveUtils.updateOdometry();

    for(int i = 0; i <= modules.length - 1; i++) {
      modules[i].setTelemetry(i);
    }

    SmartDashboard.putBoolean("Field Oriented", field);
  }

  @Override
  public void simulationPeriodic() {}

  public void resetOdometry(Pose2d pose) {
    swerveUtils.resetOdometry(pose);
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getOdometry() {
    return swerveUtils.getPose();
  }

  public Command PPmoveToPositionCommand (boolean wantCone) {  
    Pose2d closest = getOdometry().nearest(wantCone ? _coneWaypoints : _cubeWaypoints);
    if (closest == null) return new InstantCommand();
    return PPpathToCommand( closest );
  }

  public Command PPpathToCommand (Pose2d target) {
    swerveUtils.resetControllers();

    PathPlannerTrajectory _alignToTarget = PathPlanner.generatePath(
      new PathConstraints(1, 0.5),
      new PathPoint(new Translation2d(
        getOdometry().getX(), 
        getOdometry().getY()), 
        new Rotation2d(Math.toRadians(gyro.getYaw()))),

      new PathPoint(
        new Translation2d(
          getOdometry().getX(), 
          target.getY()), 
          target.getRotation()
        )        
    );

    PathPlannerTrajectory _toTarget = PathPlanner.generatePath(
      new PathConstraints(1, 0.5),
      new PathPoint(
        new Translation2d(
          getOdometry().getX(), 
          target.getY()), 
          target.getRotation()),

      new PathPoint(
        new Translation2d(
          target.getX(), 
          target.getY()), 
          target.getRotation()
        )
    );

    Command align = swerveUtils.followTrajectoryCommand(_alignToTarget, false, this);
    Command toGoal = swerveUtils.followTrajectoryCommand(_toTarget, false, this);

    return new SequentialCommandGroup(align, toGoal);
  }
}