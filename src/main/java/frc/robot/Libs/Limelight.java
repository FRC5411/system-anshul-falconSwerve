package frc.robot.Libs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Test code to change between pipelines on limelight
public class Limelight {
  private NetworkTable limelight;
  private boolean pipelineIndex;
  private double[] posevalues;
  private Pose2d offsetPose;

  public Limelight(String name, Pose2d offsetPose) {
    limelight = NetworkTableInstance.getDefault().getTable(name);
    pipelineIndex = false;
    this.offsetPose = offsetPose;
  }

  public void switchPipeline() {
    limelight.getEntry("pipeline").setNumber(!pipelineIndex ? 1 : 0);
  }

  public int getPipeLineIndex() {
    return pipelineIndex ? 1 : 0;
  }

  public boolean hastarget() {
    double bool = limelight.getEntry("tv").getDouble(0);
    if(bool == 0) {
      return false;
    }
    return true;
  }

  public double getyaw() {
    return limelight.getEntry("tx").getDouble(0);
  }

  public double getPitch() {
    return limelight.getEntry("ty").getDouble(0);
  }

  public double getArea() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public Pose2d getPose() {
    posevalues = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    Translation2d translate = new Translation2d(posevalues[0] - offsetPose.getX(), posevalues[1] - offsetPose.getY());
    Rotation2d rotation = new Rotation2d(Math.toRadians(posevalues[3]) - offsetPose.getRotation().getRadians());
    
    return new Pose2d(translate, rotation);
  }

  public Pose2d getTarget() {
    posevalues = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    Translation2d translate = new Translation2d(posevalues[0], posevalues[1]);
    Rotation2d rotation = new Rotation2d(Math.toRadians(posevalues[3]));
    return new Pose2d(translate, rotation);
  }

  /** returns latency in seconds (tl + cl) */
  public double getLatency () {
    return (limelight.getEntry("tl").getDouble(0) + limelight.getEntry("cl").getDouble(0))/1000.0;
  }

  public void periodic() {
    SmartDashboard.putNumber("Limelight/2d/yaw", getyaw());
    SmartDashboard.putNumber("Limelight/2d/pitch", getPitch());
    SmartDashboard.putNumber("Limelight/2d/area", getArea());
    SmartDashboard.putNumber("Limelight/pip/pipeline", getPipeLineIndex());
    SmartDashboard.putBoolean("Limelight/hastarget", hastarget());
    SmartDashboard.putNumber("Limelight/Odometry/X", getPose().getX());
    SmartDashboard.putNumber("Limelight/Odometry/Y", getPose().getY());
    SmartDashboard.putNumber("Limelight/Odometry/Rotation", getPose().getRotation().getDegrees());
  }
}