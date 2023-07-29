package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private double m_lastTime = 0;
  private Field2d field = new Field2d();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    Command auton = m_robotContainer.getAutonCommand();

    if(auton != null) {auton.schedule();};
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    m_lastTime = 0;
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.getSwerveSimManager().update(0.02);
    m_lastTime += 0.02;

    Pose2d robotPose = m_robotContainer.getSwerveSimManager().getSim().getCurPose();

    SmartDashboard.putNumber("x", robotPose.getX());
    SmartDashboard.putNumber("y", robotPose.getY());
    SmartDashboard.putNumber("heading", robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Last time", m_lastTime);
    field.setRobotPose(robotPose);
  }
}
