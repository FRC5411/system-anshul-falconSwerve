package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.Swervesubsystem;

public class RobotContainer {
  private Swervesubsystem m_SwerveSubsystem;
  private CommandXboxController m_controller;

  public RobotContainer() {
    m_controller = new CommandXboxController(0);

    m_SwerveSubsystem = new Swervesubsystem();

    m_SwerveSubsystem.setDefaultCommand(new SwerveCommand(
      () -> -m_controller.getLeftY() * DRIVETRAIN.MAX_LINEAR_SPEED, 
      () -> m_controller.getLeftX() * DRIVETRAIN.MAX_LINEAR_SPEED, 
      () -> m_controller.getRightX() * DRIVETRAIN.MAX_ROTATION_SPEED, 
      m_SwerveSubsystem
      ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_controller.a().onTrue(m_SwerveSubsystem.toggleField());
    m_controller.b().onTrue(new InstantCommand(() -> m_SwerveSubsystem.resetGyro()));
    m_controller.x().onTrue(new InstantCommand(() -> m_SwerveSubsystem.xLock(), m_SwerveSubsystem));
  }

  // This is only a swerve configuration, which is why we are not using an auton subsystem to manage the auton commands
  public Command getAutonCommand() {
    return m_SwerveSubsystem.getAuton();
  }

  public static DriverStation.Alliance getDriverAlliance() {
    // What to do for competition
    //return DriverStation.getAlliance();

    // What to do for testing
    return DriverStation.Alliance.Red;
  }
}