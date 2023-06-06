package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
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
      () -> deadzone(-m_controller.getLeftY()) * DRIVETRAIN.MAX_LINEAR_SPEED, 
      () -> deadzone(m_controller.getLeftX()) * DRIVETRAIN.MAX_LINEAR_SPEED, 
      () -> deadzone(m_controller.getRightX()) * DRIVETRAIN.MAX_ROTATION_SPEED, 
      m_SwerveSubsystem
      ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_controller.a().onTrue(m_SwerveSubsystem.toggleField());
  }

  public Command getAutonCommand() {
    return m_SwerveSubsystem.getAuton();
  }

  public double deadzone(double x) {
    if(Math.abs(x) <= 0.1) {
       return 0;
    }
    return x;
  }
}