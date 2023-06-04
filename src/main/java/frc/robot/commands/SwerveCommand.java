package frc.robot.commands;
import frc.robot.subsystems.Swervesubsystem;
//import frc.robot.subsystems.sim.Swervesubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Swervesubsystem m_SwerveSubsystem;
  private DoubleSupplier m_x;
  private DoubleSupplier m_y;
  private DoubleSupplier m_z;

  public SwerveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, Swervesubsystem subsystem) {
    m_x = x;
    m_y = y;
    m_z = z;
    m_SwerveSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_SwerveSubsystem.swerveDrive(m_x.getAsDouble(), m_y.getAsDouble(), m_z.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
