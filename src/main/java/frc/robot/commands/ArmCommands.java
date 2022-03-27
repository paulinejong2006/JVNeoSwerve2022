package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommands extends CommandBase {
    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_armSubsystem;
    private final XboxController m_controller;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmCommands(XboxController controller, ArmSubsystem armSubsystem) {
        this.m_armSubsystem = armSubsystem;
        this.m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_armSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //init
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //this is to get the values of the L2 and R2 triggers for the climb
        double LStick = m_controller.getLeftY();
        double RStick = m_controller.getRightY();

        if (Math.abs(LStick) < 0.05){
            LStick = 0;
        } else if(Math.abs(LStick) > 0.8) {
            LStick = 0.8;
        }
        m_armSubsystem.leftPercentMotor(LStick);

        if (Math.abs(RStick) < 0.05){
            RStick = 0;
        } else if (Math.abs(RStick) > 0.8){
            RStick = 0.8;
        }
        m_armSubsystem.rightPercentMotor(RStick);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //ends
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  