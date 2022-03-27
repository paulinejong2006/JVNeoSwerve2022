package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private static CANSparkMax leftMotor;
    private static CANSparkMax rightMotor;

    public ElevatorSubsystem() {
        leftMotor = new CANSparkMax(Constants.ElevatorLeftMotor, null);
        rightMotor = new CANSparkMax(Constants.ElevatorRightMotor, null);
      } 
    
      public void percentMotor(double input){
    
        leftMotor.set(input);
        rightMotor.follow(leftMotor, true);
      }
      
      @Override
      public void periodic() {
      // This method will be called once per scheduler run
      }
}
