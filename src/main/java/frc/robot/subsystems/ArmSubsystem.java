package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private static CANSparkMax leftArmMotor;
    private static CANSparkMax rightArmMotor;

    public ArmSubsystem() {
        leftArmMotor = new CANSparkMax(Constants.ArmLeftMotor, null);
        rightArmMotor = new CANSparkMax(Constants.ArmRightMotor, null);
    }

    public void leftPercentMotor(double lStick) {
        leftArmMotor.set(lStick);
    } public void rightPercentMotor(double rStick) {
        rightArmMotor.set(rStick);
    }
}
