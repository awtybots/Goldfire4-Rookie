package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HoodConstants;
import frc.robot.Configs;



public class Hood extends SubsystemBase 
{

    private SparkFlex HoodMotor = new SparkFlex(HoodConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();

    private RelativeEncoder hoodEncoder = HoodMotor.getEncoder();

    public Hood() {
        HoodMotor.configure(Configs.PushoutSubsystem.PushoutMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        hoodEncoder.setPosition(0);
    }

    public double angleToEncoderTicks(double angle)
    {
        // target ticks = (target angle in deg / 360) * gear reduction * counts per revolution
        return (angle / 360) * HoodConstants.GEAR_REDUCTION * HoodConstants.COUNTS_PER_REVOLUTION;
    }
    
    public void SetHoodPosition(double angle)
    {
        HoodController.setSetpoint(angleToEncoderTicks(angle), ControlType.kMAXMotionPositionControl);
    }

    public void StopHood()
    {
        HoodMotor.set(0);
    }

    public Command SetHoodPositionCommand(double angle)
    {
        return this.run(() -> SetHoodPosition(angle))
                        .finallyDo(interrupted -> StopHood());
    }

    @Override
    public void periodic() {

    }
}
