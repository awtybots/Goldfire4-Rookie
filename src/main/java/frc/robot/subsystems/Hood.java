package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.Degree;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.HoodConstants;
import frc.robot.Configs;



public class Hood extends SubsystemBase 
{

    private SparkMax HoodMotor = new SparkMax(HoodConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();

    private RelativeEncoder hoodEncoder = HoodMotor.getEncoder();

    public Hood() {
        HoodMotor.configure(Configs.HoodSubsystem.HoodControllerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        hoodEncoder.setPosition(0);
    }

    public double angleToEncoderTicks(double angle)
    {
        // target ticks = (target angle in deg / 360) * gear reduction * counts per revolution
        return (angle / 360.0) * HoodConstants.GEAR_REDUCTION * HoodConstants.COUNTS_PER_REVOLUTION;
    }
    
    public Angle encoderTicksToAngle(double encoderValue)
    {
        return Degree.of(encoderValue / (HoodConstants.COUNTS_PER_REVOLUTION  * HoodConstants.GEAR_REDUCTION) * 360.0);
    }
    
    public void SetHoodPosition(double angle)
    {
        HoodController.setSetpoint(angleToEncoderTicks(angle), ControlType.kMAXMotionPositionControl);
    }

    public Angle getHoodAngle()
    {
        return encoderTicksToAngle(hoodEncoder.getPosition());
    } 

    public Angle getHoodTargetAngle()
    {
        return encoderTicksToAngle(HoodController.getSetpoint());
    } 

    public boolean isHoodAtSetpoint()
    {
        return Math.abs(getHoodError().in(Degree)) <= HoodConstants.HOOD_TOLERANCE;
    }

    public Angle getHoodError()
    {
        return getHoodTargetAngle().minus(getHoodAngle());
    }

    public void holdCurrentHoodPosition()
    {
        SetHoodPosition(getHoodAngle().in(Degree));
    }

    public void StopHood()
    {
        HoodMotor.set(0);
    }

    public Command SetHoodPositionCommand(double angle)
    {
        return this.runOnce(() -> SetHoodPosition(angle));
    }

    @Override
    public void periodic() 
    {
        Logger.recordOutput("Hood/CurrentAngleDeg", getHoodAngle().in(Degree));
        Logger.recordOutput("Hood/CurrentEncoderTarget", getHoodTargetAngle().in(Degree));
        Logger.recordOutput("Hood/CurrentEncoderPosition", hoodEncoder.getPosition());
        Logger.recordOutput("Hood/TargetError", getHoodError().in(Degree));
        Logger.recordOutput("Hood/IsAtSetpoint", isHoodAtSetpoint());
    }
}
