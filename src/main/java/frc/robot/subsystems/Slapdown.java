package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import frc.robot.Configs;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants.SlapdownConstants;

public class Slapdown extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex SlapdownMotor = new SparkFlex(SlapdownConstants.SLAPDOWN_ID, MotorType.kBrushless);
    private SparkClosedLoopController slapdownController = SlapdownMotor.getClosedLoopController();

    private final RelativeEncoder slapdownEncoder = SlapdownMotor.getEncoder();


    public Slapdown() {
        SlapdownMotor.configure(Configs.SlapdownSubsystem.SlapdownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setHoodPosition(double position) {
        slapdownController.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
    }
    
    public void retract() {
        slapdownController.setSetpoint(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
    }

    public void slowretract() {

        slapdownController.setSetpoint(0,  ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void extend() {

        slapdownController.setSetpoint(25, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
    }

    public Command setSlapdownPositionCommand(double position) {
            return new RunCommand(() -> setHoodPosition(position), this);
        }
    public Command retractCommand() {
            return new RunCommand(() -> retract(), this);
        }
    
    public Command slowretractCommand () {
         return new RunCommand(() -> slowretract(), this);
    }

    public Command extendCommand () {
        return new RunCommand(() -> extend(), this);
    }

    public Boolean isSlapdownOut () {
        return slapdownEncoder.getPosition() > 20;
    }
    public Boolean isSlapdownIn () {
        return slapdownEncoder.getPosition() < 5;
    }

    @Override
    public void periodic() {
        // position = slapdownEncoder.getPosition();
        isSlapdownOut();
        isSlapdownIn();
    }
}
