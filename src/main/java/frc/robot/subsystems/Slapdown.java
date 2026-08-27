package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        slapdownController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    public Command setHoodPositionCommand(double position) {
            return new RunCommand(() -> setHoodPosition(position), this);
        }

    @Override
    public void periodic() {
    }
}
