package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import frc.robot.Constants.KickerConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Configs.KickerSubsystem;

public class Kicker extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex KickerLeftMotor = new SparkFlex(KickerConstants.KICKER_LEFT_ID, MotorType.kBrushless);
    private SparkFlex KickerRightMotor = new SparkFlex(KickerConstants.KICKER_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController kickerLeftController = KickerLeftMotor.getClosedLoopController();

    // private final RelativeEncoder kickerLeftEncoder = KickerLeftMotor.getEncoder();



    public Kicker() {
        KickerLeftMotor.configure(KickerSubsystem.KickerMotorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        KickerRightMotor.configure(KickerSubsystem.KickerMotorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void Kick() {
        kickerLeftController.setSetpoint(KickerConstants.KICKER_RPM, ControlType.kMAXMotionVelocityControl);
    }

    public Command kickCommand() {
        return new RunCommand(() -> Kick(), this)
            .finallyDo(interrupted -> stopKicking());
    }

    public void stopKicking() {
        kickerLeftController.setSetpoint(0, ControlType.kMAXMotionVelocityControl);
    }

    public void setKickerSpeedExample() {
        kickerLeftController.setSetpoint(1200, ControlType.kMAXMotionVelocityControl);
    }

    public Command setKickerSpeedExampleCommand() {
        return new RunCommand(() -> setKickerSpeedExample(), this).finallyDo(interrupted -> stopKicking());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // double kickerLeftRPM = KickerLeftMotor.getEncoder().getVelocity();
        // Logger.recordOutput("Shooter/KickerRightAppliedVolts",
        //         KickerRightMotor.getAppliedOutput() * KickerRightMotor.getBusVoltage());
    }
}
