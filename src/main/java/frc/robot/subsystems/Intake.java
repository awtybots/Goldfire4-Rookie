package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex IntakeMotor = new SparkFlex(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    private SparkClosedLoopController intakeController = IntakeMotor.getClosedLoopController();
  

    public Intake() {
        IntakeMotor.configure(Configs.IntakeSubsystem.IntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        // IntakeRightMotor.configure(Configs.IntakeSubsystem.IntakeRightMotorConfig,
        // ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
    }

    public void runOuttake() {
        // IntakeRightMotor.set(IntakeConstants.INTAKE_SPEED);
        // desiredPercent = IntakeConstants.OUTTAKE_SPEED;
        // IntakeMotor.set(IntakeConstants.OUTTAKE_SPEED);
        intakeController.setSetpoint(IntakeConstants.OUTTAKE_RPM, ControlType.kMAXMotionVelocityControl);

    }

    public void runIntake() {
        // desiredPercent = IntakeConstants.INTAKE_SPEED;
        // IntakeMotor.set(IntakeConstants.INTAKE_SPEED);

        // IntakeRightMotor.set(IntakeConstants.INTAKE_SPEED);
         intakeController.setSetpoint(IntakeConstants.INTAKE_RPM, ControlType.kMAXMotionVelocityControl);
    }

    public void stopIntake() {
        desiredPercent = 0.0;
        IntakeMotor.set(0);
        // IntakeRightMotor.set(0);
    }

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return new RunCommand(() -> runOuttake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command stopIntakeCommand() {
        return new RunCommand(() -> stopIntake(), this);
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Intake/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        Logger.recordOutput("Intake/AppliedVolts", IntakeMotor.getAppliedOutput() * IntakeMotor.getBusVoltage());
    }
}
