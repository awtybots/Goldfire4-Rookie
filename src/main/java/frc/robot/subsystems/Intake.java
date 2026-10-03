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
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Configs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    // AdvantageKit logging
    public SparkFlex IntakeMotor = new SparkFlex(IntakeConstants.INTAKE_ID, MotorType.kBrushless);

    public SparkClosedLoopController IntakeController = IntakeMotor.getClosedLoopController();

    public Intake() {
        IntakeMotor.configure(Configs.IntakeSubsystem.IntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runIntake() { // right motor follows the left motor, so only need to set the left motor speed
      IntakeController.setSetpoint(IntakeConstants.INTAKE_DUTY, ControlType.kDutyCycle);
     }

    public void runOuttake() { // right motor follows the left motor, so only need to set the left motor speed
      IntakeController.setSetpoint(IntakeConstants.OUTTAKE_DUTY,
                ControlType.kDutyCycle);
     }     

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        return new RunCommand(() -> runOuttake(), this)
                .finallyDo(interrupted -> stopIntake());
    }

    public void stopIntake() {
        IntakeMotor.set(0);
    }

    public void runIntakeExample() {
        IntakeController.setSetpoint(1200, ControlType.kDutyCycle);
    }

    public Command runIntakeExampleCommand() {
        return new RunCommand(() -> runIntakeExample(), this).finallyDo(interrupted -> stopIntake());
    }
    
    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        double RPM = IntakeMotor.getEncoder().getVelocity();
        Logger.recordOutput("IntakeRPM", RPM);
        Logger.recordOutput("IntakeTarget", IntakeConstants.INTAKE_DUTY);
    }
}