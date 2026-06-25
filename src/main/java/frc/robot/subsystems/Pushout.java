package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

// import edu.wpi.first.math.filter.Debouncer;

// import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PushoutConstants;
import frc.robot.Configs;

// import static edu.wpi.first.units.Units.Amp;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex PushoutMotor = new SparkFlex(PushoutConstants.PUSHOUT_LEFT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutLeftController = PushoutMotor.getClosedLoopController();

    private SparkFlex PushoutMotor2 = new SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutRightController = PushoutMotor2.getClosedLoopController();


    // private SparkFlex PushoutRightMotor = new
    // SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutRightController =
    // PushoutRightMotor.getClosedLoopController();

    private RelativeEncoder pushoutEncoderLeft = PushoutMotor.getEncoder();
    private RelativeEncoder pushoutEncoderRight = PushoutMotor2.getEncoder();

    double minVelocity = -350.0;
    // private final RelativeEncoder pushoutRightEncoder =
    // PushoutRightMotor.getEncoder();

    public Pushout() {
        PushoutMotor.configure(Configs.PushoutSubsystem.PushoutMotorLeftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        pushoutEncoderLeft.setPosition(0);
        PushoutMotor2.configure(Configs.PushoutSubsystem.PushoutMotorRightConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        pushoutEncoderRight.setPosition(0);
    }

    public void PushIntake() {
        PushoutLeftController.setSetpoint(PushoutConstants.PUSHOUT_EXTENDED_POS, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutConstants.PUSHOUT_RIGHT_EXTENDED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void RetractIntake() {
        PushoutLeftController.setSetpoint(PushoutConstants.PUSHOUT_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutConstants.PUSHOUT_RIGHT_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void FullyRetract() {
        PushoutLeftController.setSetpoint(PushoutConstants.FULLY_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
        PushoutRightController.setSetpoint(PushoutConstants.FULLY_RETRACTED_RIGHT_POS, ControlType.kMAXMotionPositionControl);
    }

    public void ResetEncoder() {
        pushoutEncoderLeft.setPosition(0);
        pushoutEncoderRight.setPosition(0);
    }

    public void StopPushout() {
        PushoutMotor.set(0);
        PushoutMotor2.set(0);
    }

    public void PushoutDutycyle() {
        PushoutMotor.set(0.8);
        PushoutMotor2.set(-0.8);
    }

    public void PushoutDutycyleRetract() {
        PushoutMotor.set(-0.8);
        PushoutMotor2.set(0.8);
    }

    public void CheeksyAgitation() {

        PushoutLeftController.setSetpoint(minVelocity, ControlType.kMAXMotionVelocityControl);
        PushoutRightController.setSetpoint(-minVelocity, ControlType.kMAXMotionVelocityControl);
    }

    public Command CheeksyAgitationCommand() {
        return this.run(() -> CheeksyAgitation())
                .finallyDo(() -> {
                    StopPushout();
                });
    }

    public Command CheesyAgitationCommand()
    {
        return this.run(() -> Commands.waitSeconds(1).andThen(() -> RetractIntake())).finallyDo(() -> StopPushout());
    }
    
    public Command HomingCommand(double threshold) {
        // Debouncer currentDebouncer = new Debouncer(0.2);

        return new RunCommand(() -> PushoutLeftController.setSetpoint(minVelocity, ControlType.kMAXMotionVelocityControl),
                this)
                .until(() -> (PushoutMotor.getEncoder().getVelocity() >= threshold))
                .finallyDo(() -> {
                    StopPushout();
                });
    }

    public Command PushoutDutycyleCommand() {
        return this.run(() -> PushoutDutycyle())
                .finallyDo(interrupted -> StopPushout());
    }

    public Command PushoutDutycyleRetractCommand() {
        return this.run(() -> PushoutDutycyleRetract())
                .finallyDo(interrupted -> StopPushout());

    }

    public Command PushCommand() {
        return this.run(() -> PushIntake())
                .finallyDo(interrupted -> StopPushout());

    }

    public Command ResetEncoderCommand() {
        return this.runOnce(() -> ResetEncoder());
    }

    public Command RetractCommand() {
        return this.runOnce(() -> RetractIntake());

    }

    public Command FullyRetractCommand() {
        return this.runOnce(() -> FullyRetract());
    }

    public Command AgitateCommand() {
        final double[] pullPositions = { 12.5, 10, 7, 5, 3 }; // each time it pushes less far in
        final double[] pushPositions = { 15, 13.5, 10, 8.5, 6 }; // each time it pulls further out
        final double finalPos = 4; // pull to this position and idle there after agitation done
        final double waitTime = PushoutConstants.PUSHOUT_AGITATE_WAIT;
        final double waitBetween = PushoutConstants.PUSHOUT_BETWEEN;
        Command agitate = Commands.sequence(
                // push to 11 & pull to 8
                runOnce(() -> PushoutLeftController.setSetpoint(pullPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutLeftController.setSetpoint(pushPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // push to 9 & pull to 6
                runOnce(() -> PushoutLeftController.setSetpoint(pullPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutLeftController.setSetpoint(pushPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // push to 7 & pull to 4
                runOnce(() -> PushoutLeftController.setSetpoint(pullPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutLeftController.setSetpoint(pushPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                runOnce(() -> PushoutLeftController.setSetpoint(pullPositions[3], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutLeftController.setSetpoint(pushPositions[3], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                runOnce(() -> PushoutLeftController.setSetpoint(pullPositions[4], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutLeftController.setSetpoint(pushPositions[4], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(waitBetween),

                // end pos
                runOnce(() -> PushoutLeftController.setSetpoint(finalPos, ControlType.kMAXMotionPositionControl)),
                Commands.idle(this)

        ).finallyDo(interrupted -> PushIntake());
        agitate.addRequirements(this);
        return agitate;
    }

    public Command runDefaultCommand() {
        return new RunCommand(() -> StopPushout(), this);
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
        Logger.recordOutput("Pushout/EncoderPosition", pushoutEncoderLeft.getPosition());

        // Applied voltage to intake motor.
        // Logger.recordOutput("Pushout/AppliedVolts",
        // PushoutLeftMotor.getAppliedOutput() * PushoutLeftMotor.getBusVoltage());
    }
}
