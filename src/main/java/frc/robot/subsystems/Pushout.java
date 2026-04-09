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

import edu.wpi.first.math.filter.Debouncer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PushoutConstants;
import frc.robot.Configs;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Pushout extends SubsystemBase {

    // AdvantageKit logging
    private double desiredPercent = 0.0;

    private SparkFlex PushoutMotor = new SparkFlex(PushoutConstants.PUSHOUT_ID, MotorType.kBrushless);
    private SparkClosedLoopController PushoutController = PushoutMotor.getClosedLoopController();

    private final Angle hardLowerLimit = Degrees.of(0);

    // private SparkFlex PushoutRightMotor = new
    // SparkFlex(PushoutConstants.PUSHOUT_RIGHT_ID, MotorType.kBrushless);
    // private SparkClosedLoopController PushoutRightController =
    // PushoutRightMotor.getClosedLoopController();

    private RelativeEncoder pushoutEncoder = PushoutMotor.getEncoder();

    double minVelocity = 1000.0;
    // private final RelativeEncoder pushoutRightEncoder =
    // PushoutRightMotor.getEncoder();

    public Pushout() {
        PushoutMotor.configure(Configs.PushoutSubsystem.PushoutMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        pushoutEncoder.setPosition(0);
    }

    public void PushIntake() {
        PushoutController.setSetpoint(PushoutConstants.PUSHOUT_EXTENDED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void RetractIntake() {
        PushoutController.setSetpoint(PushoutConstants.PUSHOUT_RETRACTED_POS, ControlType.kMAXMotionPositionControl);
    }

    public void StopPushout() {
        PushoutMotor.set(0);
    }

    public Command HomingCommand(double threshold) {
        Debouncer currentDebouncer = new Debouncer(0.2);

        return new RunCommand(() -> PushoutController.setSetpoint(minVelocity, ControlType.kCurrent), this)
                .until(() -> currentDebouncer.calculate((PushoutMotor.getEncoder().getVelocity() >= threshold)))
                .finallyDo(() -> {
                    StopPushout();
                });
    }

    public Command PushCommand() {
        return this.runOnce(() -> PushIntake());
        // .finallyDo(interrupted -> StopPushout())

    }

    public Command RetractCommand() {
        return this.runOnce(() -> RetractIntake());
        // .finallyDo(interrupted -> StopPushout())

    }

    public Command AgitateCommand() {
        final double[] pushPositions = { 11, 9, 7 }; // each time it pushes less far in
        final double[] pullPositions = { 8, 6, 4 }; // each time it pulls further out
        final double finalPos = 1; // pull to this position and idle there after agitation done
        final double waitTime = PushoutConstants.PUSHOUT_AGITATE_WAIT;
        final double betweenAgitateCycles = PushoutConstants.PUSHOUT_AGITATE_BETWEEN_CYCLES_WAIT;

        return Commands.sequence(
                // push to 11 & pull to 8
                runOnce(() -> PushoutController.setSetpoint(pushPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutController.setSetpoint(pullPositions[0], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(betweenAgitateCycles),

                // push to 9 & pull to 6
                runOnce(() -> PushoutController.setSetpoint(pushPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutController.setSetpoint(pullPositions[1], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(betweenAgitateCycles),

                // push to 7 & pull to 4
                runOnce(() -> PushoutController.setSetpoint(pushPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),
                runOnce(() -> PushoutController.setSetpoint(pullPositions[2], ControlType.kMAXMotionPositionControl)),
                Commands.waitSeconds(waitTime),

                Commands.waitSeconds(betweenAgitateCycles),

                // end pos
                runOnce(() -> PushoutController.setSetpoint(finalPos, ControlType.kMAXMotionPositionControl)),
                Commands.idle(this)

        ).finallyDo(interrupted -> PushIntake());
    }

    @Override
    public void periodic() {
        // AdvantageKit Logging
        // Commanded intake motor percent output.
        Logger.recordOutput("Pushout/DesiredPercent", desiredPercent);
        // Applied voltage to intake motor.
        // Logger.recordOutput("Pushout/AppliedVolts",
        // PushoutLeftMotor.getAppliedOutput() * PushoutLeftMotor.getBusVoltage());
    }
}
