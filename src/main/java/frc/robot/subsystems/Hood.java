package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.HoodConstants;

import java.util.function.BooleanSupplier;

public class Hood extends SubsystemBase {

    private SparkFlex HoodMotor = new SparkFlex(HoodConstants.HOOD_ID, MotorType.kBrushless);
    private SparkClosedLoopController HoodController = HoodMotor.getClosedLoopController();
    private RelativeEncoder HoodEncoder = HoodMotor.getEncoder();

    private double currentTargetRotations = HoodConstants.HOOD_MIN;

    private static final double SIM_MAX_DEG_PER_SEC = 200.0;
    private final SparkFlexSim hoodSim = RobotBase.isSimulation()
            ? new SparkFlexSim(HoodMotor, DCMotor.getNeoVortex(1))
            : null;
    private double simRotations = HoodConstants.HOOD_MIN;

    private final BooleanSupplier nearTrench;

    public Hood(BooleanSupplier nearTrench) {
        this.nearTrench = nearTrench;

        HoodMotor.configure(Configs.HoodSubsystem.HoodMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        HoodEncoder.setPosition(HoodConstants.HOOD_MIN);
    }

    public double getRotations() {
        return HoodEncoder.getPosition();
    }

    public double getTargetRotations() {
        return currentTargetRotations;
    }

    public double getAngleDegrees() {
        return HoodConstants.rotationsToDegrees(getRotations());
    }

    public boolean isTrenchLocked() {
        return nearTrench.getAsBoolean();
    }

    public boolean isAtPosition() {
        return Math.abs(getRotations() - currentTargetRotations) <= HoodConstants.POSITION_TOLERANCE_ROTATIONS;
    }

    public void setHoodPosition(double rotations) {
        double clamped = isTrenchLocked() ? HoodConstants.HOOD_MIN : HoodConstants.clampRotations(rotations);
        currentTargetRotations = clamped;
        HoodController.setSetpoint(clamped, ControlType.kPosition);
    }

    public void setAngle(double degrees) {
        setHoodPosition(HoodConstants.degreesToRotations(degrees));
    }

    public void lowerHood() {
        setHoodPosition(HoodConstants.HOOD_MIN);
    }

    public void raiseHood() {
        setHoodPosition(HoodConstants.HOOD_MAX);
    }

    public void stopHood() {
        if (isTrenchLocked()) {
            lowerHood();
            return;
        }
        HoodMotor.set(0);
    }

    public Command setHoodPositionCommand(double rotations) {
        return this.run(() -> {
            setHoodPosition(rotations);
        }).finallyDo(interrupted -> stopHood());
    }

    public Command lowerHoodCommand() {
        return this.run(() -> {
            lowerHood();
        }).finallyDo(interrupted -> stopHood());
    }

    public Command raiseHoodCommand() {
        return this.run(() -> {
            raiseHood();
        }).finallyDo(interrupted -> stopHood());
    }

    public Command tuckCommand() {
        return this.run(() -> {
            lowerHood();
        });
    }

    @Override
    public void periodic() {
        if (isTrenchLocked()) {
            lowerHood();
        }

        Logger.recordOutput("Hood/TrenchLocked", isTrenchLocked());
        Logger.recordOutput("Hood/Rotations", getRotations());
        Logger.recordOutput("Hood/TargetRotations", currentTargetRotations);
        Logger.recordOutput("Hood/AngleDegrees", getAngleDegrees());
        Logger.recordOutput("Hood/IsAtPosition", isAtPosition());
        Logger.recordOutput("Hood/AppliedVolts", HoodMotor.getAppliedOutput() * HoodMotor.getBusVoltage());
        Logger.recordOutput("Hood/StatorCurrent", HoodMotor.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {
        double step = SIM_MAX_DEG_PER_SEC / HoodConstants.DEGREES_PER_ROTATION * 0.020;
        simRotations += MathUtil.clamp(currentTargetRotations - simRotations, -step, step);
        simRotations = HoodConstants.clampRotations(simRotations);
        hoodSim.getRelativeEncoderSim().setPosition(simRotations);
    }
}
