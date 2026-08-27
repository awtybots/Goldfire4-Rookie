package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants.ShooterConstants;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import frc.robot.Configs;

public class Shooter extends SubsystemBase {

    // Instantiating the hopper to shooter motor
    private SparkFlex ShooterLeft1Motor = new SparkFlex(ShooterConstants.SHOOTER_L1_ID, MotorType.kBrushless);
    private SparkClosedLoopController shooterleft1Controller = ShooterLeft1Motor.getClosedLoopController();

    private SparkFlex ShooterRight1Motor = new SparkFlex(ShooterConstants.SHOOTER_R1_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterright1Controller = ShooterRight1Motor.getClosedLoopController();

    private SparkFlex ShooterRight2Motor = new SparkFlex(ShooterConstants.SHOOTER_R2_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterright2Controller = ShooterRight2Motor.getClosedLoopController();

    private SparkFlex ShooterLeft2Motor = new SparkFlex(ShooterConstants.SHOOTER_L2_ID, MotorType.kBrushless);
    //private SparkClosedLoopController shooterleft2Controller = ShooterLeft2Motor.getClosedLoopController();


    private final RelativeEncoder shooterLeft1Encoder = ShooterLeft1Motor.getEncoder();


    public Shooter() {
        ShooterRight1Motor.configure(Configs.ShooterSubsystem.ShooterMotorRight1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterLeft1Motor.configure(Configs.ShooterSubsystem.ShooterMotorLeft1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterRight2Motor.configure(Configs.ShooterSubsystem.ShooterMotorRight2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        ShooterLeft2Motor.configure(Configs.ShooterSubsystem.ShooterMotorLeft2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShooterSpeed(double speed) {
        shooterleft1Controller.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    public void stopShooter() {
        shooterleft1Controller.setSetpoint(0, ControlType.kDutyCycle);
    }

    public Command setShooterSpeedCommand(double speed) {
            return new RunCommand(() -> setShooterSpeed(speed), this).finallyDo(interrupted -> stopShooter());
        }

    @Override
    public void periodic() {
    }
}
