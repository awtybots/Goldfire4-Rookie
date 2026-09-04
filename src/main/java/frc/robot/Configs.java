package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SlapdownConstants;

import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public final class Configs 
{
        public static final class IntakeSubsystem {
                
            public static final SparkFlexConfig IntakeMotorConfig = new SparkFlexConfig();

                static {

                        IntakeMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true);

                        IntakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(IntakeConstants.p)
                            .i(IntakeConstants.i)
                            .d(IntakeConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(IntakeConstants.s)
                            .kV(IntakeConstants.v)
                            .kA(IntakeConstants.a)
                            ;

                        IntakeMotorConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);
                }

        };
        public static final class SlapdownSubsystem {
                
            public static final SparkFlexConfig SlapdownMotorConfig = new SparkFlexConfig();

                static {

                        SlapdownMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).inverted(true);

                        SlapdownMotorConfig.closedLoop
                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            //slow pid
                            .p(SlapdownConstants.slowP, ClosedLoopSlot.kSlot0)
                            .i(SlapdownConstants.slowI, ClosedLoopSlot.kSlot0)
                            .d(SlapdownConstants.slowD, ClosedLoopSlot.kSlot0)
                            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                            
                            //fast
                            .p(SlapdownConstants.fastP, ClosedLoopSlot.kSlot1)
                            .i(SlapdownConstants.fastI, ClosedLoopSlot.kSlot1)
                            .d(SlapdownConstants.fastD, ClosedLoopSlot.kSlot1)
                            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

                        SlapdownMotorConfig.closedLoop.maxMotion
                            .maxAcceleration(1000, ClosedLoopSlot.kSlot0)
                            .cruiseVelocity(1000, ClosedLoopSlot.kSlot0)
                            .allowedProfileError(0.2, ClosedLoopSlot.kSlot0)

                            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
                            .cruiseVelocity(500, ClosedLoopSlot.kSlot1)
                            .allowedProfileError(0.2, ClosedLoopSlot.kSlot1)
                        ;
                        

                }

        };
        public static final class HoodSubsystem {
                
            public static final SparkFlexConfig HoodMotorConfig = new SparkFlexConfig();

                static {

                        HoodMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        HoodMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(HoodConstants.p)
                            .i(HoodConstants.i)
                            .d(HoodConstants.d)
                            .outputRange(-1, 1)
                            ;

                        HoodMotorConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);

                }

        };
        public static final class ShooterSubsystem {
                
            public static final SparkFlexConfig ShooterMotorLeft1Config = new SparkFlexConfig();
            public static final SparkFlexConfig ShooterMotorRight1Config = new SparkFlexConfig();
            public static final SparkFlexConfig ShooterMotorLeft2Config = new SparkFlexConfig();
            public static final SparkFlexConfig ShooterMotorRight2Config = new SparkFlexConfig();

                static {

                        ShooterMotorLeft1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                        ShooterMotorRight1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(ShooterConstants.SHOOTER_L1_ID, true);
                        ShooterMotorLeft2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(ShooterConstants.SHOOTER_L1_ID, true);
                        ShooterMotorRight2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(ShooterConstants.SHOOTER_L1_ID, false);
                        ShooterMotorLeft1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(ShooterConstants.p)
                            .i(ShooterConstants.i)
                            .d(ShooterConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(ShooterConstants.s)
                            .kV(ShooterConstants.v)
                            .kA(ShooterConstants.a)
                            ;
                        
                        ShooterMotorLeft1Config.closedLoop
                        .maxMotion.maxAcceleration(1000000);
                }
            
        };
        public static final class KickerSubsystem {
                
            public static final SparkFlexConfig KickerMotorLeftConfig = new SparkFlexConfig();
            public static final SparkFlexConfig KickerMotorRightConfig = new SparkFlexConfig();

                static {

                        KickerMotorLeftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                        KickerMotorRightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(KickerConstants.KICKER_LEFT_ID, true);
                        KickerMotorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(KickerConstants.p)
                            .i(KickerConstants.i)
                            .d(KickerConstants.d)
                            .outputRange(-1, 1)
                            .feedForward
                            .kS(KickerConstants.s)
                            .kV(KickerConstants.v)
                            .kA(KickerConstants.a)
                            ;

                        KickerMotorLeftConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);
                }

        };
        

}
