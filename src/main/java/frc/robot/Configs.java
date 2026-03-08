package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.*;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public final class Configs 
{

        public static final class IntakeSubsystem {
                
            public static final SparkFlexConfig IntakeMotorConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig IntakeRightMotorConfig = new SparkFlexConfig();

                static {

                        IntakeMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                        // IntakeRightMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);



                        IntakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.0002355)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1);
                        IntakeMotorConfig.closedLoop
                        .maxMotion.maxAcceleration(1000000);


                        // IntakeRightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        //     // Set PID values for position control. We don't need to pass a closed
                        //     // loop slot, as it will default to slot 0.
                        //     .p(0.0002355)
                        //     .i(0)
                        //     .d(0)
                        //     .outputRange(-1, 1);
                        // IntakeRightMotorConfig.closedLoop
                        // .maxMotion.maxAcceleration(10000);

                }

        };

        public static final class ClimberSubsystem {
                
            public static final SparkFlexConfig ClimbMotorLeftConfig = new SparkFlexConfig();
            

                static {

                        ClimbMotorLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        ClimbMotorLeftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(0.0002355)
                        .i(0)
                        .d(0)
                        .outputRange(-1, 1);
                        ClimbMotorLeftConfig.closedLoop
                                .maxMotion
                                        .maxAcceleration(600000)
                                        .cruiseVelocity(600000)
                                        .allowedProfileError(0.1); // smooth extension

                                
                }

        };


        public static final class PushoutSubsystem {

            public static final SparkFlexConfig PushoutMotorConfig = new SparkFlexConfig();
            // public static final SparkFlexConfig PushoutRightMotorConfig = new SparkFlexConfig();


                static {
                        PushoutMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);


                        PushoutMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(3.3)
                        .i(0.0)
                        .d(0.001)
                        .outputRange(-1.0, 1.0);
                    
                        
                        PushoutMotorConfig.closedLoop
                        .maxMotion
                                .allowedProfileError(0.5)
                                .cruiseVelocity(10000)
                                .maxAcceleration(10000);   
                                                             

                        // PushoutRightMotorConfig.closedLoop
                        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // .p(0.2)
                        // .i(0.0)
                        // .d(0.05)
                        // .outputRange(-1.0, 1.0)
                        // .maxMotion
                        //         .maxAcceleration(3000)
                        //         .cruiseVelocity(100)
                        //         .allowedProfileError(0.01); // smooth extension, allow some error to prevent oscillation
                                
                }

        };

        public static final class KickerSubsystem {
                public static final SparkFlexConfig kickerLeftMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig kickerRightMotorConfig = new SparkFlexConfig();

                        static {
                                kickerLeftMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                                kickerRightMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12).follow(13,true);
                                
                                kickerLeftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0)
                                .d(0)
                                .outputRange(-1, 1)
                                 .feedForward
                                .kS(0.10)
                                .kV(0.003)
                                .kA(0.0003);
                                kickerLeftMotorConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);


                                kickerRightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0)
                                .d(0)
                                .outputRange(-1, 1)
                                 .feedForward
                                .kS(0.10)
                                .kV(0.003)
                                .kA(0.0003);

                                kickerRightMotorConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);
                        }

        }
        public static final class ShooterSubsystem {
                public static final SparkFlexConfig ShooterRightMotor1Config = new SparkFlexConfig(); 
                public static final SparkFlexConfig ShooterRightMotor2Config = new SparkFlexConfig(); 

                public static final SparkFlexConfig ShooterLeftMotor1Config = new SparkFlexConfig();
                public static final SparkFlexConfig ShooterLeftMotor2Config = new SparkFlexConfig();
                
                        static {


                                ShooterRightMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(45).voltageCompensation(12);
                                ShooterRightMotor2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(45).voltageCompensation(12)
                                .follow(11, true);
                                
                                ShooterLeftMotor1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(45).voltageCompensation(12);
                                ShooterLeftMotor2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(45).voltageCompensation(12)
                                .follow(9, true);
                                
                                
                                ShooterRightMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;              

                                ShooterRightMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterRightMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;

                                ShooterRightMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);
                                
                                ShooterLeftMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;      
                                
                                ShooterLeftMotor1Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                                ShooterLeftMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // Set PID values for position control. We don't need to pass a closed
                                // loop slot, as it will default to slot 0.
                                .p(0.0002355)
                                .i(0.0)
                                .d(0.000)
                                .outputRange(-1, 1)
                                .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017)
                                ;

                                ShooterLeftMotor2Config.closedLoop
                                .maxMotion.maxAcceleration(10000);

                }

 
        }

        public static final class HopperSubsystem {
                
            public static final SparkFlexConfig TwindexerRightControllerConfig = new SparkFlexConfig();
            public static final SparkFlexConfig TwindexerLeftControllerConfig = new SparkFlexConfig();

                static {

                        TwindexerRightControllerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12).follow(15, true);
                        TwindexerLeftControllerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

                        TwindexerRightControllerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for  position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.0002)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1)
                             .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017);
                        TwindexerRightControllerConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
                        

                            TwindexerLeftControllerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            // Set PID values for position control. We don't need to pass a closed
                            // loop slot, as it will default to slot 0.
                            .p(0.0002)
                            .i(0)
                            .d(0)
                            .outputRange(-1, 1)
                             .feedForward
                                .kS(0.10)
                                .kV(0.00177)
                                .kA(0.00017);
                        TwindexerLeftControllerConfig.closedLoop
                                .maxMotion.maxAcceleration(100000);
                            // Set PID values for velocity control in slot 1
                        //     .p(0.0001, ClosedLoopSlot.kSlot1)
                        //     .i(0, ClosedLoopSlot.kSlot1)
                        //     .d(0, ClosedLoopSlot.kSlot1)
                        //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                        //     .feedForward
                        //     // kV is now in Volts, so we multiply by the nominal voltage (12V)
                        //     .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

                     

                }
        }

}

