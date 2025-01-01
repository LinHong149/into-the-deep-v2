package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.sample;

@Config
@Autonomous
public class specimenAuton extends LinearOpMode{

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.8;
    public double clawOpen = 0.25, clawClose = 0.75;
    public double rotationPos = 0.465;
    public double armDown = 50;
    public double armPar = 100, armUp = 1130;
    public double slideRest = 200, slideOuttakingIn = 750, slideOuttakingOut = 1400;
    public int slideInterval = 15;
    public double outToRestBuffer = 800, restToOuttake = 1000;


    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    static double armP = 0.007, armI = 0, armD = 0, armF = 0;
    static double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    static double slideP = 0.0034, slideI = 0, slideD = 0, slideF = 0;
    static double slidePE = 0.008, slideIE = 0, slideDE = 0, slideFE = 0;
    static double slideTarget = 0.0;
    double slidePower = 0.0;

    public enum Mode {
        INTAKING,
        REST,
        OUTTAKING,
        HANG
    }
    public Mode mode = Mode.INTAKING;


    /** Arm and slide*/
    public class ArmSlide {
        private DcMotorEx S1Motor, S2Motor, AMotor;

        public ArmSlide (HardwareMap hardwareMap) {
            S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
            S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
            AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");

            AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            AMotor.setPower(0);

            S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            S1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            S1Motor.setDirection(DcMotorSimple.Direction.REVERSE);
            S1Motor.setPower(0);

            S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            S2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            S2Motor.setDirection(DcMotorSimple.Direction.REVERSE);
            S2Motor.setPower(0);
        }

        /** To rest */
        public class ToRest implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.REST;
                slideTarget = slideRest;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());

//                Arm down after slide comes down
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 400) {
                    armTarget = armPar;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 20 && armTarget == armPar) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action toRest() {
            return new ToRest();
        }


        /** To intaking */
        public class RestToIntaking implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.INTAKING;
                slideTarget = slideIntaking;
                armTarget = armPar;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm curr", AMotor.getCurrentPosition());
                packet.put("slide curr", S1Motor.getCurrentPosition());
                packet.put("arm tar", armTarget);
                packet.put("slide tar", slideTarget);


//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 100 && Math.abs(AMotor.getCurrentPosition() - armTarget) < 20) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action restToIntaking() {
            return new RestToIntaking();
        }



        /** To outtaking */
        public class RestToOuttaking implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.OUTTAKING;
                armTarget = armUp;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());


//                Arm down after slide comes down
                if (Math.abs(AMotor.getCurrentPosition() - armTarget) < 400) {
                    slideTarget = slideOuttaking;
                }

//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 200 && slideTarget == slideOuttaking) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action restToOuttaking() {
            return new RestToOuttaking();
        }


        /** To hang */
        public class RestToHang implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                mode = Mode.HANG;
                armTarget = armHang;
                slideTarget = slideHang;

//                PID
                AMotor.setPower(armPIDF(armTarget, AMotor));
                slidePower = slidePIDF(slideTarget, S1Motor, S2Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);

                packet.put("arm", AMotor.getCurrentPosition());
                packet.put("slide", S1Motor.getCurrentPosition());


//                Keep repeating until arm Motor gets to target
                if (Math.abs(S1Motor.getCurrentPosition() - slideTarget) < 200) {
                    S1Motor.setPower(0);
                    S2Motor.setPower(0);
                    AMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action restToHang() {
            return new RestToHang();
        }
    }


    /** Intaking system class */
    public class IntakingSystem {
        private Servo claw, wrist, rotation;


        public IntakingSystem(HardwareMap hardwareMap) {
            rotation = hardwareMap.get(Servo.class, "rotation");
            wrist = hardwareMap.get(Servo.class, "wrist");
            claw = hardwareMap.get(Servo.class, "claw");
        }


        /** Open claw intaking */
        public class IntakingOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPar);
                claw.setPosition(clawOpen);

                packet.put("intaking", "intaking open claw");
                return false;
            }
        }
        public Action intakingOpenClaw() {
            return new IntakingOpenClaw();
        }

        /** Close claw intaking */
        public class IntakingCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPar);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }
        public Action intakingCloseClaw() {
            return new IntakingCloseClaw();
        }

        /** Close claw rest */
        public class RestCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristPerp);
                claw.setPosition(clawClose);

                packet.put("intaking", "intaking close claw");
                return false;
            }
        }
        public Action restCloseClaw() {
            return new RestCloseClaw();
        }

        /** Open claw outtaking */
        public class OuttakingOpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(rotationPos);
                wrist.setPosition(wristOuttake);
                claw.setPosition(clawOpen);

                packet.put("intaking", "outtaking open claw");
                return false;
            }
        }
        public Action outtakingOpenClaw() {
            return new OuttakingOpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-38, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakingSystem intakingSystem = new IntakingSystem(hardwareMap);
        ArmSlide armSlide = new ArmSlide(hardwareMap);


        TrajectoryActionBuilder spin = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-52, -53, Math.toRadians(45)), Math.toRadians(45)) //drop off pre loaded
                .waitSeconds(1)
                .splineTo(new Vector2d(-48,-40),Math.toRadians(90)) //to first sample; rotation 0.5
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 1
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-53,-40),Math.toRadians(110)) //to second sample; rotation 0.75
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 2
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-56,-40),Math.toRadians(130)) // to third sample; rotation 0.95/1
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-52,-53),Math.toRadians(225)) //drop off sample 3
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-30,-11),Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder dropOffPreload = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-53,-55,Math.toRadians(45)),Math.toRadians(45))
                .waitSeconds(.2);
        TrajectoryActionBuilder toFirstSample = drive.actionBuilder(new Pose2d(-52,-53,Math.toRadians(45)))
                .splineTo(new Vector2d(-46,-40),Math.toRadians(90))
                .waitSeconds(.3);
        TrajectoryActionBuilder dropOffFirst = drive.actionBuilder(new Pose2d(-44,-40,Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-53,-54),Math.toRadians(225))
                .waitSeconds(.3);
        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(-52,-53,Math.toRadians(45)))
                .setReversed(false)
                .splineTo(new Vector2d(-53,-37),Math.toRadians(105))
                .waitSeconds(.3);
        TrajectoryActionBuilder dropOffSecond = drive.actionBuilder(new Pose2d(-53,-40,Math.toRadians(105)))
                .setReversed(true)
                .splineTo(new Vector2d(-53,-54),Math.toRadians(225))
                .waitSeconds(.3);
        TrajectoryActionBuilder toThirdSample = drive.actionBuilder(new Pose2d(-52,-53,Math.toRadians(45)))
                .setReversed(false)
                .splineTo(new Vector2d(-57,-35),Math.toRadians(125))
                .waitSeconds(.3);
        TrajectoryActionBuilder dropOffThird = drive.actionBuilder(new Pose2d(-56,-40,Math.toRadians(125)))
                .setReversed(true)
                .splineTo(new Vector2d(-53,-55),Math.toRadians(225))
                .waitSeconds(.3);
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-52,-53,Math.toRadians(45)))
                .setReversed(false)
                .splineTo(new Vector2d(-30,-11),Math.toRadians(0))
                .waitSeconds(.3);




//        Initialization
        Actions.runBlocking(
                new SequentialAction(
                        intakingSystem.restCloseClaw(),
                        armSlide.toRest()
                )
        );

        waitForStart();

//        Code Running
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
//                            dropOffPreload.build(),
//                            toFirstSample.build(),
//                            dropOffFirst.build(),
//                            toSecondSample.build(),
//                            dropOffSecond.build(),
//                            toThirdSample.build(),
//                            dropOffThird.build(),
//                            park.build()

//                            Preloaded scoring
                            armSlide.restToOuttaking(),
                            dropOffPreload.build(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.3),
                            intakingSystem.intakingOpenClaw(),
                            armSlide.toRest(),
                            intakingSystem.restCloseClaw(),

//                        Grabbing sample 1 and scoring
                            toFirstSample.build(),
                            intakingSystem.intakingOpenClaw(),
                            new SleepAction(0.3),
                            intakingSystem.intakingCloseClaw(),
                            new SleepAction(0.3),
                            intakingSystem.restCloseClaw(),
                            dropOffFirst.build(),
                            armSlide.restToOuttaking(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.3),
                            intakingSystem.intakingOpenClaw(),
                            armSlide.toRest(),
                            intakingSystem.restCloseClaw(),

//                        Grabbing sample 2 and scoring
                            toSecondSample.build(),
                            intakingSystem.intakingOpenClaw(),
                            new SleepAction(0.3),
                            intakingSystem.intakingCloseClaw(),
                            new SleepAction(0.3),
                            intakingSystem.restCloseClaw(),
                            dropOffSecond.build(),
                            armSlide.restToOuttaking(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.3),
                            intakingSystem.intakingOpenClaw(),
                            armSlide.toRest(),
                            intakingSystem.restCloseClaw(),

//                        Grabbing sample 3 and scoring
                            toThirdSample.build(),
                            intakingSystem.intakingOpenClaw(),
                            new SleepAction(0.3),
                            intakingSystem.intakingCloseClaw(),
                            new SleepAction(0.3),
                            intakingSystem.restCloseClaw(),
                            dropOffThird.build(),
                            armSlide.restToOuttaking(),
                            new SleepAction(.3),
                            intakingSystem.outtakingOpenClaw(),
                            new SleepAction(.3),
                            intakingSystem.intakingOpenClaw(),
                            armSlide.toRest(),
                            intakingSystem.restCloseClaw(),

                            park.build(),
                            armSlide.restToHang(),
                            intakingSystem.intakingCloseClaw()







                    )
            );
        }



    }





    public double armPIDF(double target, DcMotorEx motor){
        armPIDF.setPIDF(armP,armI,armD,armF);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor,DcMotorEx motor2){
        if (mode == Mode.OUTTAKING){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = (motor.getCurrentPosition()+motor2.getCurrentPosition())/2;
        double output = slidePIDF.calculate(currentPosition, target);


        return output;
    }
}

