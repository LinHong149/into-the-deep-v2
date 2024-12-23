package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;



@TeleOp
public class sample extends LinearOpMode{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx AMotor, S1Motor, S2Motor, FL, FR, BL, BR = null;
    Servo rotation, wrist, claw;

    public double wristPar = 0, wristPerp = 0.55, wristOuttake = 0.75;
    public double clawOpen = 0.2, clawClose = 0.52;
    public double rotationPos = 0.5;
    public double armPar = 325, armUp = 1650;
    public int slideInterval = 15;

    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    static double armP = 0.0025, armI = 0, armD = 0.000023, armF = 0;
    static double armPE = 0.003, armIE = 0, armDE = 0.000023, armFE = 0;
    static double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    static double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    static double slidePE = 0.045, slideIE = 0, slideDE = 0.0004, slideFE = 0;
    static double slideTarget = 0.0;
    double slidePower = 0.0;


    boolean rightBumperPrevState = false;
    boolean hangPrev = false;
    boolean clawPressed = false;
    boolean clawIsOpen = false;
    boolean init = true;
    boolean slideExtended = false;
    boolean retractSlide = false;
    boolean retracted = true;
    boolean slideOuttake = false;
    boolean micro = false;
    boolean intakePrev = false;

    double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    double armTempTarget = armPar;
    double armMax = 2300;
    double slideMax = 2900;

    public enum Mode {
        REST,
        OUTTAKING,
        INTAKING,
        HANG
    }
    Mode mode = Mode.REST;



    public void initHardware() {
        AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
        FL = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        FR = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        BL = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        BR = hardwareMap.get(DcMotorEx.class,"backRightMotor");


        rotation = hardwareMap.get(Servo.class,"rotation");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");

        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.FORWARD);
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        wrist.setPosition(wristPerp);
        claw.setPosition(clawClose);
        rotation.setPosition(0.5);

        AMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        AMotor.setPower(0);
        armTarget = 800;

        S1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S1Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        S1Motor.setPower(0);

        S2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        S2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        S2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        S2Motor.setPower(0);

        armTarget = 800;
//        Runs to arm pose
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(AMotor.getCurrentPosition() - armTarget) > 10 && timer.seconds() < 3) { // Safety timeout of 3 seconds
            double power = armPIDF(armTarget, AMotor);
            AMotor.setPower(power);

            telemetry.addData("Arm Position", AMotor.getCurrentPosition());
            telemetry.addData("Arm Target", armTarget);
            telemetry.update();
        }
        AMotor.setPower(0);
    }

//
//                        /^--^\     /^--^\     /^--^\
//                        \____/     \____/     \____/
//                       /      \   /      \   /      \
//                      |        | |        | |        |
//                       \__  __/   \__  __/   \__  __/
//  |^|^|^|^|^|^|^|^|^|^|^|^\ \^|^|^|^/ /^|^|^|^|^\ \^|^|^|^|^|^|^|^|^|^|^|^|
//  | | | | | | | | | | | | |\ \| | |/ /| | | | | | \ \ | | | | | | | | | | |
//  ########################/ /######\ \###########/ /#######################
//  | | | | | | | | | | | | \/| | | | \/| | | | | |\/ | | | | | | | | | | | |
//  |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|


    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
//  DRIVE
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (!micro) {
                double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx),1);
                frontLeftPower = (y + x + rx) / denom;
                backLeftPower = (y - x + rx) / denom;
                frontRightPower = (y - x - rx) / denom;
                backRightPower = (y + x - rx) / denom;

                FL.setPower(frontLeftPower);
                FR.setPower(frontRightPower);
                BL.setPower(backLeftPower);
                BR.setPower(backRightPower);
            }else{
                //TODO trig calculation for rotation

                frontLeftPower = rx/3;
                backLeftPower = rx/3;
                frontRightPower = -rx/3;
                backRightPower = -rx/3;

                FL.setPower(frontLeftPower);
                FR.setPower(frontRightPower);
                BL.setPower(backLeftPower);
                BR.setPower(backRightPower);
//                if (x!= 0) {
//                    if (y>=0) {
//                        rotation.setPosition(1 - (Math.acos(x / (Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5))) / Math.PI));
//                    }
//                }
                slideTarget += (y>0 && slideTarget<slideMax) ? slideInterval*y/1.5:0;
                slideTarget += (y<0 && slideTarget>500) ? slideInterval*y/1.5:0;
                if (gamepad1.left_trigger > 0 && rotationPos >=0 ) {
                    rotationPos -= gamepad1.left_trigger / 80;
                    if (rotationPos <0) rotationPos = 1; // Ensure upper bound
                }
                if (gamepad1.right_trigger > 0 && rotationPos <=1) {
                    rotationPos += gamepad1.right_trigger / 80;
                    if (rotationPos >1) rotationPos = 0; // Ensure lower bound
                }
                rotation.setPosition(rotationPos);
            }

//  ARM & SLIDE PID
            if (armTarget >= 0 && armTarget <= armMax) {
                AMotor.setPower(armPIDF(armTarget, AMotor));
            }else{
                AMotor.setPower(0);
            }
            if (slideTarget >= 200 && slideTarget <= slideMax) {
                slidePower = slidePIDF(slideTarget, S1Motor);
                S1Motor.setPower(slidePower);
                S2Motor.setPower(slidePower);
            }else{
                S1Motor.setPower(0);
                S2Motor.setPower(0);
            }

            if (mode==Mode.INTAKING || micro){
                slideMax = 2900;
            }else{
                slideMax = 5300;
            }


//  CLAW
            if (gamepad1.a && mode != Mode.HANG) {
                if (!clawPressed) {
                    clawPressed=true;
                    clawIsOpen = !clawIsOpen;
                }
            } else {
                clawPressed = false;
            }

            if (!clawIsOpen){
                claw.setPosition(clawClose);
            }else{
                claw.setPosition(clawOpen);
            }

//  SLIDES
            slideTarget += (gamepad1.dpad_up && slideTarget<slideMax) ? slideInterval : 0;
            slideTarget -= (gamepad1.dpad_down && slideTarget>500) ? slideInterval : 0;
            slideTarget = Math.min(5300, Math.max(200, slideTarget));

            slideExtended = slideTarget > 300;

//  ARM

            armTempTarget += (gamepad1.left_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget -= (gamepad1.right_trigger > 0 && !micro) ? 3 : 0;
            armTempTarget = Math.min(2300, Math.max(0, armTempTarget));

            armPar = (slideTarget > 300) ? 325 : 400;

//             /\_/\
//            ( o.o )
//             > ^ <    Purrrr...


//  MODES
            boolean rightBumperCurrentState = gamepad1.right_bumper;
            if (rightBumperCurrentState && !rightBumperPrevState) {
                if (mode == Mode.REST) {
                    mode = Mode.OUTTAKING;
                    slideInterval = 24;
                    init = true;
                } else if (mode == Mode.OUTTAKING) {
                    retractSlide=true;
                    slideTarget = 500;
                } else if (mode == Mode.INTAKING){
                    micro = false;
                    armTempTarget = armPar;
                    wrist.setPosition(wristPerp);
                    armTarget = armTempTarget;
//                    retractSlide = true;
                    slideTarget = 200;
                }
            }
            rightBumperPrevState = rightBumperCurrentState;
// RETRACT SLIDE
            if (retractSlide) {
                slideTarget = 200;
                if (S1Motor.getCurrentPosition() < 1800) {
                    retractSlide = false;
                    mode=Mode.REST;
                    init = true;
                }

            }


            telemetry.addData("retract",retractSlide);


            boolean hangCurr = gamepad1.left_stick_button;
            if (hangCurr && !hangPrev) {
                if (mode == Mode.REST) {
                    mode = Mode.HANG;
                } else if (mode == Mode.HANG) {
                    mode = Mode.REST;
                }
                init = true;
            }
            hangPrev = hangCurr;


            telemetry.addData("mode type",mode);
            switch (mode) {
/** REST */
                case REST:
                    if (init) {
                        wrist.setPosition(wristPerp);
                        slideTarget = 200;
                        armTempTarget = armPar;
                        rotation.setPosition(0.5);
                    }
                    init = false;

// ARM POSITION
                    armTarget = armTempTarget;

// CHANGE TO INTAKING



                    if (slideTarget > 300) {
                        retracted = false;
                        mode = Mode.INTAKING;
                        init = true;
                    }

                    boolean intakeCurr = gamepad1.left_bumper;
                    if (intakeCurr && !intakePrev){
                        micro = true;
                        rotationPos = 0.5;
                        slideTarget = 1500;
                        mode = Mode.INTAKING;
                        init = true;
                    }
                    intakePrev = intakeCurr;

                    break;

/** INTAKING */
                case INTAKING:
                    if (init) {
                        wrist.setPosition(wristPar);
                        clawIsOpen = true;
                        armTempTarget = armPar;
                    }
                    init = false;


//  LOWER ARM
                    armTarget = (gamepad1.left_bumper) ? 175 : armTempTarget;

//  CHANGE TO REST
                    if (slideTarget <= 250){
                        mode = Mode.REST;
                        init = true;//retract slide < 70; rest <= 80; intaking > 80
                    }

                    break;

/** OUTTAKING */
                case OUTTAKING:
                    if (init) {
                        armTempTarget = armUp;
                        slideOuttake = true;
                        rotation.setPosition(0.5);
                        wrist.setPosition(wristPar);


                    }
                    init = false;

                    if (slideOuttake && armTempTarget-AMotor.getCurrentPosition()<1000){
                        slideTarget = slideMax;
                        slideOuttake = false;
                    }

                    if (gamepad1.left_bumper){
                        wrist.setPosition(wristOuttake);
                    }else{
                        wrist.setPosition(wristPar);
                    }

//  ARM
                    armTarget = armTempTarget;

                    break;

/** HANG */
                case HANG:
                    if (init) {
                        clawIsOpen = false;
                        armTarget = 1200;
                        slideTarget = 1300;
                        wrist.setPosition(wristPar);
                        rotation.setPosition(0.5);
                    }


                    break;
            }


            telemetry.addData("arm current",AMotor.getCurrentPosition());
            telemetry.addData("arm target",armTarget);
            telemetry.addData("slide1 current",S1Motor.getCurrentPosition());
            telemetry.addData("slide2 current",S2Motor.getCurrentPosition());
            telemetry.addData("slide target",slideTarget);

            telemetry.addData("rotation",rotationPos);
            telemetry.addData("init",init);

            telemetry.addData("hello", "hellow");

            telemetry.update();
            dashboardTelemetry.update();

        }
    }

    public double armPIDF(double target, DcMotorEx motor){
        if (slideExtended) {
            armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        } else {
            armPIDF.setPIDF(armP,armI,armD,armF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        telemetry.update();
        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        if (mode == Mode.OUTTAKING){
            slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        }else {
            slidePIDF.setPIDF(slideP, slideI, slideD, slideF);
        }
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        telemetry.update();
        return output;
    }

}
