package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class PIDTuning extends LinearOpMode {
    public DcMotorEx S1Motor, S2Motor, AMotor, FL, FR, BR, BL;

    public double clawROpen = 0.2, clawRClose = 0.52;
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    public static double armP = 0.0025, armI = 0, armD = 0.000023, armF = 0;
    //    extended PID
    public static double armPE = 0.003, armIE = 0, armDE = 0.000023, armFE = 0;
    public static double armTarget = 0.0;
    public double armPower = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public static double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    public static double slidePE = 0.045, slideIE = 0, slideDE = 0.0004, slideFE = 0;
    public static double slideTarget = 0.0;
    public double slidePower = 0.0;

    public enum Mode {
        INTAKING,
        OUTTAKING,
        REST,
        HANG
    }

    Mode mode = Mode.REST;


    public void initHardware() {
//      DRIVE MOTORS

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setDirection(DcMotorEx.Direction.FORWARD);
        BL.setDirection(DcMotorEx.Direction.FORWARD);
        FR.setDirection(DcMotorEx.Direction.REVERSE);
        BR.setDirection(DcMotorEx.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);


//        ARM AND SLIDE
        S1Motor = hardwareMap.get(DcMotorEx.class, "S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class, "S2Motor");
        AMotor = hardwareMap.get(DcMotorEx.class, "AMotor");

        S1Motor.setDirection(DcMotorEx.Direction.REVERSE);
        S2Motor.setDirection(DcMotorEx.Direction.REVERSE);
        AMotor.setDirection(DcMotorEx.Direction.FORWARD);

        S1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        S2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        AMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        S1Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        S2Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        AMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        S1Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        S2Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        AMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        S1Motor.setPower(0);
        S2Motor.setPower(0);
        AMotor.setPower(0);
    }

    @Override
    public void runOpMode(){
        initHardware();

        waitForStart();

        while(opModeIsActive()){
            armPower = armPIDF(armTarget, AMotor);
            AMotor.setPower(armPower);

            slidePower = slidePIDF(slideTarget, S1Motor);
            S1Motor.setPower(slidePower);
            S2Motor.setPower(slidePower);

            telemetry.addData("armTarget", armTarget);
            telemetry.addData("armPower", armPower);
            telemetry.addData("slideTarget", slideTarget);
            telemetry.addData("slidePower", slidePower);
            telemetry.update();
        }
    }

    public double armPIDF(double target, DcMotorEx motor){
        armPIDF.setPIDF(armPE,armIE,armDE,armFE);
        int currentPosition = motor.getCurrentPosition();
        double output = armPIDF.calculate(currentPosition, target);

        return output;
    }

    public double slidePIDF(double target, DcMotorEx motor){
        slidePIDF.setPIDF(slidePE,slideIE,slideDE,slideFE);
        int currentPosition = motor.getCurrentPosition();
        double output = slidePIDF.calculate(currentPosition, target);

        return output;
    }
}
