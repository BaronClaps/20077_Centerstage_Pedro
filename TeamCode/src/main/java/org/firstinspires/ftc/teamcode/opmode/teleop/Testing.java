package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Baron")
public class Testing extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lF        = null; //c0
    private DcMotor rF        = null; //c1
    private DcMotor lB        = null; //c2
    private DcMotor rB        = null; //c3
    private DcMotor lift      = null; //e0
    private DcMotor gear      = null; //e1
    private Servo pivot       = null; //es0
    private Servo clawL       = null; //es1
    private Servo clawR       = null; //es2
    private Servo droneServo  = null; //es5
    private Servo WheelServo  = null; //es4

    private double groundClawPos = 0.835;
    private double clawPosSpecial = 0.685;
    private double clawPos = groundClawPos;
    @Override
    public void runOpMode() {
        lF = hardwareMap.get(DcMotor.class, "lF");
        rF = hardwareMap.get(DcMotor.class, "rF");
        lB = hardwareMap.get(DcMotor.class, "lB");
        rB = hardwareMap.get(DcMotor.class, "rB");
        lift = hardwareMap.get(DcMotor.class, "lift");
        gear = hardwareMap.get(DcMotor.class, "gear");
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        WheelServo = hardwareMap.get(Servo.class, "WheelServo");
        WheelServo.setPosition(0.85);
        droneServo.setPosition(0.6);
        lF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        gear.setDirection(DcMotor.Direction.REVERSE);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gear.setTargetPosition(0);
        lift.setTargetPosition(0);
        pivot.setPosition(.835);
        clawL.setPosition(.45);
        clawR.setPosition(.25);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean clawState = false;
        boolean oldArmButton = true;

        //----------------------------Main-Code----------------------------\\

        while (opModeIsActive()) {


            //----------------------------Mecanum-Drive-Code----------------------------\\

            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double lFPower = axial + lateral + yaw;
            double rFPower = axial - lateral - yaw;
            double lBPower = axial - lateral + yaw;
            double rBPower = axial + lateral - yaw;
            max = Math.max(Math.abs(lFPower), Math.abs(rFPower));
            max = Math.max(max, Math.abs(lBPower));
            max = Math.max(max, Math.abs(rBPower));
            if (max > 1.0) {
                lFPower /= max;
                rFPower /= max;
                lBPower /= max;
                rBPower /= max; }

            //----------------------------Drive-Speed----------------------------\\

            lF.setPower(0.8 * lFPower);
            rF.setPower(0.8 * rFPower);
            lB.setPower(0.8 * lBPower);
            rB.setPower(0.8 * rBPower);



            //----------------------------pivot----------------------------\\

            pivot.setPosition(clawPos);

            if (gamepad1.a) {
                clawPos = groundClawPos;
            }
            if (gamepad1.b) {
                clawPos += .001;
            }
            if (gamepad1.x) {
                clawPos -= .001;
            }
            if (gamepad1.y) {
                clawPos = clawPosSpecial;
            }
            telemetry.addData("Claw Pos", clawPos);



            //----------------------------claw----------------------------\\
//left
            if (gamepad1.left_bumper) {
                clawL.setPosition(0.33);
            } else {
                clawL.setPosition(.42);
            } //open
//right
            if (gamepad1.right_bumper) {
                clawR.setPosition(0.37);
            } else {
                clawR.setPosition(.28);
            } //open



            //----------------------------lift/gear----------------------------\\

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //lift.setPower(1);
            gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //gear.setPower(0.5);



            if (gamepad1.dpad_up) { //manual gear up
                raegup(1);
                telemetry.addData("gear", gear.getCurrentPosition());
            }
            if (gamepad1.dpad_down) { //manuel gear down
                raeg(-1);
                telemetry.addData("gear", gear.getCurrentPosition());
            }

            if (gamepad1.left_trigger > .1) {
                tfil(150, -1);
                telemetry.addData("lift", lift.getCurrentPosition());}
            if (gamepad1.right_trigger > .1) {
                tfil(200, 1);
                telemetry.addData("lift", lift.getCurrentPosition());}



            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("gear", gear.getCurrentPosition());
            telemetry.update();


        }

    }

    public void raegPosition (int h, double H) {gear.setTargetPosition(h); gear.setPower(H); }
    public void tfilPosition (int e, double E) {lift.setTargetPosition(e); lift.setPower(E); }

    public void raeg (int s) {
        gear.setPower(0.425);
        gear.setTargetPosition(gear.getCurrentPosition() + 30 * s);//was *25
    }

    public void gearupHANG (int s) {
        gear.setPower(1);
        gear.setTargetPosition(gear.getCurrentPosition() + 50 * s);//was *25
    }
    public void geardownHANG (int s) {
        gear.setPower(1);
        gear.setTargetPosition(gear.getCurrentPosition() + 50 * s);//was *25
    }

    public void raegup (int s) {
        gear.setPower(0.425);
        gear.setTargetPosition(gear.getCurrentPosition() + 30 * s);//was *25
    }
    public void tfil (int c, int s) {
        lift.setPower(0.95);
        lift.setTargetPosition(lift.getCurrentPosition() + c * s);//c was 250
        /*if (lift.getTargetPosition() < 50) {
            lift.setTargetPosition(50); }
        if (lift.getTargetPosition() > 3000) {
            lift.setTargetPosition(3000); }*/
    }
    public void fasttfil (int s) {
        lift.setPower(1);
        lift.setTargetPosition(lift.getCurrentPosition() + 250 * s);
        if (lift.getTargetPosition() > 2500) {
            lift.setTargetPosition(2500); }
    }
    //public void Tilt (double p) {pivot.setPosition(pivot.getPosition() + p); }
}
