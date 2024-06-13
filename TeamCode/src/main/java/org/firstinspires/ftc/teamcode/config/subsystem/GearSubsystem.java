package org.firstinspires.ftc.teamcode.config.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


public class GearSubsystem {
    public DcMotorEx gear;
    private PIDController controller;
    public static double p = 0.0215, i = 0, d = 0.0005;
    public static double f = 0.05;
    public int gearTarget = 0;
    public static double ticks_in_degree = 7.7395;
    public int gearPos;

    //private Servo wheelServo;

    public GearSubsystem(HardwareMap hardwareMap) {
        gear = hardwareMap.get(DcMotorEx.class, "gear");
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gear.setDirection(DcMotorSimple.Direction.REVERSE);


        //wheelServo = hardwareMap.get(Servo.class, "WheelServo");
    }

    public void gearTarget(int gtarget){
        gearTarget = gtarget;
    }

   public void gearPIDUpdate() {
        controller = new PIDController(p,i,d);
        int gearPos = gear.getCurrentPosition() + 600;
        double pid = controller.calculate(gearPos, gearTarget);
        double ff = Math.cos(Math.toRadians(gearTarget / ticks_in_degree)) * f;

        double power = pid + ff;

        gear.setPower(power);
    }

    /*public void gearTarget(int gearTarget) {
    double p = 0.0215, i = 0, d = 0.0005;
    double f = 0.05;
    double ticks_in_degree = 7.7395; // (pulses per revolution) / (degrees in revolution)
    double pid;
    double ff;
    double power;

    PIDController controller;
    controller = new PIDController(p,i,d);
    gear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    while(gearPos != gearTarget+-5 ){
        int target = gearTarget;
        controller.setPID(p, i, d);
        int gearPos = gear.getCurrentPosition();
        pid = controller.calculate(gearPos, target);
        ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        power = pid + ff;
        power = Range.clip(power, -.75, .75);

        gear.setPower(power);


    }
    }*/

    //------------------------------Ground Position------------------------------//
    public void groundGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-760);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.25);
    }

    public void whiteGroundGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-348); //-300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    //------------------------------Scoring Position------------------------------//
    public void scoringGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(760);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.75);
    }

    public void whiteScoringGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(348); //300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
            }

    //------------------------------Start Position-------------------------------//
    public void startGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-500);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.25);
    }

    //------------------------------End Position-------------------------------//
    public void endGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-500);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }


    //------------------------------ Stop Gear -------------------------------//
    public void stopGear() {
                gear.setPower(0);
    }

    public void resetGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //------------------------------ Wheel Servo for Stack -------------------------------//
 /*   public void wheelServo_Activated1() {
        stopGear();
        wheelServo.setPosition(0.636); // Top 2 Pixels | bigger # = lower // //0.641
    }

    public void wheelServo_Activated2() {
        stopGear();
        wheelServo.setPosition(0.5725); // Top 2 Pixels | bigger # = lower // //0.641
    }

    public void wheelServo_Deactivated() {
        wheelServo.setPosition(0.85);
    } */
}