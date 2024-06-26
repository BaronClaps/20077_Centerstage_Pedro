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
    public Servo wheelServo;
    private PIDController controller;
    public static double Gp = 0.0215, Gi = 0, Gd = 0.0005;
    public static double Gf = 0.05;
    public int gearTarget = 0;
    public static double Gticks_in_degree = 7.7395;
    public int gearPos;
    public double gearPower;

    //private Servo wheelServo;

    public GearSubsystem(HardwareMap hardwareMap) {
        gear = hardwareMap.get(DcMotorEx.class, "gear");
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gear.setDirection(DcMotorSimple.Direction.REVERSE);
        gear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        wheelServo = hardwareMap.get(Servo.class, "WheelServo");
    }
/*
    public void gearTarget(int gtarget){
        gearTarget = gtarget;
    }

    public void gearPIDUpdate() {

        controller = new PIDController(Gp,Gi,Gd);
        int gearPos = gear.getCurrentPosition();
        double Gpid = controller.calculate(gearPos, gearTarget);
        double Gff = Math.cos(Math.toRadians(gearTarget / Gticks_in_degree)) * Gf;

        double power = Gpid + Gff;

        gear.setPower(power * 0.7);
    }

    public void groundGear(){
        gearTarget(0);
    }

    public void scoringGear() {
        gearTarget(950);
    }*/

    /*public void white1() {
        gearTarget(5);
    }*/

    //------------------------------Ground Position------------------------------//
    public void groundGear() {
                gear.setTargetPosition(10);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.25);
    }

    public void whiteGroundGear() {

                gear.setTargetPosition(10); //-300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    //------------------------------Scoring Position------------------------------//
    public void scoringGear() {
                gear.setTargetPosition(805);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(1);
    }

    public void scoringGearRED() {
        gear.setTargetPosition(805);
        gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gear.setPower(1);
    }

    public void scoringGearZone1() {
        gear.setTargetPosition(745);
        gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(gear.getCurrentPosition() <= 400) {
            gearPower = 1;
        } else if(gear.getCurrentPosition() > 400 && gear.getCurrentPosition() <= 500){
            gearPower = 0.5;
        }
        else if(gear.getCurrentPosition() > 500 && gear.getCurrentPosition() <= 600){
            gearPower = 0.4;
        }
        else if(gear.getCurrentPosition() > 600 && gear.getCurrentPosition() <= 700){
            gearPower = 0.33;
        }
        else if(gear.getCurrentPosition() > 700 && gear.getCurrentPosition() <= 745){
            gearPower = 0.1;
        }
        else {
            gearPower = 0;
        }
        gear.setPower(gearPower);
    }

    public void scoringGear2Zone1() {
        gear.setTargetPosition(735);
        gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(gear.getCurrentPosition() <= 400) {
            gearPower = 1;
        } else if(gear.getCurrentPosition() > 400 && gear.getCurrentPosition() <= 500){
            gearPower = 0.5;
        }
        else if(gear.getCurrentPosition() > 500 && gear.getCurrentPosition() <= 600){
            gearPower = 0.4;
        }
        else if(gear.getCurrentPosition() > 600 && gear.getCurrentPosition() <= 700){
            gearPower = 0.33;
        }
        else if(gear.getCurrentPosition() > 700 && gear.getCurrentPosition() <= 735){
            gearPower = 0.1;
        }
        else {
            gearPower = 0;
        }
        gear.setPower(gearPower);
    }

    public void whiteScoringGear() {
                gear.setTargetPosition(875); //300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               if(gear.getCurrentPosition() <= 500) {
                    gearPower = 1;
                } else if(gear.getCurrentPosition() > 500 && gear.getCurrentPosition() <= 600){
                    gearPower = 0.5;
                }
                else if(gear.getCurrentPosition() > 600 && gear.getCurrentPosition() <= 700){
                    gearPower = 0.4;
                }
                else if(gear.getCurrentPosition() > 700 && gear.getCurrentPosition() <= 800){
                    gearPower = 0.33;
                }
                else if(gear.getCurrentPosition() > 800 && gear.getCurrentPosition() <= 875){
                    gearPower = 0.1;
                }
                else {
                    gearPower = 0;
                }
                gear.setPower(gearPower);
            }

    //------------------------------Start Position-------------------------------//

    //------------------------------End Position-------------------------------//
    public void scoringGearDown() {

                gear.setTargetPosition(50);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    public void whiteScoringGearDown() {
        gear.setTargetPosition(50);
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
    public void white54() {
        gear.setPower(0);
        wheelServo.setPosition(0.228); //higher # = claw pos higher //was .233
    }

    public void white32() {
        gear.setPower(0);
        wheelServo.setPosition(0.133); //.133
    }

    public void purple() {
        gear.setPower(0);
        wheelServo.setPosition(0.15); //.133
    }

    public void wheelServo_Deactivated() {
        wheelServo.setPosition(0);
    }
}