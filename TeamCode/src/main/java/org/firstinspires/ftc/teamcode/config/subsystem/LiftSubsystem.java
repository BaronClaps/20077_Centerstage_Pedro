package org.firstinspires.ftc.teamcode.config.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
    public DcMotorEx lift;
    /*
    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0.0005;
    public static double f = 0.325;
    public int liftTarget = 0;
    public static double ticks_in_degree = 2.09222222;
    public int armPos;*/

    public LiftSubsystem(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /*

    public void liftTarget(int ltarget){
        liftTarget = ltarget;
    }

    public void liftPIDUpdate() {
        controller = new PIDController(p,i,d);
        int armPos = lift.getCurrentPosition();
        double pid = controller.calculate(armPos, liftTarget);
        double ff = Math.cos(Math.toRadians(liftTarget / ticks_in_degree)) * f;

        double power = pid + ff;
        lift.setPower(power);
    }*/

    //------------------------------ Lift Extend ------------------------------//
    public void liftExtend_Scoring() {
                //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(1000);//500
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    public void liftExtend_WhiteScoring() {
                lift.setTargetPosition(1200); //-800
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    //------------------------------ Lift Retract ------------------------------//
    public void liftRetract_Scoring() {
                //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(0);//-450
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    public void liftRetract_WhiteScoring() {
                lift.setTargetPosition(0); //800
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    //------------------------------ Wait for Lift -------------------------------//
    /*public Action waitForLift() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return lift.isBusy();
            }
        };
    }//
*/
    //------------------------------ Stop Lift -------------------------------//
    public void stopLift() {
        lift.setPower(0);
    }

    public void resetLift() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

