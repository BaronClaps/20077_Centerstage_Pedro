package org.firstinspires.ftc.teamcode.config.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
    private DcMotorEx lift;

    public LiftSubsystem(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //------------------------------ Lift Extend ------------------------------//
    public void liftExtend_Scoring() {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-500);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    public void liftExtend_WhiteScoring() {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(-200); //-800
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    //------------------------------ Lift Retract ------------------------------//
    public void liftRetract_Scoring() {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(500);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
    }

    public void liftRetract_WhiteScoring() {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setTargetPosition(200); //800
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

