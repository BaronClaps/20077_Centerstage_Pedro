package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class ClawSubsystem {

    private Servo pivot, clawL, clawR;
    double closedL = 0.32; //33
    double closedR = 0.38; //37
    double openL = 0.47;//.42
    double openR = 0.23;//.28
    double openL1 = 0.44;//.42
    double openR1 = 0.35;//.28
    double openL2 = 0.41;//.42
    double openR2 = 0.33;//.28
    double startClaw = 0.174;
    double groundClaw = 0.835; //.815
    double scoringClaw = 0.25;
    double white54 = 0.865;
    double white32 = 0.831;
    double white1 = 0.835;
    double whiteScoringClaw = 0.795; //.78

    public ClawSubsystem(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
    }

    //------------------------------Close Claws------------------------------//

    public void closeLClaw() {
        clawL.setPosition(closedL);
    }

    public void closeRClaw() {
        clawR.setPosition(closedR);
    }

    public void closeClaws() {
        clawL.setPosition(closedL);
        clawR.setPosition(closedR);
    }

    //------------------------------Open Claws------------------------------//
    public void openLClaw() {
        clawL.setPosition(openL);
    }


    public void openRClaw() {
        clawR.setPosition(openR);
    }

    public void openClaws() {
        clawL.setPosition(openL);
        clawR.setPosition(openR);
    }

    public void openClaws1() {
        clawL.setPosition(openL1);
        clawR.setPosition(openR1);
    }

    public void openClaws2() {
        clawL.setPosition(openL2);
        clawR.setPosition(openR2);
    }

    //------------------------------Claw Rotate------------------------------//
    public void startClaw() {
        pivot.setPosition(startClaw);
    }

    public void groundClaw() {
        pivot.setPosition(groundClaw);
    }

    public void scoringClaw() {
        pivot.setPosition(scoringClaw);
    }

    public void white54() {
        pivot.setPosition(white54);
    }

    public void white32() {
        pivot.setPosition(white32);
    }

    public void white1() {
        pivot.setPosition(white1);
    }


    public void whiteScoringClaw() {
        pivot.setPosition(whiteScoringClaw);
    }

}