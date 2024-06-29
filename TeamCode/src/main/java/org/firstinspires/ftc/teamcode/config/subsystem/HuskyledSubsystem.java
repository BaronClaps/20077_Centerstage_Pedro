package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;


public class HuskyledSubsystem {

    public ServoImplEx huskyled;
    double On = 0;
    double Off = .75;

    public HuskyledSubsystem(HardwareMap hardwareMap) {
        huskyled = hardwareMap.get(ServoImplEx.class, "huskyled");
        huskyled.setPwmRange(new PwmControl.PwmRange(0,4000,5000));
        huskyled.setPosition(0);
    }


    //------------------------------Open-Led------------------------------//
    public void onLed() {huskyled.setPosition(On);}
    public void offLed() {huskyled.setPosition(Off);}

}