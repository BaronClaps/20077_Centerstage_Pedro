package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class PIDF_Gear extends OpMode {
    private PIDController controller;
    public static double p = 0.0215, i = 0, d = 0.0005;
    public static double f = 0.05;

    public static int target = 0;

    private final double ticks_in_degree = 7.7395; // (pulses per revolution) / (degrees in revolution)

    int armPos;

    private DcMotorEx gear;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gear =  hardwareMap.get(DcMotorEx.class, "gear");
        gear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int armPos = gear.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        power = Range.clip(power, -0.7, 0.7);

        gear.setPower(power);


        if (gamepad2.left_stick_y > 0){
            target += 2;
        } else if (gamepad2.left_stick_y < 0){
            target -= 2;
        }


        telemetry.addData("pos ", armPos);
        telemetry.addData("target", target);
    }
}
