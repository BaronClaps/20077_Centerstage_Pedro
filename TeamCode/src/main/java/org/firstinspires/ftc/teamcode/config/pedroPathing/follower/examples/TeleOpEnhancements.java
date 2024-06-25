package org.firstinspires.ftc.teamcode.config.pedroPathing.follower.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.MathFunctions;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */

@TeleOp(name = "Pedro Pathing TeleOp Enhancements", group = "Test")
public class TeleOpEnhancements extends OpMode {
    private Follower follower;

    private DcMotorEx lF;
    private DcMotorEx lB;
    private DcMotorEx rF;
    private DcMotorEx rB;

    private Vector driveVector;
    private Vector headingVector;

    private Pose startingPose = new Pose(8.5, 60, Math.toRadians(0));
    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);

        lF = hardwareMap.get(DcMotorEx.class, "lF");
        lB = hardwareMap.get(DcMotorEx.class, "lB");
        rB = hardwareMap.get(DcMotorEx.class, "rB");
        rF = hardwareMap.get(DcMotorEx.class, "rF");

        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lF.setDirection(DcMotorEx.Direction.REVERSE);
        lB.setDirection(DcMotorEx.Direction.REVERSE);

        driveVector = new Vector();
        headingVector = new Vector();

        follower.setPose(startingPose);
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
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

        if (gamepad1.left_bumper) { //speed for going across field
            lF.setPower(lFPower);
            rF.setPower(rFPower);
            lB.setPower(lBPower);
            rB.setPower(rBPower); }
        if (gamepad1.right_bumper) { //slow for corrections
            lF.setPower(0.2 * lFPower);
            rF.setPower(0.2 * rFPower);
            lB.setPower(0.2 * lBPower);
            rB.setPower(0.2 * rBPower); }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        follower.update();
    }
}
