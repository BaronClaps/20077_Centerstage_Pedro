package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.GearSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.PresetSubsystem;

@Autonomous(name = "Blue Close 2+0", group = "Blue")
public class Blue_Close_Two_Zero extends OpMode {

    private Timer pathTimer, actionTimer, opmodeTimer, scanTimer, liftTimer;
    private String navigation;
    public ClawSubsystem claw;
    public GearSubsystem gear;
    public LiftSubsystem lift;
    public PresetSubsystem presets;
    private HuskyLens huskyLens;



    //Spike mark locations
    private Pose blueLeftSpikeMark = new Pose(-36+72+16, 32+72, Math.toRadians(270)); //51
    private Pose blueMiddleSpikeMark = new Pose(-30+72+15, 22+67.5, Math.toRadians(270));
    private Pose blueRightSpikeMark = new Pose(-36+72+16, 8+72, Math.toRadians(270));

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(30+12+2, 117+1+3+0.5, Math.toRadians(270)); //41
    private Pose blueMiddleBackdrop = new Pose(30+12+10.5, 117+1+3+0.5, Math.toRadians(270));
    private Pose blueRightBackdrop = new Pose(30+12+16.5, 117+1+3+0.5, Math.toRadians(270));
    private Pose park = new Pose(22+8, 112, Math.toRadians(270));

    private Pose spikeMarkGoalPose, initialBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(8.5, 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop, parkingPath;

    private int pathState, actionState, clawState, liftState;


    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSpikeMark.getX(), blueLeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueMiddleSpikeMark.getX(), blueMiddleSpikeMark.getY()+3, Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(),Math.toRadians(270));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueRightSpikeMark.getX(), blueRightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(8.5,80.5,Point.CARTESIAN), new Point(48,135,Point.CARTESIAN), new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);

        parkingPath = new Path(new BezierLine(new Point(initialBackdropGoalPose), new Point(park)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                gear.gearReset();
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    claw.openLClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(1);
                    follower.setMaxPower(0.5);
                    follower.followPath(initialScoreOnBackdrop);
                    claw.closeLClaw();
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(initialScoreOnBackdrop.getLastControlPoint()), initialBackdropGoalPose.getHeading());
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0) {
                    claw.openRClaw();
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(2);
                    setPathState(15);
                }
                break;
            case 15:
                follower.setMaxPower(0.7);
                follower.followPath(parkingPath, true);
                claw.closeClaws();
                setPathState(16);
                break;
            case 16:
                if(opmodeTimer.getElapsedTimeSeconds() > 30) {
                    follower.breakFollowing();
                    setPathState(-1);
                }
                break;
        }
    }

    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                gear.gearTarget(130);
                setClawState(0);
                setLiftState(0);
                setActionState(-1);
                break;
            case 1:
                gear.gearTarget(775);
                setClawState(1);
                setLiftState(1);
                setActionState(-1);
                break;
            case 2:
                gear.gearTarget(300);
                setLiftState(2);
                setClawState(2);
                setActionState(-1);
                break;
        }
    }

    public void clawUpdate() {
        switch (clawState) {
            case 0:
                claw.groundClaw();
                claw.closeClaws();
                setClawState(-1);
                break;
            case 1:
                claw.scoringClaw();
                setClawState(-1);
                break;
            case 2:
                claw.groundClaw();
                claw.openClaws();
                setClawState(-1);
                break;
        }
    }

    /*public void gearUpdate() {
        switch (gearState) {
            case 10:
                //gear.startGear();
                gear.resetGear();
                gear.gearTarget(-500);
                break;
            case 11:
                gear.resetGear();
                gear.scoringGear();
                break;
            case 12:
                gear.groundGear();
                break;

        }
    }*/

    public void liftUpdate() {
        switch (liftState) {
            case 0:
                lift.lift.setPower(0);
                setLiftState(-1);
                break;
            case 1:
                lift.liftExtend_Scoring();
                if((!lift.lift.isBusy()) && (lift.lift.getCurrentPosition() > 950)) {
                    lift.lift.setPower(0);
                    setLiftState(-1);
                }
                break;
            case 2:
                lift.liftRetract_Scoring();
                if((!lift.lift.isBusy()) && (lift.lift.getCurrentPosition() < 25)) {
                    lift.lift.setPower(0);
                    setLiftState(-1);
                }
                break;

        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    /*public void setGearState(int gState) {
        gearState = gState;
    }*/

    public void setLiftState(int lState) {
        liftState = lState;
        liftTimer.resetTimer();
    }



    @Override
    public void loop() {
        follower.update();


        autonomousPathUpdate();
        autonomousActionUpdate();
        clawUpdate();
        liftUpdate();
        //lift.liftPIDUpdate();
        gear.gearPIDUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("gear pos var", gear.gearPos);
        telemetry.addData("gear pos", gear.gear.getCurrentPosition());
        telemetry.addData("gear tar", gear.gearTarget);

        // telemetry.addData("lift pos var", lift.armPos);
        telemetry.addData("lift pos", lift.lift.getCurrentPosition());
        //telemetry.addData("lift tar", lift.liftTarget);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        liftTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        gear = new GearSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        presets = new PresetSubsystem(claw, lift, gear);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        telemetry.addData("Init", "Finished");
        telemetry.update();

        claw.closeClaws();
        claw.startClaw();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        claw.closeClaws();
        navigation = "left";
        /*Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());// this gives you the data
            telemetry.addData("location?", blocks[i].x);// this gives you just x
            //----------------------------1----------------------------\\
            if (blocks[i].x < 100 && blocks[i].id == 2 && blocks[i].y < 200) {
                navigation = "left";
            }
            if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200) {
                navigation = "middle";
            }
            if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200) {
                navigation = "right";
            }
        }*/
        //telemetry.update();
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
        setActionState(0);
    }

    @Override
    public void stop() {
    }
}
