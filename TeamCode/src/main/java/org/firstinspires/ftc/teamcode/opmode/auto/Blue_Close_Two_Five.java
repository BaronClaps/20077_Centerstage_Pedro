package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.FollowPathAction;
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
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.AutoActionScheduler;

@Autonomous(name = "Blue Close 2+5", group = "Blue")
public class Blue_Close_Two_Five extends OpMode {

    private Timer pathTimer, actionTimer, opmodeTimer, scanTimer;
    private String navigation;
    public ClawSubsystem claw;
    public GearSubsystem gear;
    public LiftSubsystem lift;
    public PresetSubsystem presets;



    //Spike mark locations
    private Pose blueLeftSpikeMark = new Pose(-36+72+12, 32+72+1, Math.toRadians(270)); //51
    private Pose blueMiddleSpikeMark = new Pose(-30+72+12, 22+72, Math.toRadians(270));
    private Pose blueRightSpikeMark = new Pose(-36+72+12, 8+72, Math.toRadians(270));

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(30+12+2, 117-1.5, Math.toRadians(270)); //41
    private Pose blueMiddleBackdrop = new Pose(30+12+6, 51.5+72, Math.toRadians(270));
    private Pose blueRightBackdrop = new Pose(30+12+12, 51.5+72, Math.toRadians(270));
    private Pose blueWhiteBackdrop = new Pose(30+12, 117-1.5, Math.toRadians(270));

    //Through Truss
    private Pose blueTopTruss = new Pose(12+14, 84, Math.toRadians(270)); //22
    private Pose blueBottomTruss = new Pose(12+14, 36, Math.toRadians(270));

    // white pixel stack locations
    private Pose blueLeftStack = new Pose(-36+72+14+12, -37+72, Math.toRadians(270));
    private Pose blueMiddleStack = new Pose(-36+72+14+6, -37+72, Math.toRadians(270));
    private Pose blueRightStack = new Pose(36+12, 12, Math.toRadians(270)); //47

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(8.5, 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain cycleStackTo, cycleStackBack, cycleStackToBezier, cycleStackBackBezier;

    private int pathState, actionState, clawState, gearState, liftState;


    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSpikeMark.getX(), blueLeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));

                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueMiddleSpikeMark.getX(), blueMiddleSpikeMark.getY()+3, Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(),Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(), Math.toRadians(270));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueRightSpikeMark.getX(), blueRightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
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

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueBottomTruss), new Point(blueRightStack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(blueRightStack), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueBottomTruss), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueWhiteBackdrop)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackToBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(30+14,91.6, Point.CARTESIAN), new Point(13+14, 130.8, Point.CARTESIAN), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(blueBottomTruss), new Point(20.5+14,10, Point.CARTESIAN), new Point(42+14,35, Point.CARTESIAN), new Point(blueRightStack)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBackBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(blueRightStack), new Point(42+14,35, Point.CARTESIAN), new Point(20.5+14,10, Point.CARTESIAN), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(blueBottomTruss), new Point(13+14, 130.8, Point.CARTESIAN), new Point(30+14,91.6, Point.CARTESIAN), new Point(blueWhiteBackdrop)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
             if (pathTimer.getElapsedTimeSeconds() > 3) {
                    claw.openLClaw();
                }
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    setActionState(1);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(initialScoreOnBackdrop);
                }
                claw.closeLClaw();
                setPathState(13);
                break;
            case 13:
                follower.holdPoint(new BezierPoint(initialScoreOnBackdrop.getLastControlPoint()), initialBackdropGoalPose.getHeading());
                setPathState(14);

                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    claw.openRClaw();
                    setActionState(2);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.85);
                    follower.followPath(cycleStackTo, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    initialBackdropGoalPose = blueWhiteBackdrop;
                    follower.followPath(cycleStackBack, true);
                    setPathState(18);
                }
                break;
            case 18:
            if (pathTimer.getElapsedTimeSeconds() > 2) {

                }
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackTo, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackTo, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(26);
                }
                break;
            case 26:
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
                claw.closeClaws();
                gear.gearTarget(0);
                //lift.liftTarget(10);
                claw.groundClaw();
                setActionState(-1);
                break;
            case 1:
                gear.gearTarget(750);
                /*if (gear.gearPos < 760 && gear.gearPos > 740) {
                    lift.liftTarget(500);
                    setClawState(1);
                }*/
                setActionState(-1);
                break;
            case 2:
                gear.gearTarget(0);
                //lift.liftTarget(0);
                claw.groundClaw();
                claw.openClaws();
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

    /*public void liftUpdate() {
        switch (liftState) {
            case 10:
                lift.stopLift();
                lift.resetLift();
                break;
            case 11:
                lift.liftExtend_Scoring();
                break;
            case 12:
                lift.liftRetract_Scoring();
                break;

        }
    }*/



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

    /*public void setLiftState(int lState) {
        liftState = lState;
    }*/



    @Override
    public void loop() {
        follower.update();
        

        autonomousPathUpdate();
        autonomousActionUpdate();
        clawUpdate();
        //lift.liftPIDUpdate();
        gear.gearPIDUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("gear pos var", gear.gearPos);
        telemetry.addData("gear pos", gear.gear.getCurrentPosition());
        telemetry.addData("gear tar", gear.gearTarget);
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
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        gear = new GearSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        presets = new PresetSubsystem(claw, lift, gear);

        claw.closeClaws();

        scanTimer.resetTimer();

    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            scanTimer.resetTimer(); }
    }

    @Override
    public void start() {
        navigation = "left";
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