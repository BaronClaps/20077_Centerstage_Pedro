package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
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

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Close 2+4", group = "Red")
public class Red_Close_Two_Four extends OpMode {

    private Timer pathTimer, actionTimer, opmodeTimer, scanTimer, liftTimer;
    private String navigation;
    public ClawSubsystem claw;
    public GearSubsystem gear;
    public LiftSubsystem lift;
    public PresetSubsystem presets;
    private HuskyLens huskyLens;



    //Spike mark locations
    private Pose blueLeftSpikeMark = new Pose(39.4, 45, Math.toRadians(90)); //51
    private Pose blueMiddleSpikeMark = new Pose(48, 34.1, Math.toRadians(90));
    private Pose blueRightSpikeMark = new Pose(39.4, 45.5, Math.toRadians(90));

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(40, 9, Math.toRadians(90)); //117+1+3+0.5
    private Pose blueMiddleBackdrop = new Pose(34.7, 15.75, Math.toRadians(90));
    private Pose blueRightBackdrop = new Pose(28.6, 20.75, Math.toRadians(90));
    private Pose blueWhiteBackdrop = new Pose(31, 21.5, Math.toRadians(90));
    private Pose blueWhiteBackdrop2 = new Pose(31, 21.5, Math.toRadians(90));


    //Through Truss
    private Pose blueTopTruss = new Pose(10, 60, Math.toRadians(90)); //22
    private Pose blueBottomTruss = new Pose(10, 108, Math.toRadians(90));

    // white pixel stack locations
    private Pose blueLeftStack = new Pose(-(62-72), 35, Math.toRadians(90));
    private Pose blueMiddleStack = new Pose(-(56-72), 35, Math.toRadians(90));
    private Pose blueRightStack = new Pose(37.6, 117.5, Math.toRadians(90)); //47
    private Pose blueRightStack2 = new Pose(37.6, 117.5, Math.toRadians(90)); //47

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(8.5, 60, Math.toRadians(0));

    private Follower follower;

    private Point blueWhiteBackdropPoint;

    private Path scoreSpikeMark, initialScoreOnBackdrop, scoreSpikeMarkChosen;
    private PathChain cycleStackTo, cycleStackTo2, cycleStackBack, cycleStackBack2, cycleStackToBezier, cycleStackBackBezier;

    private int pathState, actionState, clawState, gearState, liftState;


    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSpikeMark.getX(), blueLeftSpikeMark.getY(), Math.toRadians(90));
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(blueWhiteBackdrop.getX(), blueWhiteBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5,35,Point.CARTESIAN), new Point(31.2,29.2,Point.CARTESIAN), new Point(blueLeftSpikeMark)));

                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueMiddleSpikeMark.getX(), blueMiddleSpikeMark.getY()+3, Math.toRadians(90));
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(),Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(-(8.5-72),12+72+5,Point.CARTESIAN), new Point(-(-30+15),22+67.5+20+5,Point.CARTESIAN), new Point(blueMiddleSpikeMark)));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueRightSpikeMark.getX(), blueRightSpikeMark.getY(), Math.toRadians(90));
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(-(8.5-72),12+72+5,Point.CARTESIAN), new Point(-(-36+16),80+17+4,Point.CARTESIAN), new Point(blueRightSpikeMark)));
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = scoreSpikeMarkChosen;
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);

        blueWhiteBackdropPoint = new Point(blueWhiteBackdrop);

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(blueBottomTruss), new Point(12+13+1, 12.5, Point.CARTESIAN), new Point(36+12+1,36,Point.CARTESIAN), new Point(blueRightStack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackTo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(blueBottomTruss), new Point(12+13+1, 12.5, Point.CARTESIAN), new Point(36+12+1,36,Point.CARTESIAN), new Point(blueRightStack2)))
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

        cycleStackBack2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(blueRightStack2), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueBottomTruss), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueWhiteBackdrop2)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop2.getHeading())
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
                if (pathTimer.getElapsedTimeSeconds() > 2.2) {
                    gear.purple();
                    claw.openLClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(1);
                    follower.setMaxPower(0.5);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(13);
                }
                break;
            case 13:
                claw.closeLClaw();
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
                follower.setMaxPower(0.75);
                follower.followPath(cycleStackTo, true);

                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(3);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 6) {
                    claw.closeClaws();
                    setPathState(17);
                }
                break;
            case 17:
                initialBackdropGoalPose = blueWhiteBackdrop;
                follower.setMaxPower(0.65);
                follower.followPath(cycleStackBack, true);
                setPathState(18);
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setActionState(4);
                    setPathState(100201);
                }
                break;
            case 100201:
                if(pathTimer.getElapsedTimeSeconds() > 1.5)
                {
                    follower.holdPoint(new BezierPoint(new Point(blueWhiteBackdrop)), blueWhiteBackdrop.getHeading());
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.35) {
                    claw.openClaws2();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.55) {
                    claw.openClaws1();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.75) {
                    claw.openClaws();
                    setPathState(100203);
                }
                break;
            case 100203:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(5);
                    setPathState(19);
                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.setMaxPower(0.75);
                    follower.followPath(cycleStackTo2, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setActionState(6);
                    setPathState(201101);
                }
            case 201101:
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    claw.closeClaws();
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    initialBackdropGoalPose = blueWhiteBackdrop;
                    follower.setMaxPower(0.65);
                    follower.followPath(cycleStackBack, true);
                    setActionState(3);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setActionState(4);
                    setPathState(100204);
                }
                break;
            case 100204:
                if(pathTimer.getElapsedTimeSeconds() > 1.5)
                {
                    follower.holdPoint(new BezierPoint(new Point(blueWhiteBackdrop)), blueWhiteBackdrop.getHeading());
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.35) {
                    claw.openClaws2();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.55) {
                    claw.openClaws1();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.75) {
                    claw.openClaws();
                }
                break;
            case 100202:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    setActionState(5);
                    setPathState(26);
                }
           /* case 23:
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
                break;*/
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
            case 0: //Brings arm to ground
                setClawState(0);
                setGearState(0);
                setLiftState(0);
                setActionState(-1);
                break;
            case 1: //Sets arm to YELLOW scoring position
                setGearState(1);
                setClawState(1);
                setLiftState(1);
                setActionState(-1);
                break;
            case 2: // Sets arm down after scoring YELLOW
                setGearState(2);
                setLiftState(2);
                setClawState(2);
                setActionState(-1);
                break;
            case 3: //First cycle white stack grab
                gear.stopGear();
                gear.white54();
                setClawState(3);
                setActionState(-1);
                break;
            case 4: //Sets arm to WHITE Scoring Position
                setGearState(3);
                setClawState(4);
                setLiftState(3);
                setActionState(-1);
                break;
            case 5: //Sets arm down after scoring WHITE
                setGearState(4);
                setLiftState(4);
                setClawState(2);
                setActionState(-1);
                break;
            case 6: //Second cycle white stack grab
                gear.white32();
                setClawState(5);
                setActionState(-1);
                break;
            case 7: //Sets arm down after scoring WHITE FINAL
                setGearState(4);
                setLiftState(4);
                setClawState(0);
                setActionState(-1);
                break;
        }
    }

    public void clawUpdate() {
        switch (clawState) {
            case 0: //Sets claw to ground
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
            case 3:
                claw.white54();
                setClawState(-1);
                break;
            case 5:
                claw.white32();
                setClawState(-1);
                break;
            case 4:
                claw.whiteScoringClaw();
                setClawState(-1);
                break;
        }
    }

    public void gearUpdate() {
        switch (gearState) {
            case 0:
                gear.startGear();
                break;
            case 1:
                gear.scoringGearRED();
                gear.wheelServo_Deactivated();
                break;
            case 2:
                gear.scoringGearDown();
                break;
            case 3:
                gear.whiteScoringGear();
                break;
            case 4:
                gear.whiteScoringGearDown();
                break;
            case 5:
                gear.stopGear();
                break;
            case 6:
                gear.wheelServo_Deactivated();
                gear.whiteScoringGearDown();
                break;
        }
    }

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
            case 3:
                lift.liftExtend_WhiteScoring();
                if((!lift.lift.isBusy()) && (lift.lift.getCurrentPosition() > 1000)) {
                    lift.lift.setPower(0);
                    setLiftState(-1);
                }
                break;
            case 4:
                lift.liftRetract_WhiteScoring();
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

    public void setGearState(int gState) {
        gearState = gState;
    }

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
        gearUpdate();

        //Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

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

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        gear = new GearSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        presets = new PresetSubsystem(claw, lift, gear);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        gear.wheelServo_Deactivated();
        claw.closeClaws();
        claw.startClaw();
    }

    @Override
    public void init_loop() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            //----------------------------1----------------------------\\
            if (blocks[i].x <= 100 && blocks[i].id == 1) {
                navigation = "left";
            }
            if (blocks[i].x > 100 && blocks[i].x <= 270 && blocks[i].id == 1) {
                navigation = "middle";
            }
            if (blocks[i].x > 270 && blocks[i].id == 1) {
                navigation = "right";
            }
        }

        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("Init", "Finished");
        }
    }

    @Override
    public void start() {
        claw.closeClaws();
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
