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

@Autonomous(name = "Red Close 2+2+2 (Cycle 2)", group = "Red")
public class Red_Cycle_Two_Close_Two_Two_Two extends OpMode {

    private Timer pathTimer, actionTimer, opmodeTimer, scanTimer, liftTimer;
    private String navigation;
    public ClawSubsystem claw;
    public GearSubsystem gear;
    public LiftSubsystem lift;
    public PresetSubsystem presets;
    private HuskyLens huskyLens;



    //Spike mark locations
    private Pose LeftSpikeMark = new Pose(39.4, 45.25, Math.toRadians(90)); //51
    private Pose MiddleSpikeMark = new Pose(45.8, 34.1, Math.toRadians(90));
    private Pose RightSpikeMark = new Pose(39.4, 26.3, Math.toRadians(90));

    //Backdrop zone locations
    private Pose LeftBackdrop = new Pose(39, 8.25, Math.toRadians(90)); //117+1+3+0.5
    private Pose MiddleBackdrop = new Pose(35, 8.25, Math.toRadians(90));
    private Pose RightBackdrop = new Pose(26, 8.25, Math.toRadians(90));
    private Pose WhiteBackdrop = new Pose(32, 6.25, Math.toRadians(90));
    private Pose WhiteBackdrop2 = new Pose(32, 6.25, Math.toRadians(90));
    private Pose Zone1WhiteBackdrop = new Pose(35, 8, Math.toRadians(90));
    private Pose Zone1WhiteBackdrop2 = new Pose(26, 7.5, Math.toRadians(90));
    private Pose WhiteBackstage = new Pose(12.75, 8.25, Math.toRadians(90));


    //Through Truss
    private Pose TopTruss = new Pose(12.75, 42, Math.toRadians(90)); //22
    private Pose BottomTruss = new Pose(12.75, 96, Math.toRadians(90));

    // white pixel stack locations
    private Pose Stack = new Pose(38.25, 117, Math.toRadians(90)); //47
    private Pose Stack2 = new Pose(38.25, 116.25, Math.toRadians(90)); //47
    private Pose StackZone1 = new Pose(33.5, 117, Math.toRadians(90)); //47
    private Pose Stack2Zone1 = new Pose(33, 116, Math.toRadians(90)); //47

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(8.5, 60, Math.toRadians(0));

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop, scoreSpikeMarkChosen;
    private PathChain cycleStackTo, cycleStackTo2, cycleStackBack, cycleStackBack2, cycleStackBack2Zone1, cycleStackToBezier, cycleStackBackZone1, cycleStackBackBezier, cycleStackToZone1, cycleStackTo2Zone1;

    private int pathState, actionState, clawState, gearState, liftState;


    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(LeftSpikeMark.getX(), LeftSpikeMark.getY(), Math.toRadians(90));
                initialBackdropGoalPose = new Pose(LeftBackdrop.getX(), LeftBackdrop.getY(), Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(WhiteBackdrop.getX(), WhiteBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5,35,Point.CARTESIAN), new Point(31.2,29.2,Point.CARTESIAN), new Point(LeftSpikeMark)));

                break;
            case "middle":
                spikeMarkGoalPose = new Pose(MiddleSpikeMark.getX(), MiddleSpikeMark.getY()+3, Math.toRadians(90));
                initialBackdropGoalPose = new Pose(MiddleBackdrop.getX(), MiddleBackdrop.getY(),Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(MiddleBackdrop.getX(), MiddleBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(startPose.getX(),48,Point.CARTESIAN), new Point(MiddleSpikeMark.getX(),35,Point.CARTESIAN), new Point(MiddleSpikeMark)));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(RightSpikeMark.getX(), RightSpikeMark.getY(), Math.toRadians(90));
                initialBackdropGoalPose = new Pose(RightBackdrop.getX(), RightBackdrop.getY(), Math.toRadians(90));
                firstCycleBackdropGoalPose = new Pose(RightBackdrop.getX(), RightBackdrop.getY(), Math.toRadians(90));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(startPose.getX(),36,Point.CARTESIAN), new Point(RightSpikeMark.getX(),24,Point.CARTESIAN), new Point(RightSpikeMark)));
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

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(TopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12.75, 144-24, Point.CARTESIAN), new Point(36,95,Point.CARTESIAN), new Point(Stack)))
                // .addPath(new BezierLine(new Point(BottomTruss), new Point(Stack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackTo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(WhiteBackstage), new Point(TopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12.75, 144-24, Point.CARTESIAN), new Point(36,95,Point.CARTESIAN), new Point(Stack2)))
                //.addPath(new BezierLine(new Point(BottomTruss), new Point(Stack2)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackToZone1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(TopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12, 144-24, Point.CARTESIAN), new Point(28.5,95,Point.CARTESIAN), new Point(StackZone1)))
                // .addPath(new BezierLine(new Point(BottomTruss), new Point(Stack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackTo2Zone1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(WhiteBackstage), new Point(TopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12, 144-24, Point.CARTESIAN), new Point(28.5,95,Point.CARTESIAN), new Point(Stack2Zone1)))
                //.addPath(new BezierLine(new Point(BottomTruss), new Point(Stack2)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Stack), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(WhiteBackstage)))
                .setConstantHeadingInterpolation(WhiteBackstage.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBackZone1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StackZone1), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(WhiteBackstage)))
                .setConstantHeadingInterpolation(WhiteBackstage.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Stack2), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(WhiteBackdrop2)))
                .setConstantHeadingInterpolation(WhiteBackdrop2.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack2Zone1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StackZone1), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(Zone1WhiteBackdrop2)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackToBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(30+14,91.6, Point.CARTESIAN), new Point(13+14, 130.8, Point.CARTESIAN), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(20.5+14,10, Point.CARTESIAN), new Point(42+14,35, Point.CARTESIAN), new Point(Stack)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBackBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(Stack), new Point(42+14,35, Point.CARTESIAN), new Point(20.5+14,10, Point.CARTESIAN), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(13+14, 130.8, Point.CARTESIAN), new Point(30+14,91.6, Point.CARTESIAN), new Point(WhiteBackdrop)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
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
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    claw.openLClaw();
                    setPathState(12);
                }
                break;
            case 12:
                follower.setMaxPower(0.5);
                follower.followPath(initialScoreOnBackdrop);
                setPathState(20102019);
                break;
            case 20102019:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    claw.closeLClaw();
                    setActionState(1);
                    setPathState(13);
                }
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
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setActionState(2);
                    setPathState(15);
                }
                break;
            case 15:
                follower.setMaxPower(0.7);
                if (navigation == "left") {
                    follower.followPath(cycleStackToZone1, true);
                } else {
                    follower.followPath(cycleStackTo, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setActionState(3);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 5.5) {
                    claw.closeClaws();
                    setPathState(17);
                }
                break;
            case 17:
                initialBackdropGoalPose = WhiteBackstage;
                follower.setMaxPower(0.65);
                if (navigation == "left") {
                    follower.followPath(cycleStackBackZone1, true);
                } else {
                    follower.followPath(cycleStackBack, true);
                }
                setPathState(18);
                break;
            case 18:
                if (navigation == "left") {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        setActionState(8);
                        setPathState(100201);
                    }
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        setActionState(8);
                        setPathState(100201);
                    }
                }
                break;
            case 100201:
                if(pathTimer.getElapsedTimeSeconds() > 1.5)
                {
                    follower.setMaxPower(0.7);
                    if(navigation == "left") {
                        follower.holdPoint(new BezierPoint(new Point(WhiteBackstage)), WhiteBackstage.getHeading());
                    } else {
                        follower.holdPoint(new BezierPoint(new Point(WhiteBackstage)), WhiteBackstage.getHeading());
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.35) {
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
                    follower.setMaxPower(0.7);
                    if(navigation == "left") {
                        follower.followPath(cycleStackTo2Zone1, true);
                    }
                    else {
                        follower.followPath(cycleStackTo2, true);
                    }
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
                    initialBackdropGoalPose = WhiteBackdrop;
                    follower.setMaxPower(0.75);
                    if(navigation == "left") {
                        follower.followPath(cycleStackBack2Zone1, true);
                    }
                    else {
                        follower.followPath(cycleStackBack, true);
                    }
                    setActionState(3);
                    setPathState(22);
                }
                break;
            case 22:
                if (navigation == "left") {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        setActionState(8);
                        setPathState(100204);
                    }
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        setActionState(4);
                        setPathState(100204);
                    }
                }
                break;
            case 100204:
                if(pathTimer.getElapsedTimeSeconds() > 1.5)
                {
                    if(navigation == "left") {
                        follower.holdPoint(new BezierPoint(new Point(Zone1WhiteBackdrop2)), Zone1WhiteBackdrop2.getHeading());
                    } else {
                        follower.holdPoint(new BezierPoint(new Point(WhiteBackdrop2)), WhiteBackdrop2.getHeading());
                    }
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
            case 8:
                setGearState(7);
                setClawState(4);
                setActionState(-1);
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
                gear.purple();
                setGearState(-1);
                break;
            case 1:
                gear.scoringGearRED();
                gear.wheelServo_Deactivated();
                setGearState(-1);

                break;
            case 2:
                gear.scoringGearDown();
                setGearState(-1);

                break;
            case 3:
                gear.whiteScoringGear();
                setGearState(-1);

                break;
            case 4:
                gear.whiteScoringGearDown();
                setGearState(-1);
                break;
            case 5:
                gear.stopGear();
                setGearState(-1);
                break;
            case 6:
                gear.wheelServo_Deactivated();
                gear.whiteScoringGearDown();
                setGearState(-1);
                break;
            case 7:
                gear.scoringGearZone1();
                setGearState(-1);
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
                if((!lift.lift.isBusy()) && (lift.lift.getCurrentPosition() > 1100)) {
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
            case 5:
                lift.liftExtend_WhiteScoring();
                if((!lift.lift.isBusy()) && (lift.lift.getCurrentPosition() > 1300)) {
                    lift.lift.setPower(0);
                    setLiftState(-1);
                }
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
            if (blocks[i].x > 100 && blocks[i].x < 210 && blocks[i].id == 1) {
                navigation = "middle";
            }
            if (blocks[i].x >= 210 && blocks[i].id == 1) {
                navigation = "right";
            }
        }
        telemetry.addData("Navigation", navigation);

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
