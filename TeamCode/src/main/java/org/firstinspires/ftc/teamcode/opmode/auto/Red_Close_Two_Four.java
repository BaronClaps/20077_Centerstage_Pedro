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
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

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
    private Pose redLeftSpikeMark = new Pose(-36+72+16+72, 32+72+1, Math.toRadians(270)); //51
    private Pose redMiddleSpikeMark = new Pose(-30+72+15+48, 22+67.5, Math.toRadians(270));
    private Pose redRightSpikeMark = new Pose(-36+72+16+72, 8+72, Math.toRadians(270));

    //Backdrop zone locations
    private Pose redLeftBackdrop = new Pose(30+12+2+72, 117+1+3+0.5, Math.toRadians(270)); //41
    private Pose redMiddleBackdrop = new Pose(30+12+10.5+72, 117+1+3+0.5, Math.toRadians(270));
    private Pose redRightBackdrop = new Pose(30+12+16.5+72, 117+1+3+0.5, Math.toRadians(270));
    private Pose redWhiteBackdrop = new Pose(30+10+72, 117+1+4.25, Math.toRadians(270));

    //Through Truss
    private Pose redTopTruss = new Pose(12+13+1.5+120, 84, Math.toRadians(270)); //22
    private Pose redBottomTruss = new Pose(12+13+1.5+120, 36, Math.toRadians(270));

    // white pixel stack locations
    private Pose redLeftStack = new Pose(-36+72+14+12+72, -37+72, Math.toRadians(270));
    private Pose redMiddleStack = new Pose(-36+72+14+6+72, -37+72, Math.toRadians(270));
    private Pose redRightStack = new Pose(36+9.75+96, 12+72, Math.toRadians(270)); //47
    private Pose redRightStack2 = new Pose(36+10.25+96, 12.75+72, Math.toRadians(270)); //47

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(8.5+120, 12+72, 180);

    private Follower follower;

    private Point redWhiteBackdropPoint;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain cycleStackTo, cycleStackTo2, cycleStackBack, cycleStackToBezier, cycleStackBackBezier;

    private int pathState, actionState, clawState, gearState, liftState;


    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(redLeftSpikeMark.getX(),redLeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(redLeftBackdrop.getX(), redLeftBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(redWhiteBackdrop.getX(), redWhiteBackdrop.getY(), Math.toRadians(270));

                break;
            case "middle":
                spikeMarkGoalPose = new Pose(redMiddleSpikeMark.getX(), redMiddleSpikeMark.getY()+3, Math.toRadians(270));
                initialBackdropGoalPose = new Pose(redMiddleBackdrop.getX(), redMiddleBackdrop.getY(),Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(redMiddleBackdrop.getX(), redMiddleBackdrop.getY(), Math.toRadians(270));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(redRightSpikeMark.getX(), redRightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(redRightBackdrop.getX(), redRightBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(redRightBackdrop.getX(), redRightBackdrop.getY(), Math.toRadians(270));
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), new Point(8.5+72+120,80.5,Point.CARTESIAN), new Point(48+72+120,135,Point.CARTESIAN), new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);

        redWhiteBackdropPoint = new Point(redWhiteBackdrop);

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(redTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(redTopTruss), new Point(redBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(redBottomTruss), new Point(12+13+1, 12.5, Point.CARTESIAN), new Point(36+12+1,36,Point.CARTESIAN), new Point(redRightStack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackTo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redWhiteBackdrop), new Point(redTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(redTopTruss), new Point(redBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(redBottomTruss), new Point(12+13+1, 12.5, Point.CARTESIAN), new Point(36+12+1,36,Point.CARTESIAN), new Point(redRightStack2)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redRightStack), new Point(redBottomTruss)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(redBottomTruss), new Point(redTopTruss)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(redTopTruss), new Point(redWhiteBackdrop)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackToBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(30+14,91.6, Point.CARTESIAN), new Point(13+14, 130.8, Point.CARTESIAN), new Point(redBottomTruss)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(redBottomTruss), new Point(20.5+14,10, Point.CARTESIAN), new Point(42+14,35, Point.CARTESIAN), new Point(redRightStack)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBackBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(redRightStack), new Point(42+14,35, Point.CARTESIAN), new Point(20.5+14,10, Point.CARTESIAN), new Point(redBottomTruss)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(redBottomTruss), new Point(13+14, 130.8, Point.CARTESIAN), new Point(30+14,91.6, Point.CARTESIAN), new Point(redWhiteBackdrop)))
                .setConstantHeadingInterpolation(redWhiteBackdrop.getHeading())
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
                follower.followPath(cycleStackTo, true);

                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
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
                if (pathTimer.getElapsedTimeSeconds() > 2) {
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
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.7);
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
                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    claw.closeClaws();
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
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
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    claw.openClaws();
                    setPathState(100202);
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
                gear.white54(); //148
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
                if (!gear.gear.isBusy()) {
                    gear.white32();
                }
                break;
            case 1:
                gear.scoringGear();
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


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

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

        gear.wheelServo_Deactivated();
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
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            //----------------------------1----------------------------\\
            if (blocks[i].x < 100 && blocks[i].id == 1 && blocks[i].y < 200) {
                navigation = "left";
            }
            if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 1 && blocks[i].y < 200) {
                navigation = "middle";
            }
            if (blocks[i].x > 210 && blocks[i].id == 1 && blocks[i].y < 200) {
                navigation = "right";
            }
        }
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
