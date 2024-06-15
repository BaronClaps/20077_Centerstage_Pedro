package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.FollowPathAction;
import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.AutoActionScheduler;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;

@Config
@Autonomous(name = "Blue Close 2+4 Nova",group = "Blue")
public final class Blue_Close_Two_Four_Nova extends LinearOpMode {

    protected AutoActionScheduler sched;
    private Follower follower;
    private String navigation;
    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private Pose startPose, spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;


    //Spike mark locations
    private Pose blueLeftSpikeMark = new Pose(-36+72+12, 32+72+1, Math.toRadians(270)); //51
    private Pose blueMiddleSpikeMark = new Pose(-30+72+12, 22+72, Math.toRadians(270));
    private Pose blueRightSpikeMark = new Pose(-36+72+12, 8+72, Math.toRadians(270));

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(30+12+2, 117+1, Math.toRadians(270)); //41
    private Pose blueMiddleBackdrop = new Pose(30+12+6, 51.5+72, Math.toRadians(270));
    private Pose blueRightBackdrop = new Pose(30+12+12, 51.5+72, Math.toRadians(270));
    private Pose blueWhiteBackdrop = new Pose(30+12, 117+1, Math.toRadians(270));

    //Through Truss
    private Pose blueTopTruss = new Pose(12+13+1, 84, Math.toRadians(270)); //22
    private Pose blueBottomTruss = new Pose(12+13+1, 36, Math.toRadians(270));

    // white pixel stack locations
    private Pose blueLeftStack = new Pose(-36+72+14+12, -37+72, Math.toRadians(270));
    private Pose blueMiddleStack = new Pose(-36+72+14+6, -37+72, Math.toRadians(270));
    private Pose blueRightStack = new Pose(36+12+1, 13.5, Math.toRadians(270)); //47

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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        navigation = "left";
        setBackdropGoalPose();
        buildPaths();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sched = new AutoActionScheduler(hardwareMap);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        sched.addAction(
                new SequentialAction(
                        new FollowPathAction(follower, scoreSpikeMark, false),
                        new SleepAction(0.5)
                ));
        sched.run();

        Pose currentPose = follower.getPose();

        follower.update();
        sched.addAction(
                new SequentialAction(
                        new FollowPathAction(follower, initialScoreOnBackdrop),
                        new SleepAction(1.25)
                ));
        sched.run();
    }
}