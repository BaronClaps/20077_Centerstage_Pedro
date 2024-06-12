package org.firstinspires.ftc.teamcode.config.subsystem;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;


public class PresetSubsystem {
    public ClawSubsystem claw;
    public LiftSubsystem lift;
    public GearSubsystem gear;

    public PresetSubsystem(ClawSubsystem clawSubsystem, LiftSubsystem liftSubsystem, GearSubsystem gearSubsystem) {
        this.claw = clawSubsystem;
        this.lift = liftSubsystem;
        this.gear = gearSubsystem;
    }


    //------------------------------ Start Sequence ------------------------------//
    public void StartPos() {
        ClawStartPos();
        LiftStartPos();
        GearStartPos();
    }

    public void GearStartPos() {
        //gear.resetGear();
        //gear.gearTarget(-500);
        gear.startGear();
    }

    public void LiftStartPos() {
        lift.stopLift();
        lift.resetLift();
    }

    public void ClawStartPos() {
        claw.closeClaws();
        claw.groundClaw();
    }

    //------------------------------ Scoring Sequence ------------------------------//

    public void ScoringPos() {
        ClawScoringPos();
        GearScoringPos();
        LiftScoringPos();
    }

    public void whiteScoringPos() {
        WhiteClawScoringPos();
     //   GearScoringPos();
        WhiteLiftScoringPos();
    }

    public void GearScoringPos() {
        gear.resetGear();
        gear.gearTarget(750);
     
    }

    public void LiftScoringPos() {
        lift.liftExtend_Scoring();
        //lift.waitForLift(),
        //lift.stopLift()
    }

    public void WhiteLiftScoringPos() {
        lift.liftExtend_WhiteScoring();
    }

    public void ClawScoringPos() {
        claw.scoringClaw();
    }

    public void WhiteClawScoringPos() {
        claw.whiteScoringClaw();
    }

    //------------------------------ Ground after Scoring Sequence ------------------------------//

    public void GroundPos() {
        GearGroundPos();
        ClawGroundPos();
        LiftGroundPos();
    }

    public void GearGroundPos() {
        gear.groundGear();
    }

    public void LiftGroundPos() {
        lift.liftRetract_Scoring();
    }

    public void ClawGroundPos() {
        claw.groundClaw();
        claw.openClaws();
    }

}
