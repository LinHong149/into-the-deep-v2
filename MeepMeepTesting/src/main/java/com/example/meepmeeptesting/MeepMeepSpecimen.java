package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16.25, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8, -61, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90)) //drop off pre loaded
                        .setTangent(Math.toRadians(315))
                        .splineToConstantHeading(new Vector2d(30, -40), Math.toRadians(0)) //path around submersible
                        .splineToConstantHeading(new Vector2d(44, -12), Math.toRadians(0)) //to first
                        .back(40) //push observation zone

                        .splineToConstantHeading(new Vector2d(56, -12), Math.toRadians(0)) //to second
                        .back(40) //push observation zone

                        .splineToConstantHeading(new Vector2d(62, -12), Math.toRadians(0)) //to third
                        .back(40)//push observation zone
                        .setTangent(Math.toRadians(120))
                        .splineToLinearHeading(new Pose2d(36,-59,Math.toRadians(270)),Math.toRadians(270)) //pickup
                        .splineToLinearHeading(new Pose2d(0,-32,Math.toRadians(90)),Math.toRadians(90)) //drop off first
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(36,-59,Math.toRadians(270)),Math.toRadians(270)) //pickup
                        .setTangent(Math.toRadians(120))
                        .splineToLinearHeading(new Pose2d(0,-32,Math.toRadians(90)),Math.toRadians(90)) //drop off second
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(new Pose2d(36,-59,Math.toRadians(270)),Math.toRadians(270)) //pickup
                        .setTangent(Math.toRadians(120))
                        .splineToLinearHeading(new Pose2d(0,-32,Math.toRadians(90)),Math.toRadians(90)) //drop off third
                        .setTangent(Math.toRadians(330))
                        .splineToLinearHeading(new Pose2d(40,-59,Math.toRadians(90)),Math.toRadians(0)) //park




//                        .splineTo(new Vector2d(-48,-40),Math.toRadians(90)) //to first sample; rotation 0.5


//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 1
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-53,-40),Math.toRadians(110)) //to second sample; rotation 0.75
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52, -53),Math.toRadians(225)) //drop off sample 2
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-56,-40),Math.toRadians(130)) // to third sample; rotation 0.95/1
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-52,-53),Math.toRadians(225)) //drop off sample 3
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-30,-11),Math.toRadians(0))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
