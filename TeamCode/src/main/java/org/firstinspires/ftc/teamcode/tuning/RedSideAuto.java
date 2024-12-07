package org.firstinspires.ftc.teamcode.tuning;
import android.util.Log;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import com.acmerobotics.roadrunner.VelConstraint;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RED_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedSideAuto extends LinearOpMode {

    public class Lift {
        private DcMotorEx liftR;
        private DcMotorEx liftL;
        public int LiftReference;

        public Lift(HardwareMap hardwareMap) {
            liftR = hardwareMap.get(DcMotorEx.class, "liftMotorR");
            liftL = hardwareMap.get(DcMotorEx.class, "liftMotorL");
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);
            LiftReference = liftL.getCurrentPosition();
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-1);
                    liftL.setPower(1);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posL);
                if (posL < LiftReference +925) {
                    return true;
                } else {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }


            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftUp2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-1);
                    liftL.setPower(1);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();

                packet.put("liftPos", posL);
                if (posL < LiftReference +925) {
                    return true;
                } else {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }


            }
        }
        public Action liftUp2() {
            return new LiftUp2();
        }


        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(0.95);
                    liftL.setPower(-0.95);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 400) {
                    return true;
                } else {
                    liftR.setPower(0.04);
                    liftL.setPower(-0.04);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public class Mission {
        private Servo gripper2;
        private Servo SL;
        private Servo SR;
        private Servo Smid;
        private Servo S0;
        public Mission(HardwareMap hardwareMap) {
            S0 = hardwareMap.get(Servo.class, "S0");
            gripper2 = hardwareMap.get(Servo.class, "Gripper");
            SL = hardwareMap.get(Servo.class, "SL");
            SR = hardwareMap.get(Servo.class, "SR");
            Smid = hardwareMap.get(Servo.class, "midGrip");
        }
        public class Set implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //Set Grip to Position
                gripper2.setPosition(0.82);
                sleep(200);
                Smid.setPosition(0.9);
                //Rotate Arm to 2nd level chamber
                SR.setPosition(0.47);
                SL.setPosition(0.53);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }
        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(100);
                gripper2.setPosition(0.835);
                sleep(325);
                S0.setPosition(0.65);
                Smid.setPosition(0.45);
                SR.setPosition(1);
                SL.setPosition(0);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }
        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper2.setPosition(0.5);
                return false;
            }
        }
        public Action releases() {
            return new Re();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper2.setPosition(0.2);

                Smid.setPosition(0.33);
                SR.setPosition(0.335);
                SL.setPosition(0.665);
                gripper2.setPosition(0.2);
                return false;
            }
        }
        public Action mid() {
            return new Mid();
        }

    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "S0");
        }

        public class MidUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.76);
                return false;
            }
        }
        public Action midUp() {
            return new MidUP();
        }

        public class PushDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(1);
                return false;
            }
        }
        public Action pushDown() {
            return new PushDown();
        }
        public class PushUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.59);
                return false;
            }
        }
        public Action pushUp() {
            return new PushUp();
        }
    }

    public class GripperRot {
        private Servo gripperrot;

        public GripperRot(HardwareMap hardwareMap) {
            gripperrot = hardwareMap.get(Servo.class, "S4");
        }

        public class Zero implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripperrot.setPosition(0.07);
                return false;
            }
        }
        public Action zero() {
            return new Zero();
        }
        public class Ninety implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripperrot.setPosition(0.5);
                return false;
            }
        }
        public Action ninety() {
            return new Ninety();
        }
    }



    public class Slide {
        private Servo slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(Servo.class, "S5");
            slide.setPosition(0);
        }

        public class Full implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setPosition(0);
                return false;
            }
        }

        public Action full() {
            return new Full();
        }

        public class Back implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setPosition(0.51);
                return false;
            }
        }

        public Action back() {
            return new Back();
        }
    }
    public class Rot {
        private Servo rotate;

        public Rot(HardwareMap hardwareMap) {
            rotate = hardwareMap.get(Servo.class, "S1");
            rotate.setPosition(0.15);
        }

        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.15);
                return false;
            }
        }

        public Action down() {
            return new Down();
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.88);
                return false;
            }
        }

        public Action up() {
            return new Up();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.5);
                return false;
            }
        }

        public Action mid() {
            return new Mid();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(23, -63, Math.toRadians(0));
        Pose2d secondpose = new Pose2d(44, -45, Math.PI*3/2);
        Pose2d benopos1 =  new Pose2d(2,-34,Math.PI*3/2);
        Pose2d benopos2 =  new Pose2d(4,-34,Math.PI*3/2);
        Pose2d benopos3 =  new Pose2d(6,-34,Math.PI*3/2);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Rot rot = new Rot(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);
        // vision here that outputs position


        //TrajectoryActionBuilder Tomid = drive.actionBuilder(initialPose)
        //        .splineToSplineHeading( new Pose2d(-32,0,3.14159226535897932384626433832795028841),Math.PI*0)
        //        .waitSeconds(0.5);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToLinearHeading( new Pose2d(0,-35,Math.PI*3/2),Math.PI/1.5,new TranslationalVelConstraint(42.5));

        Action tab1 = Tomid.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(25.5,-38,Math.PI*0),Math.PI*0.5)
                .splineToConstantHeading(new Vector2d(35.5,-13),Math.PI/3)//1
                .splineToConstantHeading(new Vector2d(47,-51),Math.PI*3/2,null,new ProfileAccelConstraint(-30,70))
//                .new com.acmerobotics.roadrunner.VelConstraint()
                .splineToConstantHeading(new Vector2d(44,-12.5),Math.PI/3,new TranslationalVelConstraint(42.5))//2
                .splineToConstantHeading(new Vector2d(54.75,-51),Math.PI*3/2,null,new ProfileAccelConstraint(-30,70))
                .splineToSplineHeading(new Pose2d(44,-46.5,Math.PI*3/2),Math.PI*0.15,new TranslationalVelConstraint(55))
                .waitSeconds(0.2)

                .build();

        TrajectoryActionBuilder Tomid2 = drive.actionBuilder (secondpose)
                .splineToConstantHeading( new Vector2d(5,-35),Math.PI*0.5,new TranslationalVelConstraint(40.0));
//                .waitSeconds(1.5)
//                .splineToConstantHeading( new Vector2d(48,-45),Math.PI*0.5);



        TrajectoryActionBuilder Tomid3 = drive.actionBuilder (secondpose)
                .splineToConstantHeading( new Vector2d(0,-35),Math.PI*0.5,new TranslationalVelConstraint(40.0));
//                .waitSeconds(1.5)
//                .splineToConstantHeading( new Vector2d(48,-45),Math.PI*0.5);




        TrajectoryActionBuilder Tomid4 = drive.actionBuilder (secondpose)
                .splineToConstantHeading( new Vector2d(-4,-35),Math.PI*0.5,new TranslationalVelConstraint(40.0));
//                .waitSeconds(1)
//                .splineToConstantHeading( new Vector2d(48,-45),Math.PI*0.5);





        TrajectoryActionBuilder Tobeno1 = drive.actionBuilder (benopos1)
                .splineToConstantHeading( new Vector2d(44,-45),Math.PI*1.5,new TranslationalVelConstraint(65.0))
                .waitSeconds(0.25);



        TrajectoryActionBuilder Tobeno2 = drive.actionBuilder (benopos2)
                .splineToConstantHeading( new Vector2d(44,-46.5),Math.PI*1.5,new TranslationalVelConstraint(65.0))
                .waitSeconds(0.25);


        TrajectoryActionBuilder Tobeno3 = drive.actionBuilder (benopos3)
                .splineToConstantHeading( new Vector2d(44,-46.5),Math.PI*1.5,new TranslationalVelConstraint(65.0))
                .waitSeconds(0.25);



//No use
        TrajectoryActionBuilder ThirdSample = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(-54,-30.5,-Math.PI/2),-Math.PI*0);
        TrajectoryActionBuilder ForthSample = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(-54,-30.5,-Math.PI/2),-Math.PI*0);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(-40,-35,0),Math.PI*2)
                .splineToSplineHeading( new Pose2d(-9,-55,0),Math.PI*2)
//                .waitSeconds(0.01).splineToConstantHeading(new Vector2d(-52.5,-45),Math.PI/2)
                .lineToX(-52.5)
                .waitSeconds(0.1225)
                .splineToSplineHeading( new Pose2d(-58,-31,-Math.PI/2),Math.PI/2);
        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(gripper.pushUp());
        Actions.runBlocking(slide.back());
        Actions.runBlocking(rot.down());
        Actions.runBlocking(mission.set());
        //Actions.runBlocking(lift.liftUp());
        /*while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

         */

        ///////
        //Build Trajectory and Action
        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        Action trajectoryActionChosen3;
//        Action trajectoryActionChosen4;
//        Action trajectoryActionChosen5;
//        Action trajectoryActionChosen6;
        Action Middle;
        Action Middle2;
        Action Middle3;
        Action Middle4;
        Action Benopos1;
        Action Benopos2;
        Action Benopos3;
        Action Third;
        //trajectoryActionChosen = tab1.build();
        trajectoryActionChosen2 = tab2.build();
        trajectoryActionChosen3 = ForthSample.build();
//        trajectoryActionChosen4 = tab4.build();
//        trajectoryActionChosen5 = tab5.build();
//        trajectoryActionChosen6 = tab6.build();
        Middle2 = Tomid2.build();
        Middle = Tomid.build();
        Middle3 = Tomid3.build();
        Middle4 = Tomid4.build();

        Benopos1 = Tobeno1.build();
        Benopos2 = Tobeno2.build();
        Benopos3 = Tobeno3.build();


        Third = ThirdSample.build();
/////



        waitForStart();

        if (isStopRequested()) return;
///////
        /*
        //Build Trajectory and Action
        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        Action trajectoryActionChosen3;
//        Action trajectoryActionChosen4;
//        Action trajectoryActionChosen5;
//        Action trajectoryActionChosen6;
        Action Middle;
        Action Middle2;
        Action Middle3;
        Action Middle4;
        Action Benopos1;
        Action Benopos2;
        Action Benopos3;
        Action Third;
        //trajectoryActionChosen = tab1.build();
        trajectoryActionChosen2 = tab2.build();
        trajectoryActionChosen3 = ForthSample.build();
//        trajectoryActionChosen4 = tab4.build();
//        trajectoryActionChosen5 = tab5.build();
//        trajectoryActionChosen6 = tab6.build();
        Middle2 = Tomid2.build();
        Middle = Tomid.build();
        Middle3 = Tomid3.build();
        Middle4 = Tomid4.build();

        Benopos1 = Tobeno1.build();
        Benopos2 = Tobeno2.build();
        Benopos3 = Tobeno3.build();


        Third = ThirdSample.build();

         */
/////
////**///
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                mission.grip(), //Set Top gripper Ready to ng speciment
                                lift.liftUp(),// faster place specimen Lift arm for hang level
                                Middle//first speciment Trjectory Moving Path
                                ),

                        new ParallelAction(
                                gripper.pushUp(),//Deposit Gripper
                                lift.liftDown(),
                                new SleepAction(0.45),
                                mission.releases()),// release gripper for hang

                        new ParallelAction(
                                gripper.pushUp(),
                                tab1),

                        //----------------------------------------------------------------
                        slide.full(),//Extend Lower slide for collect second speciment

                        new SleepAction(0.5),
                        new ParallelAction(
                                mission.set(),
                                gripper.pushDown()
                        ),
                        new SleepAction(0.38),
                        gripper.midUp(),
                        new SleepAction(0.02),
//
                        new ParallelAction(
                                slide.back(),
                                new SleepAction(0.2),
                                rot.up()),



                        new SleepAction(0.356),
                        mission.mid(),
                        new SleepAction(0.25),


                        new ParallelAction(
                                mission.grip(),
                                lift.liftUp(),// faster place specimen
                                Middle2//hang third speciment
                        ),

                        new ParallelAction(
                                mission.releases(),
                                rot.down(),
                                lift.liftDown(),
                                gripper.pushUp()),
                        new SleepAction(0.26),
                        Benopos1,
                        //back to beno for 3 speciment


                        //-----------------------------------------------------------------
                        slide.full(),//Extend Lower slide for collect third speciment

                        new SleepAction(0.5),
                        gripper.pushDown(),
                        new SleepAction(0.4),
                        gripper.midUp(),
                        new SleepAction(0.02),
//
                        new ParallelAction(
                                slide.back(),
                                new SleepAction(0.3),
                                rot.up()),

                        new SleepAction(0.2),
                        mission.mid(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                mission.grip(),
                                lift.liftUp(),// faster place specimen
                                Middle3//hang third speciment
                        ),

                        new ParallelAction(
                                mission.releases(),
                                rot.down(),
                                lift.liftDown()),
                        new SleepAction(0.3),
                                Benopos2,
                                gripper.pushUp(),//back to beno for 3 speciment

                        //-------------------------------------------------------------------
                        slide.full(),//slide for collect forth speciment

                        new SleepAction(0.5),
                        gripper.pushDown(),
                        gripper.pushDown(),
                        new SleepAction(0.4),
                        gripper.midUp(),
                        new SleepAction(0.15),
//
                        new ParallelAction(
                                slide.back(),
                                new SleepAction(0.3),
                                rot.up()),
                        new SleepAction(0.4),
                        mission.mid(),
                        new SleepAction(0.25),


                        new ParallelAction(
                                mission.grip(),
                                lift.liftUp2(),// faster place specimen
                                Middle4//hang forth speciment
                        ),
                        new SleepAction(0.12),
                        new ParallelAction(
                                mission.releases(),
                                rot.down(),
                                lift.liftDown()),
                        new ParallelAction(
                                Benopos3,
                                slide.full()
                        ),

                       new SleepAction(0.5)
                )
        );



    }
}
