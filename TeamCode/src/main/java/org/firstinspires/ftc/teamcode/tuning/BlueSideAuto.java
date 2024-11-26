package org.firstinspires.ftc.teamcode.tuning;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.ref.Reference;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideAuto extends LinearOpMode {

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
                    liftR.setPower(-0.9);
                    liftL.setPower(0.9);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posL);
                if (posL < LiftReference +3600) {
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
        public class LiftUp1 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(-0.9);
                    liftL.setPower(0.9);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posL);
                if (posL < LiftReference +1200) {
                    return true;
                } else {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    return false;
                }


            }
        }
        public Action liftUp1() {
            return new LiftUp1();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftR.setPower(0.8);
                    liftL.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 200) {
                    return true;
                } else {
                    liftR.setPower(0.1);
                    liftL.setPower(-0.1);
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
            Smid = hardwareMap.get(Servo.class, "Smid");
        }
        public class Set implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper2.setPosition(0.81);
                sleep(200);
                Smid.setPosition(0.11);
                SR.setPosition(0.45);
                SL.setPosition(0.55);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }
        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper2.setPosition(0.82);
                S0.setPosition(0.65);
                sleep(420);
                Smid.setPosition(0.85);
                SR.setPosition(0.95);
                SL.setPosition(0.05);
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
                Smid.setPosition(0.05);
                return false;
            }
        }
        public Action releases() {
            return new Re();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Smid.setPosition(0.68);
                SR.setPosition(0.27);
                SL.setPosition(0.73);
                gripper2.setPosition(0.51);
                return false;
            }
        }
        public Action mid() {
            return new Mid();
        }
        public class Af implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Smid.setPosition(0.75);
                SR.setPosition(0.29);
                SL.setPosition(0.71);
                gripper2.setPosition(0.5);
                return false;
            }
        }
        public Action af() {
            return new Af();
        }
    }







    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "S0");
            gripper.setPosition(0.65);
        }

        public class MidUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.5);
                return false;
            }
        }
        public Action midUp() {
            return new MidUP();
        }

        public class PushDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.1);
                return false;
            }
        }
        public Action pushDown() {
            return new PushDown();
        }
        public class PushUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.65);
                return false;
            }
        }
        public Action pushUp() {
            return new PushUp();
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
                slide.setPosition(0.5);
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
            rotate.setPosition(0);
        }

        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0);
                return false;
            }
        }

        public Action down() {
            return new Down();
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotate.setPosition(0.82);
                return false;
            }
        }

        public Action up() {
            return new Up();
        }
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-61.4, -23, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Rot rot = new Rot(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        TrajectoryActionBuilder Tomid = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(25,-20,3.14145265358979323846264338327),Math.PI*0)
                .waitSeconds(0.5);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(20,23,0),-Math.PI*0)
                .waitSeconds(0.3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(15,28,-Math.PI*2.25),Math.PI*0)
                .waitSeconds(0.3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .splineToSplineHeading( new Pose2d(22,33,0),Math.PI*0)
                .waitSeconds(0.5);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose);
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose);


        // actions that need to happen on init; for instance, a claw tightening.

        //Actions.runBlocking(new ParallelAction(
        //        gripper.pushUp(),
        //        slide.back()
        //));
        Actions.runBlocking(gripper.pushUp());
        Actions.runBlocking(slide.back());
        Actions.runBlocking(rot.down());
        /*while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

         */

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        Action trajectoryActionChosen3;
        Action trajectoryActionChosen4;
        Action trajectoryActionChosen5;
        Action trajectoryActionChosen6;
        Action Middle;
        trajectoryActionChosen = tab1.build();
        trajectoryActionChosen2 = tab2.build();
        trajectoryActionChosen3 = tab3.build();
        trajectoryActionChosen4 = tab4.build();
        trajectoryActionChosen5 = tab5.build();
        trajectoryActionChosen6 = tab6.build();
        Middle = Tomid.build();
        Actions.runBlocking(
                new SequentialAction(
                        Middle,
                        trajectoryActionChosen,
                        //lift.liftUp(),
                        slide.full(),
                        new SleepAction(0.7),
                        gripper.pushDown(),
                        new SleepAction(0.5),
                        gripper.midUp(),
                        new SleepAction(0.3),
                        rot.up(),
                        slide.back(),
                        new SleepAction(1),
                        gripper.pushUp(),
                        new SleepAction(0.3),
                        trajectoryActionChosen2,
                        lift.liftUp(),
                        new SleepAction(3)
                )
//                new SequentialAction(
//                        Middle,
//                        trajectoryActionChosen,
//                        //lift.liftUp(),
//                        slide.full(),
//                        new SleepAction(0.7),
//                        gripper.pushDown(),
//                        new SleepAction(0.5),
//                        gripper.midUp(),
//                        new SleepAction(0.3),
//                        rot.up(),
//                        slide.back(),
//                        new SleepAction(1),
//                        gripper.pushUp(),
//                        new SleepAction(0.3),
//                        trajectoryActionChosen2,
//                        lift.liftUp(),
//                        new SleepAction(3)
//                )
        );



    }
}