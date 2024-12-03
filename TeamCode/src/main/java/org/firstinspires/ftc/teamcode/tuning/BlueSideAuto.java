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
                    liftR.setPower(-0.98);
                    liftL.setPower(0.98);
                    initialized = true;
                }

                int posR = liftR.getCurrentPosition();
                int posL = liftL.getCurrentPosition();

                telemetry.addData("Position bef", posL);
                telemetry.update();


                packet.put("liftPos", posL);
                if (posL < LiftReference +4050) {
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
                if (pos > 1500) {
                    return true;
                } else {
                    liftR.setPower(0.95);
                    liftL.setPower(-0.95);
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
                Smid.setPosition(0.1);
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
                sleep(350);
                S0.setPosition(0.65);
                Smid.setPosition(0.15);
                SR.setPosition(0.85);
                SL.setPosition(0.15);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }
        public class PreGrip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sleep(100);
                gripper2.setPosition(0.835);
                sleep(350);
                S0.setPosition(0.65);
                Smid.setPosition(0.15);
                SR.setPosition(0.7);
                SL.setPosition(0.3);
                return false;
            }
        }
        public Action pregrip() {
            return new PreGrip();
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

                Smid.setPosition(0.65);
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
                slide.setPosition(0.48);
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
        Pose2d initialPose = new Pose2d(-23, -61.5, Math.toRadians(0));
        Pose2d second = new Pose2d(-47,-47.5,Math.PI/4);
        Pose2d second2 = new Pose2d(-47,-47.5,Math.PI/2);
        Pose2d Third = new Pose2d(-56.5,-41.75,Math.PI/2);
        Pose2d Forth = new Pose2d(-52.3,-25,Math.PI+0);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Rot rot = new Rot(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);
        GripperRot gripperRot = new GripperRot(hardwareMap);
        // vision here that outputs position
        TrajectoryActionBuilder Putspec1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-47,-47.5,Math.PI/4),-Math.PI*2);


        TrajectoryActionBuilder firstspec1 = drive.actionBuilder(second)
                .turn(Math.PI/4)
                .lineToY(-43);

        TrajectoryActionBuilder firstspec2 = drive.actionBuilder(second2)
                .lineToY(-47.5)
                .turn(-Math.PI/4);//turn to put in to the basket

        TrajectoryActionBuilder tab3 = drive.actionBuilder(second)
                .splineToLinearHeading(new Pose2d(-56.5,-41.75,Math.PI/2),Math.PI*2);//go get second spec

        TrajectoryActionBuilder tab4 = drive.actionBuilder(Third)
                .splineToLinearHeading(new Pose2d(-47,-48,Math.PI/4),Math.PI*2);//return to basket

        TrajectoryActionBuilder tab5 = drive.actionBuilder(second)
                .splineToLinearHeading(new Pose2d(-52.3,-25,Math.PI+0),Math.PI*2);//go to collect third spec
        TrajectoryActionBuilder tab6 = drive.actionBuilder(Forth)
                .splineToLinearHeading(new Pose2d(-47.5,-48,Math.PI/4),Math.PI*2);//return to basket
        TrajectoryActionBuilder Park = drive.actionBuilder(second)
                .splineToLinearHeading(new Pose2d(-25,-0,Math.PI+0),Math.PI*2);//park






        Actions.runBlocking(gripper.pushUp());
        Actions.runBlocking(slide.back());
        Actions.runBlocking(rot.down());
        Actions.runBlocking(mission.set());



        Action T0pos1;
        Action T0pos2;
        Action T0pos3;
        Action T0pos4;
        Action T0pos5;
        Action T0pos6;
        Action T0pos7;
        Action T0pos8;
        Action T0pos9;

        T0pos1 = Putspec1.build();
        T0pos2 = firstspec1.build();
        T0pos3 = firstspec2.build();
        T0pos4 = tab3 .build();
        T0pos5 = tab4.build();
        T0pos6 = tab5.build();
        T0pos7 = tab6.build();
        T0pos8 = Park.build();
        T0pos9 = Putspec1.build();


        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                T0pos1,
                                lift.liftUp()),
                        mission.grip(),
                        new SleepAction(0.5),
                        mission.releases(),
                        new SleepAction(0.3),

                        new ParallelAction(
                                mission.set(),
                                new SleepAction(0.15),
                                lift.liftDown(),
                                T0pos2

                        ),
                        slide.full(),
                        new SleepAction(0.55),
                        gripper.pushDown(),
                        new SleepAction(0.35),
                        gripper.midUp(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                slide.back(),
                                gripperRot.zero(),
                                new SleepAction(0.3),
                                rot.up()),
                        new SleepAction(0.35),
                        mission.mid(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                mission.pregrip(),
                                T0pos3
                        ),
                        lift.liftUp(),
                        mission.grip(),
                        new SleepAction(0.2),
                        mission.releases(),
                        new SleepAction(0.4),
                        lift.liftDown(),
                        new ParallelAction(
                                lift.liftDown(),
                                T0pos4,
                                rot.down()

                        ),


                        slide.full(),
                        new SleepAction(0.55),
                        gripper.pushDown(),
                        new SleepAction(0.35),
                        gripper.midUp(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                slide.back(),
                                new SleepAction(0.3),
                                rot.up()),
                        new SleepAction(0.4),
                        mission.mid(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                mission.pregrip(),
                                T0pos5
                        ),
                        lift.liftUp(),
                        mission.grip(),
                        new SleepAction(0.2),
                        mission.releases(),
                        new SleepAction(0.25),
                        lift.liftDown(),
                        new ParallelAction(
                                lift.liftDown(),
                                new SleepAction(0.25),
                                T0pos6,
                                gripperRot.ninety(),
                                rot.down(),
                                new SleepAction(0.25)
                        ),
                        slide.full(),
                        new SleepAction(0.55),
                        gripper.pushDown(),
                        new SleepAction(0.35),
                        gripper.midUp(),
                        new SleepAction(0.2),

                        new ParallelAction(
                                slide.back(),
                                gripperRot.zero(),
                                new SleepAction(0.3),
                                rot.up()),
                        new SleepAction(0.4),
                        mission.mid(),
                        new SleepAction(0.25),
                        new ParallelAction(
                                mission.pregrip(),
                                T0pos7
                        ),
                        lift.liftUp(),
                        mission.grip(),
                        new SleepAction(0.25),
                        mission.releases(),
                        new SleepAction(0.25),
                        lift.liftDown(),
                        new SleepAction(0.25),
                        T0pos8,
                        new SleepAction(0.25)                )

//                )
        );



    }


}
