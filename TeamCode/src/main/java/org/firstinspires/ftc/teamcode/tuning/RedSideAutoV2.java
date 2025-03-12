package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL_", group = "Autonomous")
public class RedSideAutoV2 extends LinearOpMode {

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
                if (posL < LiftReference +890) {
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
                if (posL < LiftReference +890) {
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
                    liftR.setPower(0.055);
                    liftL.setPower(-0.055);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public class Mission {
        public Servo S0;
        public Servo S1;
        public Servo S4;
        public Servo S5;
        private Servo Smid;
        private Servo SR;
        private Servo SL;
        private Servo SRG;
        private Servo Gripper;
        private Servo Sarm;
        public Mission(HardwareMap hardwareMap) {
            SRG = hardwareMap.get(Servo.class, "SRG");
            S0 = hardwareMap.get(Servo.class, "S0");
            S1 = hardwareMap.get(Servo.class, "S1");
            S5 = hardwareMap.get(Servo.class, "S5");
            S4 = hardwareMap.get(Servo.class, "S4");
            Smid = hardwareMap.get(Servo.class, "midGrip");
            SR = hardwareMap.get(Servo.class, "SR");
            SL = hardwareMap.get(Servo.class, "SL");
            Gripper = hardwareMap.get(Servo.class, "Gripper");
            Sarm = hardwareMap.get(Servo.class, "Sarm");
        }
        public class Set implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.1);
                sleep(100);
                Smid.setPosition(0.5);
                sleep(100);
                SL.setPosition(0.320);
                SR.setPosition(0.680);
                Gripper.setPosition(0.19);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }

        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.1);
                sleep(100);
                Smid.setPosition(0.5);
                SL.setPosition(0.320);
                SR.setPosition(0.680);
                Gripper.setPosition(0.19);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }

        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.25);
                SL.setPosition(0.75);
                sleep(500);
                SR.setPosition(0.19);
                SL.setPosition(0.81);
                Gripper.setPosition(0.8);
                Smid.setPosition(0.65);
                SRG.setPosition(0.15);
                return false;
            }
        }
        public Action releases() {
            return new Re();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Gripper.setPosition(0.65);
                Smid.setPosition(0.31);
                SR.setPosition(0.31);
                SL.setPosition(0.69);
                Gripper.setPosition(0.2);
                return false;
            }
        }
        public Action mid() {
            return new Mid();
        }
        public class SlideFullUP implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S4.setPosition(0.5);
                S1.setPosition(0);
                S0.setPosition(1);
                S5.setPosition(0.55);
                return false;
            }
        }
        public Action slideFullUP() {
            return new SlideFullUP();
        }
        public class SlideIN implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                S5.setPosition(0.15);
                S1.setPosition(0);
                S4.setPosition(0.97);
                return false;
            }
        }
        public Action slideIN() {
            return new SlideIN();
        }


        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.078);
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sarm.setPosition(0.59);
                return false;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }
    }




    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-63, 0-3.5, Math.toRadians(0));
        Pose2d second = new Pose2d(-63+30,0-3.5,Math.toRadians(0));
        Pose2d third = new Pose2d(-55.5,-48,Math.toRadians(0));
        Pose2d forth = new Pose2d(-28.5,0,Math.toRadians(0));
        Pose2d forthfixed = new Pose2d(-40,0,Math.toRadians(0));
        Pose2d fifth = new Pose2d(-55.5,-39,Math.toRadians(0));
        Pose2d six = new Pose2d(-28.5,3.5,Math.toRadians(0));
        Pose2d sev = new Pose2d(-28.5,-6,Math.toRadians(0));
        Pose2d exx = new Pose2d(-28.5,6,Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);

        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(-63+33,0-3.5,Math.PI*2),Math.PI*2,null,new ProfileAccelConstraint(-100,100));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                .splineToConstantHeading(new Vector2d(-63+23,0-3.5),-Math.PI*2,null,new ProfileAccelConstraint(-30,80))
                .splineToConstantHeading(new Vector2d(-63+22,-10),-Math.PI*0.5,new TranslationalVelConstraint(100))
                .splineToSplineHeading( new Pose2d(-34,-28,-Math.PI/3.8),-Math.PI*1.5,new TranslationalVelConstraint(90))
                .waitSeconds(0.01)
                .splineToSplineHeading( new Pose2d(-40,-32,-Math.PI*4/6),-Math.PI*1.5,new TranslationalVelConstraint(90))
//                .splineToSplineHeading( new Pose2d(-35,-31,-Math.PI/4.6),-Math.PI*1.5,new TranslationalVelConstraint(60))
                .splineToSplineHeading( new Pose2d(-35,-40,-Math.PI/4.6),-Math.PI*1.5,new TranslationalVelConstraint(70))
                .waitSeconds(0.01)
                .splineToSplineHeading( new Pose2d(-40,-40,-Math.PI*4/6),-Math.PI*1.5,new TranslationalVelConstraint(90))
//                .splineToSplineHeading( new Pose2d(-35,-40,-Math.PI/4.6),-Math.PI*1.5,new TranslationalVelConstraint(60))
                .splineToSplineHeading( new Pose2d(-35,-49.5,-Math.PI/4.6),-Math.PI*1.5,new TranslationalVelConstraint(90))
                .waitSeconds(0.01)
                .splineToSplineHeading( new Pose2d(-40,-49.5,-Math.PI* 4/6),-Math.PI*1.5,null,new ProfileAccelConstraint(-50,100))
                .splineToSplineHeading( new Pose2d(-40,-49.5,-Math.PI*12/6),-Math.PI*1.5,new TranslationalVelConstraint(100))
                .waitSeconds(0.01)
                .splineToSplineHeading( new Pose2d(-55.5,-48,-Math.PI*12/6),-Math.PI*1.5,null,new ProfileAccelConstraint(-50,100));

//                .splineToConstantHeading(new Vector2d(-19.5,-35),Math.PI*2,new TranslationalVelConstraint(50))
//                .splineToConstantHeading(new Vector2d(-19.5,-48),-Math.PI*1,new TranslationalVelConstraint(100))
//                .splineToConstantHeading(new Vector2d(-55.5,-48),Math.PI*2,null,new ProfileAccelConstraint(-20,100));

        TrajectoryActionBuilder Tosam2 = drive.actionBuilder (third)
//
//                .splineToConstantHeading(new Vector2d(-19.5,-45),Math.PI*2,null,new ProfileAccelConstraint(-50,100))
//                .splineToConstantHeading(new Vector2d(-19.5,-56.5),-Math.PI*1,new TranslationalVelConstraint(100))
//                .splineToConstantHeading(new Vector2d(-46,-56.5),Math.PI*2,null,new ProfileAccelConstraint(-20,60))
//
//
//                .splineToConstantHeading(new Vector2d(-17,-54),Math.PI*2,null,new ProfileAccelConstraint(-80,35))
//                .splineToConstantHeading(new Vector2d(-17,-62.25),-Math.PI*1,new TranslationalVelConstraint(100))
//                .splineToConstantHeading(new Vector2d(-46,-62.25),Math.PI*2,null,new ProfileAccelConstraint(-60,100))
//
//
                .splineToConstantHeading(new Vector2d(-28.5,0),Math.PI*0.25,new TranslationalVelConstraint(90));
        //.splineToConstantHeading(new Vector2d(-63+33,-3.5),Math.PI*2,new TranslationalVelConstraint(100));


        TrajectoryActionBuilder Tosam3 = drive.actionBuilder (forth)
                .splineToConstantHeading(new Vector2d(-40,0),Math.PI*2,null,new ProfileAccelConstraint(-45,100));
                //.splineToConstantHeading(new Vector2d(-55,-39.5),-Math.PI*0.4,new TranslationalVelConstraint(100));
                //.splineToConstantHeading(new Vector2d(-63+7.15,-39),Math.PI*2,new TranslationalVelConstraint(27));

        TrajectoryActionBuilder Tosam3fix = drive.actionBuilder (forthfixed)
                .splineToConstantHeading(new Vector2d(-55,-39.5),-Math.PI*0.45,new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam4 = drive.actionBuilder (fifth)
                .splineToConstantHeading(new Vector2d(-28.5,3.5),Math.PI*0.25,new TranslationalVelConstraint(95));
                //.splineToConstantHeading(new Vector2d(-63+33,3.5),Math.PI*2,new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam5 = drive.actionBuilder (six)
                .splineToConstantHeading(new Vector2d(-36,3.5),Math.PI*2,null,new ProfileAccelConstraint(-45,100))
                .splineToConstantHeading(new Vector2d(-55,-39),-Math.PI*0.4,new TranslationalVelConstraint(80));
                //.splineToConstantHeading(new Vector2d(-63+6,-40),Math.PI*2,new TranslationalVelConstraint(27));

        TrajectoryActionBuilder Tosam6 = drive.actionBuilder (fifth)

                .splineToConstantHeading(new Vector2d(-28.5,-6),Math.PI*0.25,new TranslationalVelConstraint(95));
                //.splineToConstantHeading(new Vector2d(-31,-6),Math.PI*2,new TranslationalVelConstraint(80));


        TrajectoryActionBuilder Tosam7 = drive.actionBuilder (sev)
                .splineToConstantHeading(new Vector2d(-36,-6),Math.PI*2,null,new ProfileAccelConstraint(-45,80))
                .splineToConstantHeading(new Vector2d(-55,-39),-Math.PI*0.4,new TranslationalVelConstraint(80));
                //.splineToConstantHeading(new Vector2d(-63+6,-40),Math.PI*2,new TranslationalVelConstraint(27));

        TrajectoryActionBuilder Tosam8 = drive.actionBuilder (fifth)

                .splineToConstantHeading(new Vector2d(-28.5,6),Math.PI*0.5,new TranslationalVelConstraint(100));
                //.splineToConstantHeading(new Vector2d(-32,6),Math.PI*2,new TranslationalVelConstraint(100));

        TrajectoryActionBuilder Tosam9 = drive.actionBuilder (exx)
                .splineToConstantHeading(new Vector2d(-36,6),Math.PI*2,null,new ProfileAccelConstraint(-45,80))
                .splineToConstantHeading(new Vector2d(-52,-40),-Math.PI*0.5,new TranslationalVelConstraint(100));




                /*.setTangent(Math.PI*3/2)
                .splineToConstantHeading(new Vector2d(-20,-40),Math.PI*3/2,new TranslationalVelConstraint(80))
                .splineToConstantHeading(new Vector2d(-12,-51),Math.PI*3/2,new TranslationalVelConstraint(50))

                .splineToSplineHeading(new Pose2d(-52,-52.5,Math.PI*2),Math.PI*3/2,new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(-20,-50,Math.PI*2),Math.PI*3/2,null,new ProfileAccelConstraint(-10,10))
                .splineToSplineHeading(new Pose2d(-12,-62,Math.PI*2),Math.PI*3/2,new TranslationalVelConstraint(50))

                .splineToSplineHeading(new Pose2d(-50,-62,Math.PI*2),Math.PI*2,new TranslationalVelConstraint(50));*/


        Actions.runBlocking(mission.set());
        Actions.runBlocking(mission.slideIN());

        Actions.runBlocking(mission.set());
        Actions.runBlocking(mission.slideIN());


        Action Middle;
        Action Sam1;
        Action Sam2;
        Action Sam3;
        Action Sam3fix;
        Action Sam4;
        Action Sam5;
        Action Sam6;
        Action Sam7;
        Action Sam8;
        Action Sam9;



        Middle = Tomid.build();
        Sam1 = Tosam1.build();
        Sam2 = Tosam2.build();
        Sam3 = Tosam3.build();
        Sam3fix = Tosam3fix.build();
        Sam4 = Tosam4.build();
        Sam5 = Tosam5.build();
        Sam6 = Tosam6.build();
        Sam7 = Tosam7.build();
        Sam8 = Tosam8.build();
        Sam9 = Tosam9.build();



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                mission.set(), //Set Top gripper Ready to ng speciment
                                Middle//first speciment Trjectory Moving Path
                                ),
                        new ParallelAction(
                                mission.releases(),
                                Sam1,
                                new SequentialAction(
                                        new SleepAction(0.85),
                                        mission.armDown()
                                )
                        ),

                        new ParallelAction(
                                Sam2,
                                mission.armUp(),
                                mission.grip()
                        ),
                        new ParallelAction(
                                Sam3,
                                mission.releases()
                        ),
                        Sam3fix,
                        new ParallelAction(
                                mission.grip(),
                                Sam4

                        ),
                        new ParallelAction(
                                Sam5,
                                mission.releases()
                        ),
                        new ParallelAction(
                                mission.grip(),
                                Sam6

                        ),
                        new ParallelAction(
                                Sam7,
                                mission.releases()
                        ),
                        new ParallelAction(
                                mission.grip(),
                                Sam8

                        ),
                        new ParallelAction(
                                Sam9,
                                mission.releases()
                        ),
                        new SleepAction(1.455)

                )
        );



    }
}
