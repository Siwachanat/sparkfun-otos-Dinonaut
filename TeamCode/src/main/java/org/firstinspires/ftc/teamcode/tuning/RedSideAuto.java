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
                Gripper.setPosition(0.175);
                return false;
            }
        }
        public Action set() {
            return new Set();
        }

        public class Grip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.19);
                SL.setPosition(0.81);
                sleep(350);
                Gripper.setPosition(0.8);
                Smid.setPosition(0.65);
                SRG.setPosition(0.15);
                return false;
            }
        }
        public Action grip() {
            return new Grip();
        }
        public class Re implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SR.setPosition(0.19);
                SL.setPosition(0.81);
                sleep(350);
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
    }




    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-63, 0, Math.toRadians(0));
        Pose2d second = new Pose2d(-63+30,0,Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Mission mission = new Mission(hardwareMap);





        TrajectoryActionBuilder Tomid = drive.actionBuilder (initialPose)
                .splineToSplineHeading(new Pose2d(-63+30,0,Math.PI*2),Math.PI*2,new TranslationalVelConstraint(70));
        TrajectoryActionBuilder Tosam1 = drive.actionBuilder (second)
                .splineToConstantHeading(new Vector2d(-63+13,-30),Math.PI*2,new TranslationalVelConstraint(40))

                .splineToConstantHeading(new Vector2d(-20,-40),Math.PI*3/2,new TranslationalVelConstraint(39))
                .splineToConstantHeading(new Vector2d(-19,-44),Math.PI*3/2,new TranslationalVelConstraint(43))
                .splineToConstantHeading(new Vector2d(-52,-48),Math.PI*3/2,new TranslationalVelConstraint(48))

                .splineToConstantHeading(new Vector2d(-20,-40),Math.PI*3/2,new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-13,-51),Math.PI*3/2,new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(-52,-50),Math.PI*3/2,new TranslationalVelConstraint(48))

                //.splineToConstantHeading(new Vector2d(-9,-50),Math.PI*3/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(-20,-45),Math.PI*3/2,new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-13,-55),Math.PI*3/2,new TranslationalVelConstraint(35))
                .splineToConstantHeading(new Vector2d(-57,-60),Math.PI*3/2,new TranslationalVelConstraint(35));
                //.splineToSplineHeading(new Pose2d(-63+30,0,Math.PI*2),Math.PI*2,new TranslationalVelConstraint(60));


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


        Middle = Tomid.build();
        Sam1 = Tosam1.build();



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
                            new SleepAction(0.1),
                            Sam1
                                ),
                        new SleepAction(2)

                )
        );



    }
}
