package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public class LocalizationTest extends LinearOpMode {
    public Servo S0;
    public Servo S1;
    public Servo S4;
    public Servo S5;
    private Servo Smid;
    private Servo SR;
    private Servo SL;
    private Servo Gripper;
    private DcMotor liftR;
    private DcMotor liftL;
    @Override
    public void runOpMode() throws InterruptedException {
        S0 = hardwareMap.get(Servo.class, "S0");
        S1 = hardwareMap.get(Servo.class, "S1");
        S5 = hardwareMap.get(Servo.class, "S5");
        S4 = hardwareMap.get(Servo.class, "S4");
        Smid = hardwareMap.get(Servo.class, "midGrip");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        liftR = hardwareMap.get(DcMotor.class, "liftMotorR");
        liftL = hardwareMap.get(DcMotor.class, "liftMotorL");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        S0.setPosition(0.59);
        S1.setPosition(0.15);
        S5.setPosition(0.6);

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();
            if (gamepad1.dpad_up) {
                S0.setPosition(0.59);
            } else if (gamepad1.dpad_down) {
                S1.setPosition(0.15);
                S0.setPosition(1);
                sleep(450);
                S0.setPosition(0.8);
            }
            if (gamepad1.dpad_left) {
                S4.setPosition(0.07);
            } else if (gamepad1.dpad_right) {
                S4.setPosition(0.5);
            }
            if (gamepad1.left_bumper) {
                S1.setPosition(0.15);
                S0.setPosition(0.59);
                S5.setPosition(0.15);
            } else if (gamepad1.right_bumper) {
                S1.setPosition(0.5);
                S5.setPosition(0.58);
                sleep(800);
                S1.setPosition(0.88);
                S4.setPosition(0.07);
            } else if (gamepad1.b) {
                S5.setPosition(gamepad1.right_trigger);
            }
            if (gamepad2.dpad_up){
                liftR.setPower(-1);
                liftL.setPower(1);
            } else if (gamepad2.dpad_down) {
                liftR.setPower(1);
                liftL.setPower(-1);
            }else {
                liftR.setPower(0.1);
                liftL.setPower(0.1);
            }
            if (gamepad2.right_bumper){
                Gripper.setPosition(0.81);
                S0.setPosition(0.65);
            } else if (gamepad2.left_bumper) {
                Gripper.setPosition(0.35);
                Smid.setPosition(0.85);
                liftR.setPower(0);
                liftL.setPower(0);

            }
            if (gamepad2.a){
                Smid.setPosition(0.65);
                SR.setPosition(0.91);
                SL.setPosition(0.09);
                Gripper.setPosition(0.2);
            }else if (gamepad2.b){
                SR.setPosition(0.45);
                SL.setPosition(0.55);
            }else if (gamepad2.y){
                Gripper.setPosition(0.82);
                S0.setPosition(0.65);
                sleep(420);
                Smid.setPosition(0.85);
                SR.setPosition(0.05);
                SL.setPosition(0.95);
           }else if (gamepad2.x){
//                Gripper.setPosition(0.82);
//                Smid.setPosition(0.85);
//                SR.setPosition(0.95);
//                SL.setPosition(0.05);
                liftR.setPower(-0.7);
                liftL.setPower(0.7);
                sleep(750);


           }
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
