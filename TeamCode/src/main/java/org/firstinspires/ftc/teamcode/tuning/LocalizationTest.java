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
        Smid = hardwareMap.get(Servo.class, "Smid");
        SR = hardwareMap.get(Servo.class, "SR");
        SL = hardwareMap.get(Servo.class, "SL");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        liftR = hardwareMap.get(DcMotor.class, "liftMotorR");
        liftL = hardwareMap.get(DcMotor.class, "liftMotorL");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        S0.setPosition(0.65);
        S1.setPosition(0);
        S5.setPosition(0.5);

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
                S0.setPosition(0.65);
            } else if (gamepad1.dpad_down) {
                S1.setPosition(0);
                S0.setPosition(0.15);
                sleep(450);
                S0.setPosition(0.5);
            }
            if (gamepad1.left_bumper) {
                S1.setPosition(0);
                S0.setPosition(0.7);
            } else if (gamepad1.right_bumper) {
                S1.setPosition(0.36);
                S5.setPosition(0.38);
                sleep(800);
                S1.setPosition(0.72);
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
                Gripper.setPosition(0.5);
                Smid.setPosition(0.05);
            }
            if (gamepad2.a){
                Smid.setPosition(0.57);
                SR.setPosition(0.37);
                SL.setPosition(0.63);
            }else if (gamepad2.b){
                Smid.setPosition(0.11);
                SR.setPosition(0.45);
                SL.setPosition(0.55);
            }else if (gamepad2.y){
                Smid.setPosition(0.58);
                SR.setPosition(1);
                SL.setPosition(0);
                sleep(550);
           }//else if (gamepad2.x){
//                Smid.setPosition(0.05);
//            }
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
