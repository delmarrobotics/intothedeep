package test.code;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import common.Config;
import common.Robot;

@TeleOp(name = "Limelight3A Align", group = "Test")
public class LimelightAlign extends LinearOpMode {
    public Limelight3A limelight;
    public DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);
        robot.init();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);

        limelight.pipelineSwitch(2);
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        LLResult result = limelight.getLatestResult();

        while(abs(result.getTx()) >0.007) {
            LLStatus status = limelight.getStatus();
            result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getTa() > 0.04) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                if(result.getTx() < 0) {
                    robot.strafeLeft(0.1);
                } else {
                    robot.strafeRight(0.1);
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
        }
        while(distanceSensor.getDistance(DistanceUnit.INCH) > 1 && distanceSensor.getDistance(DistanceUnit.INCH) < 0.9) {
            robot.forwardSlow(0.05);
        }
        telemetry.addLine("FINISHED");
    }
}
