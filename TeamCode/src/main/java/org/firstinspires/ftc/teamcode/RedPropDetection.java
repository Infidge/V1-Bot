package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPropDetection extends OpenCvPipeline {
    private OpenCvCamera camera;
    private Mat imageHSV = new Mat();
    private Rect leftRegion;
    private Rect middleRegion;
    private Rect rightRegion;
    private Mat leftImage = new Mat();
    private Mat middleImage = new Mat();
    private Mat rightImage = new Mat();
    static final Rect LEFT_IMG = new Rect(
            new Point( 0, 0 ),
            new Point( 320, 720 ) );
    static final Rect MIDDLE_IMG = new Rect(
            new Point( 320, 0 ),
            new Point( 950, 720 ) );
    static final Rect RIGHT_IMG = new Rect(
            new Point( 950, 0 ),
            new Point( 1278, 720 ) );
    public enum ElementPosition {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private ElementPosition position;

    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    private Telemetry telemetry;

    public RedPropDetection(HardwareMap hardwareMap, Telemetry telemetry){
        /*this.leftRegion=LEFT_IMG;
        this.middleRegion=MIDDLE_IMG;
        this.rightRegion=RIGHT_IMG;
         */
        this.telemetry=telemetry;
        int camMonViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                camMonViewId
        );
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener( ) {
                                         @Override
                                         public void onOpened( ) {
                                             camera.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError( int errorCode ) {
                                             //This will be called if the camera could not be opened
                                             Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
                                         }
                                     }
        );

    }

    @Override
    public Mat processFrame(Mat input){
        //Convert input to HSV
        Imgproc.cvtColor(input,imageHSV,Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(imageHSV,imageHSV,Imgproc.COLOR_RGB2HSV);

        //Color Thresholds
        Scalar lowerBound = new Scalar(0,61,67); //RED
        Scalar upperBound = new Scalar(179,231,162); //RED

        /*Scalar lowerBound = new Scalar(59,46,0); //BLUE
        Scalar upperBound = new Scalar(117,215,164); //BLUE
        */
        Core.inRange(imageHSV,lowerBound,upperBound,imageHSV);

        //Divide Image
        leftImage = imageHSV.submat(LEFT_IMG);
        middleImage = imageHSV.submat(MIDDLE_IMG);
        rightImage = imageHSV.submat(RIGHT_IMG);

        //Get Color Average
        double leftValue = Core.mean(leftImage).val[0];
        double middleValue = Core.mean(middleImage).val[0];
        double rightValue = Core.mean(rightImage).val[0];


        //Clear Memory
        leftImage.release();
        middleImage.release();
        rightImage.release();;
        imageHSV.release();

        //Compare Values
        final double THRESHOLD = 0;
        final double max = Math.max(Math.max(leftValue, rightValue), middleValue);
        if(leftValue>THRESHOLD && leftValue == max){
            this.position=ElementPosition.LEFT;
            Imgproc.rectangle(input,LEFT_IMG,GREEN,1);
        }
        else if(middleValue>THRESHOLD && middleValue == max){
            this.position=ElementPosition.MIDDLE;
            Imgproc.rectangle(input,MIDDLE_IMG,GREEN,1);
        }
        else if(rightValue>THRESHOLD && rightValue == max){
            this.position=ElementPosition.RIGHT;
            Imgproc.rectangle(input,RIGHT_IMG,GREEN,1);
        }
        else{
            this.position=ElementPosition.NOT_FOUND;
        }
        telemetry.addLine("DONE");
        telemetry.addData("left", leftValue);
        telemetry.addData("middle", middleValue);
        telemetry.addData("right",rightValue);
        telemetry.addData("position", position);
        telemetry.update();
        return input;
    }

    public ElementPosition getElementPosition(){
        return position;
    }

    public void stopCamera(){
        camera.closeCameraDeviceAsync(() -> {});
    }
}

