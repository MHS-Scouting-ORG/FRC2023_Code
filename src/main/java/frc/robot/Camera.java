package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;


public class Camera{
    private int width; // width resolution
    private int height; //height resolution
    private int exposure; // the exposure 
    private int framerateFront; // framerate of the front camera
    private int framerateBack; // framerate of the back camera
    private UsbCamera wideview;


    
    public Camera(int newWidth, int newHeight, int newExposure){
        width = newWidth;
        height = newHeight;
        exposure = newExposure; 
        framerateFront = 10;
        framerateBack = 20;

    }
    public Camera(){ // width, height, exposure
        this(160, 120, 3);
    }
    public void setExposure(int value){
        wideview.setExposureManual(value);
    }
    public void init(){ //initializing
        wideview = CameraServer.startAutomaticCapture(); // 
        wideview.setResolution(width, height); // setting the resolution
        wideview.setExposureManual(25); // manually setting the exposure
        wideview.setFPS(framerateFront); // setting 50the framerate
        wideview.setWhiteBalanceHoldCurrent(); // white balance of the camera

    }
}
