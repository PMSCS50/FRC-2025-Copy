package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    private boolean hasTarget = false;
    private double targetYaw = 0.0;
    //Suhas: added target pitch, area, skew, and corners
    private double targetPitch = 0.0;
    private double targetArea = 0.0;
    private double targetSkew = 0.0;
    List<TargetCorner> targetCorners = new ArrayList<>();
    
    private int targetID = -1;

    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            hasTarget = false;
            targetID = -1;
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();

        hasTarget = true;
        targetYaw = target.getYaw();
        
        targetPitch = target.getPitch();
        targetArea = target.getArea();
        targetSkew = target.getSkew();
        targetCorners = target.getCorners();
        
        targetID = target.getFiducialId();
    }



    public boolean hasTargets() {
        return hasTarget;
    }

    public double getTargetYaw() {
        return hasTarget ? targetYaw : 0.0;
    }

    public double getTargetPitch() {
        return hasTarget ? targetPitch : 0.0;
    }

    public double getTargetArea() {
        return hasTarget ? targetArea : 0.0;
    }

    public double getTargetSkew() {
        return hasTarget ? targetSkew : 0.0;
    }

    public doubel getTargetCorners() {
        return hasTarget ? targetCOrners : 0.0;
    }

    public int getTargetID() {
        return targetID;
    }
}

