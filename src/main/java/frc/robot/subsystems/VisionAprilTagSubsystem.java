package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionAprilTagConstants;

public class VisionAprilTagSubsystem extends SubsystemBase {

    //april tag
    PhotonCamera m_AprilTagCamera = new PhotonCamera("AprilTagCamera");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    Transform3d robotToCamera = new Transform3d(
            new Translation3d(VisionAprilTagConstants.kXOffset, VisionAprilTagConstants.kYOffset,
                    VisionAprilTagConstants.kZOffset),
            new Rotation3d(VisionAprilTagConstants.kRollOffset, VisionAprilTagConstants.kPitchOffset,
                    VisionAprilTagConstants.kYawOffset));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

    //Note
    //PhotonCamera m_NoteCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    //April Tag
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(getResultsAprilTag());
    }

    public PhotonPipelineResult getResultsAprilTag() {
        return m_AprilTagCamera.getLatestResult();
    }

    public boolean hasTargetsAprilTag() {
        return getResultsAprilTag().hasTargets();
    }

    public boolean getTargetVisibleAprilTag(int id) {
        for (PhotonTrackedTarget p : getResultsAprilTag().targets) {
            if (p.getFiducialId() == id)
                return true;
        }
        return false;
    }

    /*public double getSpeakerYaw() {
        if (hasTargetsAprilTag()) {
            return getSpeakerTarget().getYaw();
        }
        return 180;
    }
    
    /**
     * If the alliance information is not available, defaults to the ID of the blue alliance speaker.
     */
    /* public int getAprilTagSpeakerIDAprilTagIDSpeaker() {
        try {
            if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get()))
                return 4;
            return 7;
        } catch (Exception e) {
            return 7;
        }
    
    }
    
    public boolean getSpeakerTargetVisibleAprilTag() {
    return (getTargetVisibleAprilTag(getAprilTagSpeakerIDAprilTagIDSpeaker()));
    }
    
    public PhotonTrackedTarget getSpeakerTarget() {
    return getFiducial(getAprilTagSpeakerIDAprilTagIDSpeaker());
    }
    
    public PhotonTrackedTarget getFiducial(int id) {
    for (PhotonTrackedTarget p : getResultsAprilTag().targets) {
        if (p.getFiducialId() == id)
            return p;
    }
    return null;
    }
    
    //Note
    public PhotonPipelineResult getResultsNote() {
    return m_NoteCamera.getLatestResult();
    }
    
    public boolean hasTargetsNote() {
    return getResultsNote().hasTargets();
    }
    
    public double getBestResultYaw() {
    if (hasTargetsNote()) {
        return getResultsNote().getBestTarget().getYaw();
    } else {
        return 0;
    }
    }
    
    public double getDistanceToSpeaker() {
    PhotonTrackedTarget target = getSpeakerTarget();
    double distance;
    try {
        distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    } catch (Exception e) {
        // TODO: handle exception
        distance = 0;
    }
    return distance;
    }
    
    public boolean isTargetingSpeaker() {
    if (hasTargetsAprilTag() && getSpeakerTargetVisibleAprilTag()) {
        boolean targeting;
        try {
            targeting = getSpeakerTarget().getYaw() > 20
                    && getSpeakerTarget().getYaw() < 30;
        } catch (Exception e) {
            // TODO: handle exception
            targeting = false;
        }
        if (targeting) {
            //System.out.println(getSpeakerTarget().getYaw());
            return true;
        }
    }
    return false;
    }
    
    /*public PhotonTrackedTarget getTargetFromList(int ID, List<PhotonTrackedTarget> targetList) {
    for (int i = 0; i <= targetList.size(); i++) {
    if (ID == targetList.get(i).getFiducialId()) {
        return targetList.get(i);
    }
    }
    return new PhotonTrackedTarget(0, 0, 0, 0, -1, robotToCamera, robotToCamera, ID, null, null);
    
    }*/

@Override
public void periodic() {
    /*if (hasTargetsAprilTag()) {
    SmartDashboard.putBoolean("Targeting Speaker", getSpeakerTargetVisibleAprilTag());
    if (getSpeakerTargetVisibleAprilTag()) {
        SmartDashboard.putNumber("distance to speaker", getDistanceToSpeaker());
    }
    
    }
    SmartDashboard.putBoolean("facing toward speaker", isTargetingSpeaker());
    */
}

}
