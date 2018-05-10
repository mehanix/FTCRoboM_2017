package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuMarkFinder
{
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private VuforiaTrackable template;

    public VuMarkInstanceId instanceId;
    public OpenGLMatrix pose;
    public double               tX, tY, tZ, rX, rY, rZ;

    /** Constructor.
     * Create an instance of the class.
     * @param hMap HardwareMap object.
     * @param assetName Name of the asset file containing the VuMark definition.
     * @param includeViewer True to display camera viewer on RC phone.
     * @param camera Front or Back camera choice.
     */
    public VuMarkFinder(com.qualcomm.robotcore.hardware.HardwareMap hMap,
                        String assetName,
                        boolean includeViewer,
                        VuforiaLocalizer.CameraDirection camera)
    {
        VuforiaLocalizer.Parameters parameters;

            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
             */

        if (includeViewer)
        {
            int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        else
            // OR...  Do Not Activate the Camera Monitor View, to save power
            parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeVoNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzhfmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7ADigF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDirMkTdwPbfBxUYkkCBGUDQMtYstBS58G";

            /*
             * We also indicate which camera on the RC that we wish to use.
             * Here we chose the back (HiRes) camera (for greater range), but
             * for a competition robot, the front camera might be more convenient.
             */
        parameters.cameraDirection = camera;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /*
             * Load the data set containing the VuMark. This code supports 1 VuMark.
             */
        trackables = vuforia.loadTrackablesFromAsset(assetName);
        template = trackables.get(0);
        template.setName(assetName); // can help in debugging; otherwise not necessary
    }

    /**
     * Activate VuForia image processing. Call after waitForStart().
     */
    public void activate()
    {
        trackables.activate();
    }

    /**
     * Call to find out if VuMark is visible to the phone camera.
     * @return True if VuMark found, false if not.
     */
    public boolean findVuMark()
    {
        // See if any of the instances of the template are currently visible.
        instanceId = ((VuforiaTrackableDefaultListener) template.getListener()).getVuMarkInstanceId();

        if (instanceId != null)
        {
            pose = ((VuforiaTrackableDefaultListener) template.getListener()).getPose();

            if (pose != null)
            {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                tX = trans.get(0);
                tY = trans.get(1);
                tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                rX = rot.firstAngle;
                rY = rot.secondAngle;
                rZ = rot.thirdAngle;
            }

            return true;
        }
        else
        {
            pose = null;
            return false;
        }
    }

    /**
     * Format pose object for human viewing.
     * @param pose Pose object returned when VuMark is found.
     * @return Pose description.
     */
    String formatPose(OpenGLMatrix pose)
    {
        return (pose != null) ? pose.formatAsTransform() : "null";
    }
}