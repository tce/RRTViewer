package us.ihmc.viewers;


import peasy.PeasyCam;
import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PImage;

/**
 * The Class PickerViewPApplet is for debugging only; it allows developers to see the offscreen buffer
 * used for picking objects in the 3D scene.
 */
public class PickerViewPApplet  extends PApplet
{

    /** The offscreen graphics. */
    PGraphics remoteGraphics;

    /** The offscreen camera. */
    PeasyCam remoteCamera;

    /** The cam. */
    PeasyCam localCamera;

    /** The img. */
    PImage pickerImage;

    /**
     * Instantiates a new picker view p applet.
     *
     * @param graphics the graphics
     * @param camera the camera
     */
    public PickerViewPApplet(PGraphics graphics, PeasyCam camera)
    {
        this.remoteGraphics = graphics;
        this.remoteCamera = camera;
    }

    /* (non-Javadoc)
      * @see processing.core.PApplet#setup()
      */
    public void setup()
    {
        size(remoteGraphics.width, remoteGraphics.height, P3D);
        frameRate(15);
        localCamera = new PeasyCam(this, remoteCamera.getDistance());
    }

    /* (non-Javadoc)
      * @see processing.core.PApplet#draw()
      */
    public void draw()
    {
        localCamera.setState(remoteCamera.getState());
        remoteGraphics.loadPixels();
        pickerImage = remoteGraphics.get(0, 0, remoteGraphics.width, remoteGraphics.height);
        g.set(0, 0, pickerImage);
    }
}
