/**
 * Created with IntelliJ IDEA.
 * User: teskridge
 * Date: 8/24/12
 * Time: 11:48 PM
 * To change this template use File | Settings | File Templates.
 */
package us.ihmc.viewers;

import gov.nasa.arc.sirca.path.ExtState;
import peasy.CameraState;
import peasy.PeasyCam;
import peasy.PeasyDragHandler;
import peasy.PeasyWheelHandler;
import peasy.org.apache.commons.math.geometry.Rotation;
import peasy.org.apache.commons.math.geometry.Vector3D;
import picking.Picker;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PFont;
import us.ihmc.planning.world.Obstacle;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Timer;

public class RouteViewer3D extends PApplet
{
    private static final boolean DEBUG_PICKER            = false;
    private final float defaultPointRadius = 10;
    private final int startPointPickerId = 1000;
    private final int            startProjectionPickerId = 10000;

    PeasyCam                             _camera;
    /** The handlers for controlling the camera with the mouse */
    PeasyDragHandler                     _panHandler, _rotateHandler, _zoomHandler;

    /** The _wheel handler controlling the camera zoom in/out */
    PeasyWheelHandler                    _wheelHandler;

    ArrayList<RoutePoint> _selectedPoints = new ArrayList<RoutePoint>();
    Picker                               _picker;
    int                                  _axisLength              = 1000;
    int                                  _cameraControlMultiplier = 1;

    Timer                                _moveTimer               = null;
    Timer                                _changeTimer             = null;

    ArrayList<RoutePoint> _points;
    ArrayList<Obstacle>   _obstacles;

    int                                  maxSpineLength           = 300;

    int                                  _selectedProjection      = -1;

    int negxColor          = color(180, 180, 40, 160);
    int posxColor          = color(180, 115, 35, 160);
    int negyColor          = color(180, 25, 25, 160);
    int posyColor          = color(40, 150, 26, 160);
    int negzColor          = color(70, 100, 185, 160);
    int poszColor          = color(145, 65, 150, 160);

    int defaultSphereColor = color(20,40,40,200);//color(128, 128, 128, 220); // color(95, 95, 120, 200);
    int selectedColor      = color(33, 133, 12, 220);
    private float changeScale = 0.25f;

    PFont pointFont = createFont("Arial Black", 10, true);

    public void setup()
    {
        Dimension d = Toolkit.getDefaultToolkit().getScreenSize();

        // General preferences
        size(d.width, d.height, P3D);
        hint(ENABLE_NATIVE_FONTS);
        frameRate(30);
        textureMode(NORMALIZED);

        // User controlled camera
        setupCamera();

        // Shape selection handling
        setupShapePicker();

        // Objects
        _points = new ArrayList<RoutePoint>();

/*        RoutePoint root = randomPoint("root",null);
        for (int i = 0; i < 1000; i++ )
        {
            RoutePoint newPoint = randomPoint("p"+i,null);
            RoutePoint closestPoint = findClosest(newPoint);
            newPoint.setParent( closestPoint );
            _points.add( newPoint );
        }
*/
    }

    RoutePoint findClosest(RoutePoint srcPt )
    {
        RoutePoint closestPoint = null;
        double closestDistance = 99999.0;

        for (RoutePoint rpt : _points )
        {
            double dist = srcPt.distanceTo(rpt);
            if ( dist < closestDistance )
            {
                closestDistance = dist;
                closestPoint = rpt;
            }
        }

        return closestPoint;
    }

    public void draw()
    {
        background(0);
        drawAxis();
        drawRoutePoints();

        textMode(PConstants.SCREEN);
        textFont(createFont("Arial Black", 12, true));
        stroke(255, 255, 255, 255);
        fill(255, 255, 255, 255);
        float[] camPos = _camera.getPosition();
        float[] camRot = _camera.getRotations();
        //text("x:" + camPos[0] + " y:" + camPos[1] + " z:" + camPos[2], 20f, 20f);
        //text("xr:" + camRot[0] + " yr:" + camRot[1] + " zr:" + camRot[2], 20f, 40f);
        text((int)frameRate + "fps " + _points.size() + " nodes", 20f, 60f);
    }


    protected RoutePoint randomPoint(String name, RoutePoint parent)
    {
        float halfLength = _axisLength / 2;
        ExtState randomExtState = new ExtState(random(- halfLength, halfLength ),
                                      random(- halfLength, halfLength ),
                                      random(- halfLength, halfLength ),
                                      random(0,180),
                                      random(0,360));
        RoutePoint s1 = new RoutePoint(name, randomExtState, defaultPointRadius, defaultSphereColor, parent);

        return s1;
    }

    private void setupCamera()
    {
        // User controlled camera
        _camera = new PeasyCam(this, _axisLength);
        // _camera.setResetOnDoubleClick(false);
        _camera.setMouseControlled(true);

        _panHandler = _camera.getPanDragHandler();
        _rotateHandler = _camera.getRotateDragHandler();
        _zoomHandler = _camera.getZoomDragHandler();
        _wheelHandler = _camera.getZoomWheelHandler();

        final PeasyDragHandler panHandler = new PeasyDragHandler()
        {
            public void handleDrag(final double dx, final double dy)
            {
                if (_selectedPoints.size() == 0)
                    _panHandler.handleDrag(dx * 2, dy * 2);
            }
        };
        _camera.setRightDragHandler(panHandler);

        final PeasyDragHandler rotateHandler = new PeasyDragHandler()
        {
            final double dragMaxDelta = 30.0;

            public void handleDrag(final double dx, final double dy)
            {
                if (dx > dragMaxDelta || dx < -dragMaxDelta || dy > dragMaxDelta || dy < -dragMaxDelta)
                {
                    // ignore because it's too fast
                    return;
                }
                if (_selectedPoints.size() == 0)
                    _rotateHandler.handleDrag(dx * _cameraControlMultiplier, dy * _cameraControlMultiplier);
            }
        };
        _camera.setLeftDragHandler(rotateHandler);

        final PeasyDragHandler zoomHandler = new PeasyDragHandler()
        {
            public void handleDrag(final double dx, final double dy)
            {
                if (_selectedPoints.size() == 0)
                    _zoomHandler.handleDrag(dx * _cameraControlMultiplier, dy * _cameraControlMultiplier);
            }
        };
        _camera.setCenterDragHandler(zoomHandler);

        final PeasyWheelHandler zoomWheelHandler = new PeasyWheelHandler()
        {
            public void handleWheel(final int delta)
            {
                if (_selectedPoints.size() == 0)
                    _wheelHandler.handleWheel(delta * _cameraControlMultiplier);
                else
                {
                    for (RoutePoint sp : _selectedPoints)
                    {
                        sp._Ext_state.setZ(sp._Ext_state.getZ() + delta*3);
                    }
                }
            }
        };
        _camera.setWheelHandler(zoomWheelHandler);

        Vector3D center = new Vector3D(1, 1, 0);
        Rotation rotate = new Rotation(center, 0.0f);
        CameraState state = new CameraState(rotate, center, _camera.getDistance());
        _camera.setState(state, 1000);
    }

    private void setupShapePicker()
    {
        _picker = new Picker(this);
        if (DEBUG_PICKER)
        {
            JFrame pickerFrame = new JFrame("Picker");
            pickerFrame.setPreferredSize(this.getPreferredSize());
            pickerFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

            JScrollPane pickerScrollPane = new JScrollPane();
            pickerFrame.getContentPane().add(pickerScrollPane);

            PickerViewPApplet pva = new PickerViewPApplet(_picker.getBuffer(), _camera);
            pickerScrollPane.add(pva);
            pva.init();

            pickerFrame.pack();
            pickerFrame.setVisible(true);
        }
    }

    private void drawAxis()
    {
        pushMatrix();
        // Center and spin grid
        translate(-_axisLength / 2, -_axisLength / 2, -_axisLength / 2);
        noFill();

        // x (white)
        if (key == 'x')
            strokeWeight(3.0f);
        else
            strokeWeight(1.0f);
        stroke(color(255, 255, 255, 100));
        line(0, 0, 0, _axisLength, 0, 0);

        // y (blue)
        if (key == 'y')
            strokeWeight(3.0f);
        else
            strokeWeight(1.0f);
        stroke(color(100, 255, 255, 100));
        line(0, 0, 0, 0, _axisLength, 0);

        // z (red)
        if (key == 'z')
            strokeWeight(3.0f);
        else
            strokeWeight(1.0f);
        stroke(color(255, 100, 100, 100));
        line(0, 0, 0, 0, 0, _axisLength);
        popMatrix();
    }

    private void drawRoutePoints()
    {
        int pickIdx = startPointPickerId;
        ArrayList<RoutePoint> pointsToDraw = (ArrayList<RoutePoint>) _points.clone();
        for (RoutePoint sphere : pointsToDraw)
        {
            sphere.draw(pickIdx);
            pickIdx++;
        }
    }

    static public void main(String args[])
    {
        PApplet.main(new String[] { "--bgcolor=#000000", "us.ihmc.viewers.RouteViewer3D" });
    }

    public class RoutePointSAV
    {
        RoutePointSAV sPosX, sPosY, sPosZ, sNegX, sNegY, sNegZ;
        {
            sPosX = sPosY = sPosZ = sNegX = sNegY = sNegZ = null;
        }

        String                 _name;
        float                  _x, _y, _z, _speed, _heading, _radius;
        int                    _originalColor, _rgbColor;
        RoutePointSAV             _parent;
        boolean                _onPath = false;

        public RoutePointSAV(String name, float x, float y, float z, float as, float hdg, float radius, int rgbColor, RoutePointSAV parent)
        {
            _name = name;
            _x = x;
            _y = y;
            _z = z;
            _speed = as;
            _heading = hdg;
            _radius = radius;
            _rgbColor = _originalColor = rgbColor;
            _parent = parent;
        }

        public RoutePointSAV(RoutePointSAV origPoint)
        {
            _name = origPoint._name;
            _x = origPoint._x;
            _y = origPoint._y;
            _z = origPoint._z;
            _speed = origPoint._speed;
            _heading = origPoint._heading;
            _radius = origPoint._radius;
            _rgbColor = _originalColor = origPoint._originalColor;
            _parent = origPoint._parent;
        }

        public void draw(int pickId)
        {
            pushMatrix();
            translate(_x, _y, _z);
            noStroke();
            fill(_rgbColor);
            sphereDetail(10);
            _picker.start(pickId);
            sphere(_radius);
           /*
            textMode(PConstants.MODEL);

            textFont(pointFont);
            stroke(255, 255, 255, 255);
            fill(155, 255, 255, 155);
            float[] camPosition = _camera.getPosition();
            float[] camRotations = _camera.getRotations();
            rotateX(camRotations[0]);
            rotateY(camRotations[1]);
            rotateZ(camRotations[2]);
            translate(-textWidth(_name)/2,_radius+textAscent()+textDescent(),0);
            text(_name,0,0,0);
            */
            popMatrix();

            // draw the connecting lines
            if (_onPath )
            {
                stroke(0, 0, 255, 200);
                strokeWeight(3.0f);
            }
            else
            {
                stroke(255, 255, 255, 90);
                strokeWeight(1.0f);
            }


            if (_parent != null)
                line(_x, _y, _z, _parent._x, _parent._y, _parent._z);
            // TODO: draw heading and speed

            int lineAlpha = 100;
            if ((key == '7' || (keyPressed && key == '1')) && sPosX != null)
            {
                stroke(posxColor, lineAlpha);
                line(_x + defaultPointRadius, _y, _z, sPosX._x + defaultPointRadius, sPosX._y, sPosX._z);
            }
            if ((key == '7' || (keyPressed && key == '2')) && sPosY != null)
            {
                stroke(posyColor, lineAlpha);
                line(_x, _y + defaultPointRadius, _z, sPosY._x, sPosY._y + defaultPointRadius, sPosY._z);
            }
            if ((key == '7' || (keyPressed && key == '3')) && sPosZ != null)
            {
                stroke(poszColor, lineAlpha);
                line(_x, _y, _z + defaultPointRadius, sPosZ._x, sPosZ._y, sPosZ._z + defaultPointRadius);
            }
            if ((key == '7' || (keyPressed && key == '4')) && sNegX != null)
            {
                stroke(negxColor, lineAlpha);
                line(_x - defaultPointRadius, _y, _z, sNegX._x - defaultPointRadius, sNegX._y, sNegX._z);
            }
            if ((key == '7' || (keyPressed && key == '5')) && sNegY != null)
            {
                stroke(negyColor, lineAlpha);
                line(_x, _y - defaultPointRadius, _z, sNegY._x, sNegY._y - defaultPointRadius, sNegY._z);
            }
            if ((key == '7' || (keyPressed && key == '6')) && sNegZ != null)
            {
                stroke(negzColor, lineAlpha);
                line(_x, _y, _z - defaultPointRadius, sNegZ._x, sNegZ._y, sNegZ._z - defaultPointRadius);
            }
        }

        public void move()
        {
            if (dragging && _selectedPoints.contains(this))
                return;


            ArrayList<RoutePointSAV> intersectingSpheres = new ArrayList<RoutePointSAV>();

            // find most similar spheres in each of 6 dimensions
/*
            for (RoutePointSAV curSphere : _points)
            {
                if (curSphere == this)
                    continue;

                if (dist(_x, _y, _z, curSphere._x, curSphere._y, curSphere._z) < defaultPointRadius * 2)
                {
                    intersectingSpheres.add(curSphere);
                }

            }
*/

            int intersectMoveDelta = 2;
            int neighborMoveDelta = 2;
            // move away from intersecting sphere
            if (intersectingSpheres.size() > 0)
            {
                for (RoutePointSAV intersectingSphere : intersectingSpheres)
                {
                    if (this._x < intersectingSphere._x)
                    {
                        this._x -= intersectMoveDelta;
                    }
                    else if (this._x > intersectingSphere._x)
                    {
                        this._x += intersectMoveDelta;
                    }
                    if (this._y < intersectingSphere._y)
                    {
                        this._y -= intersectMoveDelta;
                    }
                    else if (this._y > intersectingSphere._y)
                    {
                        this._y += intersectMoveDelta;
                    }
                    if (this._z < intersectingSphere._z)
                    {
                        this._z -= intersectMoveDelta;
                    }
                    else if (this._z > intersectingSphere._z)
                    {
                        this._z += intersectMoveDelta;
                    }
                }
            }
    }

        public double distanceTo(RoutePointSAV rpt)
        {
            double xdif = _x - rpt._x;
            xdif *= xdif;
            double ydif = _y - rpt._y;
            ydif *= ydif;
            double zdif = _z - rpt._z;
            zdif *= zdif;
            double sdif = _speed - rpt._speed;
            sdif *= sdif;
            double hdif = _heading - rpt._heading;
            hdif *= hdif;

            return Math.sqrt(xdif + ydif + zdif + sdif + hdif);
        }

        public void setParent(RoutePointSAV closestPoint)
        {
            _parent = closestPoint;
        }
    }

    public class RoutePoint
    {
        RoutePoint sPosX, sPosY, sPosZ, sNegX, sNegY, sNegZ;
        {
            sPosX = sPosY = sPosZ = sNegX = sNegY = sNegZ = null;
        }

        String                 _name;
        ExtState _Ext_state;
        //    float                  _x, _y, _z, _speed, _heading;
        float                  _radius;
        int                    _originalColor, _rgbColor;
        RoutePoint             _parent;
        boolean                _onPath = false;
        boolean                _showText = false;

        boolean                _goal = false;
        boolean                _start = false;

        public RoutePoint(String name, ExtState extState, float radius, int rgbColor, RoutePoint parent)
        {
            _name = name;
            _Ext_state = extState;
            _radius = radius;
            _rgbColor = _originalColor = rgbColor;
            _parent = parent;
        }

        public RoutePoint(RoutePoint origPoint)
        {
            _name = origPoint._name;
            _Ext_state = (ExtState) origPoint._Ext_state;
            _radius = origPoint._radius;
            _rgbColor = _originalColor = origPoint._originalColor;
            _parent = origPoint._parent;
        }

        public void draw(int pickId)
        {
            pushMatrix();
            translate(_Ext_state.getXf(), _Ext_state.getYf(), _Ext_state.getZf());
            noStroke();
            fill(_rgbColor);
            sphereDetail(10);
            _picker.start(pickId);
//            if ( _Ext_state.interior )
//                sphere( 3 );            // make a very small sphere if this is an interior piece
//            else
            if ( ! _Ext_state.interior )
            {
                sphere(_radius);

                if ( _showText )
                {
                    textMode(PConstants.MODEL);

                    textFont(pointFont);
                    stroke(255, 255, 255, 255);
                    fill(155, 255, 255, 155);
                    float[] camPosition = _camera.getPosition();
                    float[] camRotations = _camera.getRotations();
                    rotateX(camRotations[0]);
                    rotateY(camRotations[1]);
                    rotateZ(camRotations[2]);
                    translate(-textWidth(_name)/2,_radius+textAscent()+textDescent(),0);
                    text(_name,0,0,0);
                }
            }
            popMatrix();

            // draw the connecting lines
            if (_onPath )
            {
                stroke(0, 0, 255, 200);
                strokeWeight(3.0f);
            }
            else
            {
                stroke(255, 255, 255, 90);
                strokeWeight(1.0f);
            }


            if (_parent != null)
                line(_Ext_state.getXf(), _Ext_state.getYf(), _Ext_state.getZf(), _parent._Ext_state.getXf(),_parent._Ext_state.getYf(),_parent._Ext_state.getZf());

            // TODO: draw heading and speed

            int lineAlpha = 100;
            if ((key == '7' || (keyPressed && key == '1')) && sPosX != null)
            {
                stroke(posxColor, lineAlpha);
                line(_Ext_state.getXf() + defaultPointRadius, _Ext_state.getYf(), _Ext_state.getZf(),
                      sPosX._Ext_state.getXf() + defaultPointRadius, sPosX._Ext_state.getYf(), sPosX._Ext_state.getZf());
            }
            if ((key == '7' || (keyPressed && key == '2')) && sPosY != null)
            {
                stroke(posyColor, lineAlpha);
                line(_Ext_state.getXf(), _Ext_state.getYf() + defaultPointRadius, _Ext_state.getZf(),
                     sPosX._Ext_state.getXf(), sPosX._Ext_state.getYf() + defaultPointRadius, sPosX._Ext_state.getZf());
            }
            if ((key == '7' || (keyPressed && key == '3')) && sPosZ != null)
            {
                stroke(poszColor, lineAlpha);
                line(_Ext_state.getXf(), _Ext_state.getYf(), _Ext_state.getZf() + defaultPointRadius,
                     sPosX._Ext_state.getXf(), sPosX._Ext_state.getYf(), sPosX._Ext_state.getZf() + defaultPointRadius);

            }
            if ((key == '7' || (keyPressed && key == '4')) && sNegX != null)
            {
                stroke(negxColor, lineAlpha);
                line(_Ext_state.getXf() - defaultPointRadius, _Ext_state.getYf(), _Ext_state.getZf(),
                     sNegX._Ext_state.getXf() - defaultPointRadius, sNegX._Ext_state.getYf(), sNegX._Ext_state.getZf());
            }
            if ((key == '7' || (keyPressed && key == '5')) && sNegY != null)
            {
                stroke(negyColor, lineAlpha);
                line(_Ext_state.getXf(), _Ext_state.getYf() - defaultPointRadius, _Ext_state.getZf(),
                     sNegX._Ext_state.getXf(), sNegX._Ext_state.getYf() - defaultPointRadius, sNegX._Ext_state.getZf());
            }
            if ((key == '7' || (keyPressed && key == '6')) && sNegZ != null)
            {
                stroke(negzColor, lineAlpha);
                line(_Ext_state.getXf(), _Ext_state.getYf(), _Ext_state.getZf() - defaultPointRadius,
                     sNegX._Ext_state.getXf(), sNegX._Ext_state.getYf(), sNegX._Ext_state.getZf() - defaultPointRadius);
            }
        }

        public void move()
        {
            if (dragging && _selectedPoints.contains(this))
                return;


            ArrayList<RoutePoint> intersectingSpheres = new ArrayList<RoutePoint>();

            // find most similar spheres in each of 6 dimensions
            for (RoutePoint curSphere : _points)
            {
                if (curSphere == this)
                    continue;

                if (dist(_Ext_state.getXf(), _Ext_state.getYf(), _Ext_state.getZf(), curSphere._Ext_state.getXf(), curSphere._Ext_state.getYf(), curSphere._Ext_state.getZf()) < defaultPointRadius * 2)
                {
                    intersectingSpheres.add(curSphere);
                }

            }

            int intersectMoveDelta = 2;
            int neighborMoveDelta = 2;
            // move away from intersecting sphere
            if (intersectingSpheres.size() > 0)
            {
                for (RoutePoint intersectingSphere : intersectingSpheres)
                {
                    if (_Ext_state.getX() < intersectingSphere._Ext_state.getX())
                    {
                        _Ext_state.setX(_Ext_state.getX() - intersectMoveDelta);
                    }
                    else if (_Ext_state.getX() > intersectingSphere._Ext_state.getX())
                    {
                        _Ext_state.setX( _Ext_state.getX() + intersectMoveDelta);
                    }
                    if (_Ext_state.getY() < intersectingSphere._Ext_state.getY())
                    {
                        _Ext_state.setY( _Ext_state.getY() - intersectMoveDelta);
                    }
                    else if (_Ext_state.getY() > intersectingSphere._Ext_state.getY())
                    {
                        _Ext_state.setY( _Ext_state.getY() + intersectMoveDelta);
                    }
                    if (_Ext_state.getZ() < intersectingSphere._Ext_state.getZ())
                    {
                        _Ext_state.setZ( _Ext_state.getZ() - intersectMoveDelta);
                    }
                    else if (_Ext_state.getZ() > intersectingSphere._Ext_state.getZ())
                    {
                        _Ext_state.setZ( _Ext_state.getZ() + intersectMoveDelta);
                    }
                }
            }
        }

        public double distanceTo(RoutePoint rpt)
        {
            return _Ext_state.distance(rpt._Ext_state);
        }

        public void setParent(RoutePoint closestPoint)
        {
            _parent = closestPoint;
        }
    }

    boolean dragging = false;

    public void mousePressed()
    {
        dragging = false;
        super.mousePressed();
        int pickid = _picker.get(mouseX, mouseY);
        if (pickid == -1)
        {
            if (_selectedPoints.size() > 0)
            {
                for (RoutePoint sp : _selectedPoints)
                {
                    sp._rgbColor = sp._originalColor;
                }
            }
            clearSelectedPoints();
        }
        else if (pickid >= startPointPickerId)
        {
            if (pickid - startPointPickerId < _points.size())
            {
                RoutePoint sp = _points.get(pickid - startPointPickerId);
                selectPoint(sp);
            }
        }
    }

    protected void selectPoint(RoutePoint sp)
    {
        sp._rgbColor = selectedColor;
        _selectedPoints.add(sp);
        setPath(sp,true);
    }

    private void setPath(RoutePoint sp, boolean val)
    {
        RoutePoint cur = sp;
        while ( cur._parent != null)
        {
            cur._onPath = val;
            cur = cur._parent;
        }
    }

    private void clearSelectedPoints()
    {
        if (_selectedPoints.size() > 0)
        {
            for (RoutePoint sp : _selectedPoints)
            {
                sp._rgbColor = sp._originalColor;
                setPath(sp,false);
            }
            _selectedPoints.clear();
        }
    }

    public void mouseDragged()
    {
        dragging = true;
//        if (_selectedPoints.size() == 0)
        if ( true )
        {
            super.mouseDragged();
            return;
        }

        float diffx = mouseX - pmouseX;
        float diffy = mouseY - pmouseY;

        if (keyPressed && key == 'x')
        {
            for (RoutePoint sp : _selectedPoints)
                sp._Ext_state.setX( sp._Ext_state.getX() + diffx);
        }
        else if (keyPressed && key == 'y')
        {
            for (RoutePoint sp : _selectedPoints)
                sp._Ext_state.setY( sp._Ext_state.getY() + diffy);
        }
        else if (keyPressed && key == 'z')
        {
            for (RoutePoint sp : _selectedPoints)
                sp._Ext_state.setZ( sp._Ext_state.getZ() + diffx);
        }
        else
        {
            for (RoutePoint sp : _selectedPoints)
                sp._Ext_state.setX( sp._Ext_state.getX() + diffx);
            for (RoutePoint sp : _selectedPoints)
                sp._Ext_state.setY( sp._Ext_state.getY() + diffy);
            _camera.getLookAt();
            _camera.getRotations();
        }
    }

    public void keyPressed()
    {
        if (key == 'x')
        {
            _camera.rotateX(0.03);
        }
        if (key == 'X')
        {
            _camera.rotateX(-0.03);
        }
        if (key == 'y')
        {
            _camera.rotateY(0.03);
        }
        if (key == 'Y')
        {
            _camera.rotateY(-0.03);
        }
        if (key == 'z')
        {
            _camera.rotateZ(0.03);
        }
        if (key == 'Z')
        {
            _camera.rotateZ(-0.03);
        }
    }

    public void mouseReleased()
    {
        if (_selectedPoints.size() == 0)
        {
            super.mouseReleased();
            return;
        }
        if (dragging)
        {
//            setupShapeMoveTimer(100);
            dragging = false;
        }
    }
}
