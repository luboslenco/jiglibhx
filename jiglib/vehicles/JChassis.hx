package jiglib.vehicles;


import jiglib.geometry.JBox;
import jiglib.plugin.ISkin3D;

class JChassis extends JBox
{
    public var car(get, never) : JCar;

    
    private var _car : JCar;
    
    public function new(car : JCar, skin : ISkin3D, width : Float = 40, depth : Float = 70, height : Float = 30)
    {
        super(skin, width, depth, height);
        
        _car = car;
    }
    
    private function get_car() : JCar
    {
        return _car;
    }
    
    override public function postPhysics(dt : Float) : Void
    {
        super.postPhysics(dt);
        _car.addExternalForces(dt);
        _car.postPhysics(dt);
    }
}

