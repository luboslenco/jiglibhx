package jiglib.vehicles;

import jiglib.vehicles.JChassis;
import jiglib.vehicles.JWheel;

import jiglib.math.JNumber3D;
import jiglib.math.Vector3D;
import jiglib.physics.PhysicsSystem;
import jiglib.plugin.ISkin3D;

class JCar
{
    public var chassis(get, never) : JChassis;
    public var wheels(get, never) : Map<String, JWheel>;

    
    private var _maxSteerAngle : Float;
    private var _steerRate : Float;
    private var _driveTorque : Float;
    
    private var _destSteering : Float;
    private var _destAccelerate : Float;
    
    private var _steering : Float;
    private var _accelerate : Float;
    private var _HBrake : Float;
    
    private var _chassis : JChassis;
    private var _wheels : Map<String, JWheel>;
    private var _steerWheels : Map<String, JWheel>;
    
    public function new(skin : ISkin3D)
    {
        _chassis = new JChassis(this, skin);
        _wheels = new Map<String, JWheel>();
        _steerWheels = new Map<String, JWheel>();
        _destSteering = _destAccelerate = _steering = _accelerate = _HBrake = 0;
        setCar();
    }
    
    /*
		 * maxSteerAngle: the max steer angle
		 * steerRate: the flexibility of steer
		 * driveTorque: the max torque of wheel
		 */
    public function setCar(maxSteerAngle : Float = 45, steerRate : Float = 1, driveTorque : Float = 500) : Void
    {
        _maxSteerAngle = maxSteerAngle;
        _steerRate = steerRate;
        _driveTorque = driveTorque;
    }
    
    /*
		 * _name: name of wheel
		 * pos: position of wheel relative to the car's center
		 * wheelSideFriction: side friction
		 * wheelFwdFriction: forward friction
		 * wheelTravel: suspension travel upwards
		 * wheelRadius: wheel radius
		 * wheelRestingFrac: elasticity coefficient
		 * wheelDampingFrac: suspension damping
		 */
    public function setupWheel(_name : String, pos : Vector3D,
            wheelSideFriction : Float = 2, wheelFwdFriction : Float = 2,
            wheelTravel : Float = 3, wheelRadius : Float = 10,
            wheelRestingFrac : Float = 0.5, wheelDampingFrac : Float = 0.5,
            wheelNumRays : Int = 1) : Void
    {
        var mass : Float = _chassis.mass;
        var mass4 : Float = 0.25 * mass;
        
        var gravity : Vector3D = PhysicsSystem.getInstance().gravity.clone();
        var gravityLen : Float = PhysicsSystem.getInstance().gravity.length;
        gravity.normalize();
        var axis : Vector3D = JNumber3D.getScaleVector(gravity, -1);
        var spring : Float = mass4 * gravityLen / (wheelRestingFrac * wheelTravel);
        var inertia : Float = 0.015 * wheelRadius * wheelRadius * mass;
        var damping : Float = 2 * Math.sqrt(spring * mass);
        damping *= (0.25 * wheelDampingFrac);
        
        _wheels[_name] = new JWheel(this);
        _wheels[_name].setup(pos, axis, spring, wheelTravel, inertia,
                wheelRadius, wheelSideFriction, wheelFwdFriction,
                damping, wheelNumRays);
    }
    
    private function get_chassis() : JChassis
    {
        return _chassis;
    }
    
    private function get_wheels() : Map<String, JWheel>
    {
        return _wheels;
    }
    
    public function setAccelerate(val : Float) : Void
    {
        _destAccelerate = val;
    }
    
    public function setSteer(wheels : Array<String>, val : Float) : Void
    {
        _destSteering = val;
        _steerWheels = new Map<String, JWheel>();
        for (wheelname in wheels)
        {
            if (findWheel(wheelname)) 
            {
                _steerWheels[wheelname] = _wheels[wheelname];
            }
        }
    }
    
    private function findWheel(_name : String) : Bool
    {
		return _wheels.exists(_name);
    }
    
    public function setHBrake(val : Float) : Void
    {
        _HBrake = val;
    }
    
    public function addExternalForces(dt : Float) : Void
    {
        for (wheel in wheels)
        {
            wheel.addForcesToCar(dt);
        }
    }
    
    // Update stuff at the end of physics
    public function postPhysics(dt : Float) : Void
    {
        var wheel : JWheel;
        for (wheel in wheels)
        {
            wheel.update(dt);
        }
        
        var deltaAccelerate : Float;
        var deltaSteering : Float;
        var dAccelerate : Float;
        var dSteering : Float;
        var alpha : Float;
        var angleSgn : Float;
        deltaAccelerate = dt;
        deltaSteering = dt * _steerRate;
        dAccelerate = _destAccelerate - _accelerate;
        if (dAccelerate < -deltaAccelerate) 
        {
            dAccelerate = -deltaAccelerate;
        }
        else if (dAccelerate > deltaAccelerate) 
        {
            dAccelerate = deltaAccelerate;
        }
        _accelerate += dAccelerate;
        
        dSteering = _destSteering - _steering;
        if (dSteering < -deltaSteering) 
        {
            dSteering = -deltaSteering;
        }
        else if (dSteering > deltaSteering) 
        {
            dSteering = deltaSteering;
        }
        _steering += dSteering;
        
        for (wheel in wheels)
        {
            wheel.addTorque(_driveTorque * _accelerate);
            wheel.setLock(_HBrake > 0.5);
        }
        
        alpha = Math.abs(_maxSteerAngle * _steering);
        angleSgn = ((_steering > 0)) ? 1 : -1;
        for (_steerWheel in _steerWheels)
        {
            _steerWheel.setSteerAngle(angleSgn * alpha);
        }
    }
    
    public function getNumWheelsOnFloor() : Int
    {
        var count : Int = 0;
        for (wheel in wheels)
        {
            if (wheel.getOnFloor()) 
            {
                count++;
            }
        }
        return count;
    }
}


