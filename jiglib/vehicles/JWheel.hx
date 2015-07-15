package jiglib.vehicles;

import jiglib.collision.CollisionSystemAbstract;
import jiglib.data.CollOutBodyData;
import jiglib.geometry.JSegment;
import jiglib.math.*;
import jiglib.physics.PhysicsSystem;
import jiglib.physics.RigidBody;

class JWheel
{
    
    private static inline var noslipVel : Float = 0.2;
    private static inline var slipVel : Float = 0.4;
    private static inline var slipFactor : Float = 0.7;
    private static inline var smallVel : Float = 3;
    
    private var _car : JCar;
    private var _pos : Vector3D;
    private var _axisUp : Vector3D;
    private var _spring : Float;
    private var _travel : Float;
    private var _inertia : Float;
    private var _radius : Float;
    private var _sideFriction : Float;
    private var _fwdFriction : Float;
    private var _damping : Float;
    private var _numRays : Int;
    
    private var _angVel : Float;
    private var _steerAngle : Float;
    private var _torque : Float;
    private var _driveTorque : Float;
    private var _axisAngle : Float;
    private var _displacement : Float;
    private var _upSpeed : Float;
    private var _rotDamping : Float;
    
    private var _locked : Bool;
    private var _lastDisplacement : Float;
    private var _lastOnFloor : Bool;
    private var _angVelForGrip : Float;
    
    private var worldPos : Vector3D;
    private var worldAxis : Vector3D;
    private var wheelFwd : Vector3D;
    private var wheelUp : Vector3D;
    private var wheelLeft : Vector3D;
    private var wheelRayEnd : Vector3D;
    private var wheelRay : JSegment;
    private var groundUp : Vector3D;
    private var groundLeft : Vector3D;
    private var groundFwd : Vector3D;
    private var wheelPointVel : Vector3D;
    private var rimVel : Vector3D;
    private var worldVel : Vector3D;
    private var wheelCentreVel : Vector3D;
    
    // proxy for CollisionSystem, avoid calling singleton every time in loop
    private var _collisionSystem : CollisionSystemAbstract;
    
    public function new(car : JCar)
    {
        _car = car;
    }
    
    /*
		 * pos: position relative to car, in car's space
		 * axisUp: in car's space
		 * spring: force per suspension offset
		 * travel: suspension travel upwards
		 * inertia: inertia about the axel
		 * radius: wheel radius
		 */
    public function setup(pos : Vector3D, axisUp : Vector3D,
            spring : Float = 0, travel : Float = 0,
            inertia : Float = 0, radius : Float = 0,
            sideFriction : Float = 0, fwdFriction : Float = 0,
            damping : Float = 0, numRays : Int = 0) : Void
    {
        _pos = pos;
        _axisUp = axisUp;
        _spring = spring;
        _travel = travel;
        _inertia = inertia;
        _radius = radius;
        _sideFriction = sideFriction;
        _fwdFriction = fwdFriction;
        _damping = damping;
        _numRays = numRays;
        reset();
    }
    
    // power
    public function addTorque(torque : Float) : Void
    {
        _driveTorque += torque;
    }
    
    // lock/unlock the wheel
    public function setLock(lock : Bool) : Void
    {
        _locked = lock;
    }
    
    public function setSteerAngle(steer : Float) : Void
    {
        _steerAngle = steer;
    }
    
    // get steering angle in degrees
    public function getSteerAngle() : Float
    {
        return _steerAngle;
    }
    
    public function getPos() : Vector3D
    {
        return _pos;
    }
    
    // the suspension axis in the car's frame
    public function getLocalAxisUp() : Vector3D
    {
        return _axisUp;
    }
    
    public function getActualPos() : Vector3D
    {
        return _pos.add(JNumber3D.getScaleVector(_axisUp, _displacement));
    }
    
    // wheel radius
    public function getRadius() : Float
    {
        return _radius;
    }
    
    // the displacement along our up axis
    public function getDisplacement() : Float
    {
        return _displacement;
    }
    
    public function getAxisAngle() : Float
    {
        return _axisAngle;
    }
    
    public function getRollAngle() : Float
    {
        return 0.1 * _angVel * 180 / Math.PI;
    }
    
    public function setRotationDamping(vel : Float) : Void{
        _rotDamping = vel;
    }
    
    public function getRotationDamping() : Float{
        return _rotDamping;
    }
    
    //if it's on the ground.
    public function getOnFloor() : Bool
    {
        return _lastOnFloor;
    }
    
    // Adds the forces die to this wheel to the parent. Return value indicates if it's on the ground.
    public function addForcesToCar(dt : Float) : Bool
    {
        var force : Vector3D = new Vector3D();
        _lastDisplacement = _displacement;
        _displacement = 0;
        
        var carBody : JChassis = _car.chassis;
        worldPos = carBody.currentState.orientation.transformVector(_pos);
        worldPos = carBody.currentState.position.add(worldPos);
        worldAxis = carBody.currentState.orientation.transformVector(_axisUp);
        
        wheelFwd = JMatrix3D.getRotationMatrix(worldAxis.x, worldAxis.y, worldAxis.z, _steerAngle).transformVector(carBody.currentState.getOrientationCols()[2]);
        wheelUp = worldAxis;
        wheelLeft = wheelUp.crossProduct(wheelFwd);
        wheelLeft.normalize();
        
        var rayLen : Float = 2 * _radius + _travel;
        wheelRayEnd = worldPos.subtract(JNumber3D.getScaleVector(worldAxis, _radius));
        wheelRay = new JSegment(wheelRayEnd.add(JNumber3D.getScaleVector(worldAxis, rayLen)), JNumber3D.getScaleVector(worldAxis, -rayLen));
        
        if (_collisionSystem == null) 
            _collisionSystem = PhysicsSystem.getInstance().getCollisionSystem();
        
        var maxNumRays : Int = 10;
        var numRays : Int = (_numRays < maxNumRays) ? _numRays : maxNumRays;
        
        var objArr : Array<CollOutBodyData> = [for (i in 0...numRays) null];
        var segments : Array<JSegment> = [for (i in 0...numRays) null];
        
        var deltaFwd : Float = (2 * _radius) / (numRays + 1);
        var deltaFwdStart : Float = deltaFwd;
        
        _lastOnFloor = false;
        
        var distFwd : Float;
        var yOffset : Float;
        var bestIRay : Int = 0;
        var iRay : Int = 0;
        var collOutBodyData : CollOutBodyData;
        var segment : JSegment;
        for (iRay in 0...numRays){
            collOutBodyData = objArr[iRay] = new CollOutBodyData();
            distFwd = (deltaFwdStart + iRay * deltaFwd) - _radius;
            yOffset = _radius * (1 - Math.cos(90 * (distFwd / _radius) * Math.PI / 180));
            segment = segments[iRay] = wheelRay.clone();
            segment.origin = segment.origin.add(JNumber3D.getScaleVector(wheelFwd, distFwd).add(JNumber3D.getScaleVector(wheelUp, yOffset)));
            if (_collisionSystem.segmentIntersect(collOutBodyData, segment, carBody)) 
            {
                _lastOnFloor = true;
                if (collOutBodyData.frac < objArr[bestIRay].frac) 
                {
                    bestIRay = iRay;
                }
            }
        }
        
        if (!_lastOnFloor) 
        {
            return false;
        }
        
        var frac : Float = objArr[bestIRay].frac;
        var groundPos : Vector3D = objArr[bestIRay].position;
        var otherBody : RigidBody = objArr[bestIRay].rigidBody;
        
        var groundNormal : Vector3D = worldAxis.clone();
        if (numRays > 1) 
        {
            for (iRay in 0...numRays){
                collOutBodyData = objArr[iRay];
                if (collOutBodyData.frac <= 1) 
                    groundNormal = groundNormal.add(JNumber3D.getScaleVector(worldPos.subtract(segments[iRay].getEnd()), 1 - collOutBodyData.frac));
            }
            
            groundNormal.normalize();
        }
        else 
        {
            groundNormal = objArr[bestIRay].normal;
        }
        
        _displacement = rayLen * (1 - frac);
        
        if (_displacement < 0) 
            _displacement = 0
        else if (_displacement > _travel) 
            _displacement = _travel;
        
        var displacementForceMag : Float = _displacement * _spring;
        displacementForceMag *= groundNormal.dotProduct(worldAxis);
        
        var dampingForceMag : Float = _upSpeed * _damping;
        var totalForceMag : Float = displacementForceMag + dampingForceMag;
        if (totalForceMag < 0) 
            totalForceMag = 0;
        
        var extraForce : Vector3D = JNumber3D.getScaleVector(worldAxis, totalForceMag);
        force = force.add(extraForce);
        
        groundUp = groundNormal;
        groundLeft = groundNormal.crossProduct(wheelFwd);
        groundLeft.normalize();
        groundFwd = groundLeft.crossProduct(groundUp);
        
        var tempv : Vector3D = carBody.currentState.orientation.transformVector(_pos);
        wheelPointVel = carBody.currentState.linVelocity.add(carBody.currentState.rotVelocity.crossProduct(tempv));
        
        rimVel = JNumber3D.getScaleVector(wheelLeft.crossProduct(groundPos.subtract(worldPos)), _angVel);
        wheelPointVel = wheelPointVel.add(rimVel);
        
        if (otherBody.movable) 
        {
            worldVel = otherBody.currentState.linVelocity.add(otherBody.currentState.rotVelocity.crossProduct(groundPos.subtract(otherBody.currentState.position)));
            wheelPointVel = wheelPointVel.subtract(worldVel);
        }
        
        var friction : Float = _sideFriction;
        var sideVel : Float = wheelPointVel.dotProduct(groundLeft);
        
        if ((sideVel > slipVel) || (sideVel < -slipVel)) 
            friction *= slipFactor
        else if ((sideVel > noslipVel) || (sideVel < -noslipVel)) 
            friction *= (1 - (1 - slipFactor) * (Math.abs(sideVel) - noslipVel) / (slipVel - noslipVel));
        
        if (sideVel < 0) 
        {
            friction *= -1;
        }
        if (Math.abs(sideVel) < smallVel) 
        {
            friction *= Math.abs(sideVel) / smallVel;
        }
        
        var sideForce : Float = -friction * totalForceMag;
        extraForce = JNumber3D.getScaleVector(groundLeft, sideForce);
        force = force.add(extraForce);
        
        friction = _fwdFriction;
        var fwdVel : Float = wheelPointVel.dotProduct(groundFwd);
        if ((fwdVel > slipVel) || (fwdVel < -slipVel)) 
        {
            friction *= slipFactor;
        }
        else if ((fwdVel > noslipVel) || (fwdVel < -noslipVel)) 
        {
            friction *= (1 - (1 - slipFactor) * (Math.abs(fwdVel) - noslipVel) / (slipVel - noslipVel));
        }
        if (fwdVel < 0) 
        {
            friction *= -1;
        }
        if (Math.abs(fwdVel) < smallVel) 
        {
            friction *= (Math.abs(fwdVel) / smallVel);
        }
        var fwdForce : Float = -friction * totalForceMag;
        extraForce = JNumber3D.getScaleVector(groundFwd, fwdForce);
        force = force.add(extraForce);
        
        wheelCentreVel = carBody.currentState.linVelocity.add(carBody.currentState.rotVelocity.crossProduct(tempv));
        _angVelForGrip = wheelCentreVel.dotProduct(groundFwd) / _radius;
        _torque += (-fwdForce * _radius);
        
        carBody.addWorldForce(force, groundPos, false);
        if (otherBody.movable) 
        {
            var maxOtherBodyAcc : Float = 500;
            var maxOtherBodyForce : Float = maxOtherBodyAcc * otherBody.mass;
            if (force.lengthSquared > maxOtherBodyForce * maxOtherBodyForce) 
            {
                force = JNumber3D.getScaleVector(force, maxOtherBodyForce / force.length);
            }
            otherBody.addWorldForce(JNumber3D.getScaleVector(force, -1), groundPos, false);
        }
        return true;
    }
    
    // Updates the rotational state etc
    public function update(dt : Float) : Void
    {
        if (dt <= 0) 
        {
            return;
        }
        var origAngVel : Float = _angVel;
        _upSpeed = (_displacement - _lastDisplacement) / Math.max(dt, JMath3D.NUM_TINY);
        
        if (_locked) 
        {
            _angVel = 0;
            _torque = 0;
        }
        else 
        {
            _angVel += (_torque * dt / _inertia);
            _torque = 0;
            
            if (((origAngVel > _angVelForGrip) && (_angVel < _angVelForGrip)) || ((origAngVel < _angVelForGrip) && (_angVel > _angVelForGrip))) 
            {
                _angVel = _angVelForGrip;
            }
            
            _angVel += _driveTorque * dt / _inertia;
            _driveTorque = 0;
            
            if (_angVel < -100) 
            {
                _angVel = -100;
            }
            else if (_angVel > 100) 
            {
                _angVel = 100;
            }
            _angVel *= _rotDamping;
            _axisAngle += (_angVel * dt * 180 / Math.PI);
        }
    }
    
    public function reset() : Void
    {
        _angVel = 0;
        _steerAngle = 0;
        _torque = 0;
        _driveTorque = 0;
        _axisAngle = 0;
        _displacement = 0;
        _upSpeed = 0;
        _locked = false;
        _lastDisplacement = 0;
        _lastOnFloor = false;
        _angVelForGrip = 0;
        _rotDamping = 0.99;
    }
}
