/*
Copyright (c) 2007 Danny Chapman
http://www.rowlhouse.co.uk

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software
in a product, an acknowledgment in the product documentation would be
appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source
distribution.
*/

/**
 * @authors Ringo (http://www.ringo.nl/)
 * 			Muzer(http://www.muzerly.com/)
 * 			Devin Reimer (blog.almostlogical.com)
 * 			Bartek (http://www.everyday3d.com/)
 * 			Katopz (http://www.sleepydesign.com/)
 
 * 			 
 * @link 	http://www.jiglibflash.com
 * @contact	jiglibflash at ringo.nl
 */

package jiglib.physics;

import jiglib.physics.RigidBody;

import jiglib.cof.JConfig;
import jiglib.collision.CollDetectInfo;
import jiglib.collision.CollPointInfo;
import jiglib.collision.CollisionInfo;
import jiglib.collision.CollisionSystemAbstract;
import jiglib.collision.CollisionSystemBrute;
import jiglib.collision.CollisionSystemGrid;
import jiglib.data.ContactData;
import jiglib.math.JNumber3D;
import jiglib.math.JMath3D;
import jiglib.math.Vector3D;
import jiglib.physics.constraint.JConstraint;

class PhysicsSystem
{
    public var gravity(get, never) : Vector3D;
    public var gravityAxis(get, never) : Int;
    public var bodies(get, never) : Array<RigidBody>;
    public var activeBodies(get, never) : Array<RigidBody>;
    public var constraints(get, never) : Array<JConstraint>;

    private static var _currentPhysicsSystem : PhysicsSystem;
    
    private inline static var _maxVelMag : Float = 0.5;
    private inline static var _minVelForProcessing : Float = 0.001;
    
    private var _bodies : Array<RigidBody>;
    private var _activeBodies : Array<RigidBody>;
    private var _collisions : Array<CollisionInfo>;
    private var _constraints : Array<JConstraint>;
    private var _controllers : Array<PhysicsController>;
    
    private var _gravityAxis : Int;
    private var _gravity : Vector3D;
    
    private var _doingIntegration : Bool;
    
    private var preProcessCollisionFn : CollisionInfo->Float->Void; // FUNCTION
    private var preProcessContactFn : CollisionInfo->Float->Void;
    private var processCollisionFn : CollisionInfo->Float->Bool;
    private var processContactFn : CollisionInfo->Float->Bool;
    
    private var _cachedContacts : Array<ContactData>;
    private var _collisionSystem : CollisionSystemAbstract;
    
    public static function getInstance() : PhysicsSystem
    {
        if (_currentPhysicsSystem == null) 
        {
            trace("version: JigLibHx (2015-07-07)");
            _currentPhysicsSystem = new PhysicsSystem();
        }
        return _currentPhysicsSystem;
    }
    
    public function new()
    {
        setSolverType(JConfig.solverType);
        _doingIntegration = false;
        _bodies = new Array<RigidBody>();
        _collisions = new Array<CollisionInfo>();
        _activeBodies = new Array<RigidBody>();
        _constraints = new Array<JConstraint>();
        _controllers = new Array<PhysicsController>();
        
        _cachedContacts = new Array<ContactData>();
        
        setGravity(JNumber3D.getScaleVector(Vector3D.Y_AXIS, -10));
    }
    
    public function setCollisionSystem(collisionSystemGrid : Bool = false, sx : Float = 0, sy : Float = 0, sz : Float = 0, nx : Int = 20, ny : Int = 20, nz : Int = 20, dx : Int = 200, dy : Int = 200, dz : Int = 200) : Void{
        // which collisionsystem to use grid / brute
        if (collisionSystemGrid) 
        {
            _collisionSystem = new CollisionSystemGrid(sx, sy, sz, nx, ny, nz, dx, dy, dz);
        }
        else {
            _collisionSystem = new CollisionSystemBrute();
        }
    }
    
    public function getCollisionSystem() : CollisionSystemAbstract
    {
        return _collisionSystem;
    }
    
    public function setGravity(gravity : Vector3D) : Void
    {
        _gravity = gravity;
        if (_gravity.x == _gravity.y && _gravity.y == _gravity.z) 
            _gravityAxis = -1;
        
        _gravityAxis = 0;
        if (Math.abs(_gravity.y) > Math.abs(_gravity.z)) 
            _gravityAxis = 1;
        
        if (Math.abs(_gravity.z) > Math.abs(JNumber3D.toArray(_gravity)[_gravityAxis])) 
            _gravityAxis = 2;
        
        // do update only when dirty, faster than call every time in step
        for (body in _bodies)
			body.updateGravity(_gravity, _gravityAxis);
    }
    
    // global gravity acceleration
    private function get_gravity() : Vector3D
    {
        return _gravity;
    }
    
    private function get_gravityAxis() : Int
    {
        return _gravityAxis;
    }
    
    //get all rigid bodies in the simulation
    private function get_bodies() : Array<RigidBody>
    {
        return _bodies;
    }
    
    //get all activated rigid bodies in the simulation
    private function get_activeBodies() : Array<RigidBody>
    {
        return _activeBodies;
    }
    
    //get all constraints in the simulation
    private function get_constraints() : Array<JConstraint>
    {
        return _constraints;
    }
    
    // Add a rigid body to the simulation
    public function addBody(body : RigidBody) : Void
    {
        if (_bodies.indexOf(body) < 0) 
        {
            _bodies.push(body);
            _collisionSystem.addCollisionBody(body);
            
            // update only once, and callback later when dirty
            body.updateGravity(_gravity, _gravityAxis);
        }
    }
    
    //remove a rigid body from the simulation
    public function removeBody(body : RigidBody) : Void
    {
        if (_bodies.indexOf(body) >= 0) 
        {
            _bodies.splice(_bodies.indexOf(body), 1);
            _collisionSystem.removeCollisionBody(body);
        }
    }
    
    //remove all rigid bodies from the simulation
    public function removeAllBodies() : Void
    {
        //_bodies.length = 0;
        _bodies.splice(0, _bodies.length);
        _collisionSystem.removeAllCollisionBodies();
    }
    
    // add a constraint to the simulation, do not need call this function yourself,
    // this just used in constraint system
    public function addConstraint(constraint : JConstraint) : Void
    {
        if (_constraints.indexOf(constraint) < 0) 
            _constraints.push(constraint);
    }
    
    //remove a constraint from the simulation, do not need call this function yourself,
    //this just used in constraint system
    public function removeConstraint(constraint : JConstraint) : Void
    {
        if (_constraints.indexOf(constraint) >= 0) 
            _constraints.splice(_constraints.indexOf(constraint), 1);
    }
    
    //remove all constraints from the simulation
    public function removeAllConstraints() : Void{
        for (constraint in _constraints){
            constraint.disableConstraint();
        }
        //_constraints.length = 0;
        _constraints.splice(0, _constraints.length);
    }
    
    // Add a physics controlled to the simulation,do not need call this function yourself,
    //this just used in PhysicsController
    public function addController(controller : PhysicsController) : Void
    {
        if (_controllers.indexOf(controller) < 0) 
            _controllers.push(controller);
    }
    
    // Remove a physics controlled from the simulation,do not need call this function yourself,
    //this just used in PhysicsController
    public function removeController(controller : PhysicsController) : Void
    {
        if (_controllers.indexOf(controller) >= 0) 
            _controllers.splice(_controllers.indexOf(controller), 1);
    }
    
    //remove all controllers from the simulation
    public function removeAllControllers() : Void
    {
        for (controller in _controllers){
            controller.disableController();
        }
        //_controllers.length = 0;
        _controllers.splice(0, _controllers.length);
    }
    
    public function setSolverType(type : String) : Void
    {
        switch (type)
        {
            case "FAST":
                preProcessCollisionFn = preProcessCollisionFast;
                preProcessContactFn = preProcessCollisionFast;
                processCollisionFn = processCollisionNormal;
                processContactFn = processCollisionNormal;
                return;
            case "NORMAL":
                preProcessCollisionFn = preProcessCollisionNormal;
                preProcessContactFn = preProcessCollisionNormal;
                processCollisionFn = processCollisionNormal;
                processContactFn = processCollisionNormal;
                return;
            case "ACCUMULATED":
                preProcessCollisionFn = preProcessCollisionNormal;
                preProcessContactFn = preProcessCollisionAccumulated;
                processCollisionFn = processCollisionNormal;
                processContactFn = processCollisionAccumulated;
                return;
            default:
                preProcessCollisionFn = preProcessCollisionNormal;
                preProcessContactFn = preProcessCollisionNormal;
                processCollisionFn = processCollisionNormal;
                processContactFn = processCollisionNormal;
                return;
        }
    }
    
    private function moreCollPtPenetration(info0 : CollPointInfo, info1 : CollPointInfo) : Int
    {
        if (info0.initialPenetration < info1.initialPenetration) 
            return 1
        else if (info0.initialPenetration > info1.initialPenetration) 
            return -1
        else 
        return 0;
    }
    
    // fast-but-inaccurate pre-processor
    private function preProcessCollisionFast(collision : CollisionInfo, dt : Float) : Void
    {
        collision.satisfied = false;
        
        var body0 : RigidBody;
        var body1 : RigidBody;
        
        body0 = collision.objInfo.body0;
        body1 = collision.objInfo.body1;
        
        var N : Vector3D = collision.dirToBody;
        var tempV : Vector3D;
        var timescale : Float = JConfig.numPenetrationRelaxationTimesteps * dt;
        var approachScale : Float = 0;
        var tiny : Float = JMath3D.NUM_TINY;
        var allowedPenetration : Float = JConfig.allowedPenetration;
        var ptInfo : CollPointInfo;
        var collision_pointInfo : Array<CollPointInfo> = collision.pointInfo;
        
        if (collision_pointInfo.length > 3) 
        {
            /*collision_pointInfo =*/ collision_pointInfo.sort(moreCollPtPenetration);

            while (collision_pointInfo.length > 3) collision_pointInfo.pop();
            //collision_pointInfo.fixed = false;
            //collision_pointInfo.length = 3;
            //collision_pointInfo.fixed = true;
        }
        
        for (ptInfo in collision_pointInfo)
        {
            if (!body0.movable) 
            {
                ptInfo.denominator = 0;
            }
            else 
            {
                tempV = ptInfo.r0.crossProduct(N);
                tempV = body0.worldInvInertia.transformVector(tempV);
                ptInfo.denominator = body0.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r0));
            }
            
            if (body1 != null && body1.movable) 
            {
                tempV = ptInfo.r1.crossProduct(N);
                tempV = body1.worldInvInertia.transformVector(tempV);
                ptInfo.denominator += (body1.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
            }
            
            if (ptInfo.denominator < tiny) 
                ptInfo.denominator = tiny;
            
            if (ptInfo.initialPenetration > allowedPenetration) 
            {
                ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
            }
            else 
            {
                approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
                
                if (approachScale < tiny) 
                {
                    approachScale = tiny;
                }
                else if (approachScale > 1) 
                {
                    approachScale = 1;
                }
                
                ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
            }
            
            if (ptInfo.minSeparationVel > _maxVelMag) 
                ptInfo.minSeparationVel = _maxVelMag;
        }
    }
    
    // Special pre-processor for the normal solver
    private function preProcessCollisionNormal(collision : CollisionInfo, dt : Float) : Void
    {
        collision.satisfied = false;
        
        var body0 : RigidBody;
        var body1 : RigidBody;
        
        body0 = collision.objInfo.body0;
        body1 = collision.objInfo.body1;
        
        var N : Vector3D = collision.dirToBody;
        var tempV : Vector3D;
        var timescale : Float = JConfig.numPenetrationRelaxationTimesteps * dt;
        var approachScale : Float = 0;
        var tiny : Float = JMath3D.NUM_TINY;
        var allowedPenetration : Float = JConfig.allowedPenetration;
        var ptInfo : CollPointInfo;
        var collision_pointInfo : Array<CollPointInfo> = collision.pointInfo;
        
        for (ptInfo in collision_pointInfo)
        {
            if (!body0.movable) 
            {
                ptInfo.denominator = 0;
            }
            else 
            {
                tempV = ptInfo.r0.crossProduct(N);
                tempV = body0.worldInvInertia.transformVector(tempV);
                ptInfo.denominator = body0.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r0));
            }
            
            if (body1 != null && body1.movable) 
            {
                tempV = ptInfo.r1.crossProduct(N);
                tempV = body1.worldInvInertia.transformVector(tempV);
                ptInfo.denominator += (body1.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
            }
            
            if (ptInfo.denominator < tiny) 
                ptInfo.denominator = tiny;
            
            if (ptInfo.initialPenetration > allowedPenetration) 
            {
                ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
            }
            else 
            {
                approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
                
                if (approachScale < tiny) 
                {
                    approachScale = tiny;
                }
                else if (approachScale > 1) 
                {
                    approachScale = 1;
                }
                ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
            }
            
            if (ptInfo.minSeparationVel > _maxVelMag) 
                ptInfo.minSeparationVel = _maxVelMag;
        }
    }
    
    // Special pre-processor for the accumulated solver
    private function preProcessCollisionAccumulated(collision : CollisionInfo, dt : Float) : Void
    {
        collision.satisfied = false;
        
        var body0 : RigidBody;
        var body1 : RigidBody;
        
        body0 = collision.objInfo.body0;
        body1 = collision.objInfo.body1;
        
        var N : Vector3D = collision.dirToBody;
        var tempV : Vector3D;
        var timescale : Float = JConfig.numPenetrationRelaxationTimesteps * dt;
        var approachScale : Float = 0;
        var numTiny : Float = JMath3D.NUM_TINY;
        var allowedPenetration : Float = JConfig.allowedPenetration;
        var ptInfo : CollPointInfo;
        var collision_pointInfo : Array<CollPointInfo> = collision.pointInfo;
        
        for (ptInfo in collision_pointInfo)
        {
            if (!body0.movable) 
            {
                ptInfo.denominator = 0;
            }
            else 
            {
                tempV = ptInfo.r0.crossProduct(N);
                tempV = body0.worldInvInertia.transformVector(tempV);
                ptInfo.denominator = body0.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r0));
            }
            
            if (body1 != null && body1.movable) 
            {
                tempV = ptInfo.r1.crossProduct(N);
                tempV = body1.worldInvInertia.transformVector(tempV);
                ptInfo.denominator += (body1.invMass + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
            }
            
            if (ptInfo.denominator < numTiny) 
            {
                ptInfo.denominator = numTiny;
            }
            
            if (ptInfo.initialPenetration > allowedPenetration) 
            {
                ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
            }
            else 
            {
                approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
                
                if (approachScale < numTiny) 
                {
                    approachScale = numTiny;
                }
                else if (approachScale > 1) 
                {
                    approachScale = 1;
                }
                
                ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / Math.max(dt, numTiny);
            }
            
            ptInfo.accumulatedNormalImpulse = 0;
            ptInfo.accumulatedNormalImpulseAux = 0;
            ptInfo.accumulatedFrictionImpulse = new Vector3D();
            
            var bestDistSq : Float = 0.04;
            var bp : BodyPair = new BodyPair(body0, body1, new Vector3D(), new Vector3D());
            
            for (cachedContact in _cachedContacts)
            {
                if (!(bp.body0 == cachedContact.pair.body0 && bp.body1 == cachedContact.pair.body1)) 
                    continue;
                
                var distSq : Float = ((cachedContact.pair.body0 == body0)) ? cachedContact.pair.r.subtract(ptInfo.r0).lengthSquared : cachedContact.pair.r.subtract(ptInfo.r1).lengthSquared;
                
                if (distSq < bestDistSq) 
                {
                    bestDistSq = distSq;
                    ptInfo.accumulatedNormalImpulse = cachedContact.impulse.normalImpulse;
                    ptInfo.accumulatedNormalImpulseAux = cachedContact.impulse.normalImpulseAux;
                    ptInfo.accumulatedFrictionImpulse = cachedContact.impulse.frictionImpulse;
                    
                    if (cachedContact.pair.body0 != body0) 
                        ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);
                }
            }
            

            if (ptInfo.accumulatedNormalImpulse != 0) 
            {
                var impulse : Vector3D = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulse);
                impulse = impulse.add(ptInfo.accumulatedFrictionImpulse);
                body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
                if (body1 != null) 
                    body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
            }
            
            if (ptInfo.accumulatedNormalImpulseAux != 0) 
            {
                var impulse : Vector3D = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux);
                body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
                if (body1 != null) 
                    body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
            }
        }
    }
    
    /* Handle an individual collision by classifying it, calculating
		impulse, applying impulse and updating the velocities of the
		objects. Allows over-riding of the elasticity. Ret val indicates
		if an impulse was applied
		*/
    private function processCollisionNormal(collision : CollisionInfo, dt : Float) : Bool
    {
        collision.satisfied = true;
        
        var body0 : RigidBody;
        var body1 : RigidBody;
        
        body0 = collision.objInfo.body0;
        body1 = collision.objInfo.body1;
        
        var gotOne : Bool = false;
        var deltaVel : Float = 0;
        var normalVel : Float = 0;
        var finalNormalVel : Float = 0;
        var normalImpulse : Float = 0;
        var tangent_speed : Float;
        var denominator : Float;
        var impulseToReverse : Float;
        var impulseFromNormalImpulse : Float;
        var frictionImpulse : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        var N : Vector3D = collision.dirToBody;
        var impulse : Vector3D;
        var Vr0 : Vector3D;
        var Vr1 : Vector3D = null;
        var tempV : Vector3D;
        var VR : Vector3D;
        var tangent_vel : Vector3D;
        var T : Vector3D;
        var ptInfo : CollPointInfo;
        
        var collision_pointInfo : Array<CollPointInfo> = collision.pointInfo;
        
        for (ptInfo in collision_pointInfo)
        {
            Vr0 = body0.getVelocity(ptInfo.r0);
            if (body1 != null) {
                Vr1 = body1.getVelocity(ptInfo.r1);
                normalVel = Vr0.subtract(Vr1).dotProduct(N);
            }
            else {
                normalVel = Vr0.dotProduct(N);
            }
            if (normalVel > ptInfo.minSeparationVel) 
                continue;
            
            finalNormalVel = -1 * collision.mat.restitution * normalVel;
            
            if (finalNormalVel < _minVelForProcessing) 
                finalNormalVel = ptInfo.minSeparationVel;
            
            deltaVel = finalNormalVel - normalVel;
            
            if (deltaVel <= _minVelForProcessing) 
                continue;
            
            normalImpulse = deltaVel / ptInfo.denominator;
            
            gotOne = true;
            impulse = JNumber3D.getScaleVector(N, normalImpulse);
            
            body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
            if (body1 != null) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
            
            VR = Vr0.clone();
            if (body1 != null) VR = VR.subtract(Vr1);
            tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
            tangent_speed = tangent_vel.length;
            
            if (tangent_speed > _minVelForProcessing) 
            {
                T = JNumber3D.getDivideVector(tangent_vel, -tangent_speed);
                denominator = 0;
                
                if (body0.movable) 
                {
                    tempV = ptInfo.r0.crossProduct(T);
                    tempV = body0.worldInvInertia.transformVector(tempV);
                    denominator = body0.invMass + T.dotProduct(tempV.crossProduct(ptInfo.r0));
                }
                
                if (body1 != null && body1.movable) 
                {
                    tempV = ptInfo.r1.crossProduct(T);
                    tempV = body1.worldInvInertia.transformVector(tempV);
                    denominator += (body1.invMass + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
                }
                
                if (denominator > tiny) 
                {
                    impulseToReverse = tangent_speed / denominator;
                    
                    impulseFromNormalImpulse = collision.mat.friction * normalImpulse;
                    if (impulseToReverse < impulseFromNormalImpulse) {
                        frictionImpulse = impulseToReverse;
                    }
                    else {
                        frictionImpulse = collision.mat.friction * normalImpulse;
                    }
                    T.scaleBy(frictionImpulse);
                    body0.applyBodyWorldImpulse(T, ptInfo.r0, false);
                    if (body1 != null) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(T, -1), ptInfo.r1, false);
                }
            }
        }
        
        if (gotOne) 
        {
            body0.setConstraintsAndCollisionsUnsatisfied();
            if (body1 != null) body1.setConstraintsAndCollisionsUnsatisfied();
        }
        
        return gotOne;
    }
    
    // Accumulated and clamp impulses
    private function processCollisionAccumulated(collision : CollisionInfo, dt : Float) : Bool
    {
        collision.satisfied = true;
        
        var body0 : RigidBody;
        var body1 : RigidBody;
        body0 = collision.objInfo.body0;
        body1 = collision.objInfo.body1;
        
        var gotOne : Bool = false;
        var deltaVel : Float = 0;
        var normalVel : Float = 0;
        var finalNormalVel : Float = 0;
        var normalImpulse : Float = 0;
        var tangent_speed : Float;
        var denominator : Float;
        var impulseToReverse : Float;
        var AFIMag : Float;
        var maxAllowedAFIMag : Float;
        var tiny : Float = JMath3D.NUM_TINY;
        var N : Vector3D = collision.dirToBody;
        var impulse : Vector3D;
        var Vr0 : Vector3D;
        var Vr1 : Vector3D;
        var tempV : Vector3D;
        var VR : Vector3D;
        var tangent_vel : Vector3D;
        var T : Vector3D;
        var frictionImpulseVec : Vector3D;
        var origAccumulatedFrictionImpulse : Vector3D;
        var actualFrictionImpulse : Vector3D;
        var ptInfo : CollPointInfo;
        
        var collision_pointInfo : Array<CollPointInfo> = collision.pointInfo;
        
        for (ptInfo in collision_pointInfo)
        {
            Vr0 = body0.getVelocity(ptInfo.r0);
            if (body1 != null) {
                Vr1 = body1.getVelocity(ptInfo.r1);
                normalVel = Vr0.subtract(Vr1).dotProduct(N);
            }
            else {
                normalVel = Vr0.dotProduct(N);
            }
            deltaVel = -normalVel;
            
            if (ptInfo.minSeparationVel < 0) 
                deltaVel += ptInfo.minSeparationVel;
            
            if (Math.abs(deltaVel) > _minVelForProcessing) 
            {
                normalImpulse = deltaVel / ptInfo.denominator;
                var origAccumulatedNormalImpulse : Float = ptInfo.accumulatedNormalImpulse;
                ptInfo.accumulatedNormalImpulse = Math.max(ptInfo.accumulatedNormalImpulse + normalImpulse, 0);
                var actualImpulse : Float = ptInfo.accumulatedNormalImpulse - origAccumulatedNormalImpulse;
                
                impulse = JNumber3D.getScaleVector(N, actualImpulse);
                body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
                if (body1 != null) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
                
                gotOne = true;
            }
            
            Vr0 = body0.getVelocityAux(ptInfo.r0);
            if (body1 != null) {
                Vr1 = body1.getVelocityAux(ptInfo.r1);
                normalVel = Vr0.subtract(Vr1).dotProduct(N);
            }
            else {
                normalVel = Vr0.dotProduct(N);
            }
            
            deltaVel = -normalVel;
            
            if (ptInfo.minSeparationVel > 0) 
                deltaVel += ptInfo.minSeparationVel;
            
            if (Math.abs(deltaVel) > _minVelForProcessing) 
            {
                normalImpulse = deltaVel / ptInfo.denominator;
                var origAccumulatedNormalImpulse:Float = ptInfo.accumulatedNormalImpulseAux;
                ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + normalImpulse, 0);
                var actualImpulse:Float = ptInfo.accumulatedNormalImpulseAux - origAccumulatedNormalImpulse;
                
                impulse = JNumber3D.getScaleVector(N, actualImpulse);
                body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
                if (body1 != null) body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
                
                gotOne = true;
            }
            
            if (ptInfo.accumulatedNormalImpulse > 0) 
            {
                Vr0 = body0.getVelocity(ptInfo.r0);
                VR = Vr0.clone();
                if (body1 != null) {
                    Vr1 = body1.getVelocity(ptInfo.r1);
                    VR = VR.subtract(Vr1);
                }
                tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
                tangent_speed = tangent_vel.length;
                
                if (tangent_speed > _minVelForProcessing) 
                {
                    
                    T = JNumber3D.getScaleVector(JNumber3D.getDivideVector(tangent_vel, tangent_speed), -1);
                    denominator = 0;
                    if (body0.movable) 
                    {
                        tempV = ptInfo.r0.crossProduct(T);
                        tempV = body0.worldInvInertia.transformVector(tempV);
                        denominator = body0.invMass + T.dotProduct(tempV.crossProduct(ptInfo.r0));
                    }
                    
                    if (body1 != null && body1.movable) 
                    {
                        tempV = ptInfo.r1.crossProduct(T);
                        tempV = body1.worldInvInertia.transformVector(tempV);
                        denominator += (body1.invMass + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
                    }
                    
                    if (denominator > tiny) 
                    {
                        impulseToReverse = tangent_speed / denominator;
                        frictionImpulseVec = JNumber3D.getScaleVector(T, impulseToReverse);
                        
                        origAccumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.clone();
                        ptInfo.accumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.add(frictionImpulseVec);
                        
                        AFIMag = ptInfo.accumulatedFrictionImpulse.length;
                        maxAllowedAFIMag = collision.mat.friction * ptInfo.accumulatedNormalImpulse;
                        
                        if (AFIMag > tiny && AFIMag > maxAllowedAFIMag) 
                            ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, maxAllowedAFIMag / AFIMag);
                        
                        actualFrictionImpulse = ptInfo.accumulatedFrictionImpulse.subtract(origAccumulatedFrictionImpulse);
                        
                        body0.applyBodyWorldImpulse(actualFrictionImpulse, ptInfo.r0, false);
                        if (body1 != null) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualFrictionImpulse, -1), ptInfo.r1, false);
                    }
                }
            }
        }
        
        if (gotOne) 
        {
            body0.setConstraintsAndCollisionsUnsatisfied();
            if (body1 != null) body1.setConstraintsAndCollisionsUnsatisfied();
        }
        
        return gotOne;
    }
    
    private function processCollisionForShock(collision : CollisionInfo, dt : Float) : Bool{
        
        collision.satisfied = true;
        var N : Vector3D = collision.dirToBody;
        
        var timescale : Float = JConfig.numPenetrationRelaxationTimesteps * dt;
        var body0 : RigidBody = collision.objInfo.body0;
        var body1 : RigidBody = collision.objInfo.body1;
        
        if (!body0.movable) 
            body0 = null;
        if (body1 != null && !body1.movable) 
            body1 = null;
        
        if (body0 == null && body1 == null) {
            return false;
        }
        
        var normalVel : Float = 0;
        var finalNormalVel : Float;
        var impulse : Float;
        var orig : Float;
        var actualImpulse : Vector3D;
        
        for (ptInfo in collision.pointInfo){
            normalVel = 0;
            if (body0 != null) {
                normalVel = body0.getVelocity(ptInfo.r0).dotProduct(N) + body0.getVelocityAux(ptInfo.r0).dotProduct(N);
            }
            if (body1 != null) {
                normalVel -= (body1.getVelocity(ptInfo.r1).dotProduct(N) + body1.getVelocityAux(ptInfo.r1).dotProduct(N));
            }
            
            finalNormalVel = (ptInfo.initialPenetration - JConfig.allowedPenetration) / timescale;
            if (finalNormalVel < 0) {
                continue;
            }
            impulse = (finalNormalVel - normalVel) / ptInfo.denominator;
            orig = ptInfo.accumulatedNormalImpulseAux;
            ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + impulse, 0);
            actualImpulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux - orig);
            
            if (body0 != null) body0.applyBodyWorldImpulse(actualImpulse, ptInfo.r0, false);
            if (body1 != null) body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualImpulse, -1), ptInfo.r1, false);
        }
        
        if (body0 != null) body0.setConstraintsAndCollisionsUnsatisfied();
        if (body1 != null) body1.setConstraintsAndCollisionsUnsatisfied();
        return true;
    }
    
    private function sortPositionX(body0 : RigidBody, body1 : RigidBody) : Int
    {
        if (body0.currentState.position.x < body1.currentState.position.x) 
            return -1;
        else if (body0.currentState.position.x > body1.currentState.position.x) 
            return 1;
        return 0;
    }
    
    private function sortPositionY(body0 : RigidBody, body1 : RigidBody) : Int
    {
        if (body0.currentState.position.y < body1.currentState.position.y) 
            return -1;
        else if (body0.currentState.position.y > body1.currentState.position.y) 
            return 1;
        return 0;
    }
    
    private function sortPositionZ(body0 : RigidBody, body1 : RigidBody) : Int
    {
        if (body0.currentState.position.z < body1.currentState.position.z) 
            return -1;
        else if (body0.currentState.position.z > body1.currentState.position.z) 
            return 1;
        return 0;
    }
    
    private function doShockStep(dt : Float) : Void
    {
        if (Math.abs(_gravity.x) > Math.abs(_gravity.y) && Math.abs(_gravity.x) > Math.abs(_gravity.z)) 
        {
            /*_bodies =*/ _bodies.sort(sortPositionX);
            /*_collisionSystem.collBody =*/ _collisionSystem.collBody.sort(sortPositionX);
        }
        else if (Math.abs(_gravity.y) > Math.abs(_gravity.z) && Math.abs(_gravity.y) > Math.abs(_gravity.x)) 
        {
            /*_bodies =*/ _bodies.sort(sortPositionY);
            /*_collisionSystem.collBody =*/ _collisionSystem.collBody.sort(sortPositionY);
        }
        else if (Math.abs(_gravity.z) > Math.abs(_gravity.x) && Math.abs(_gravity.z) > Math.abs(_gravity.y)) 
        {
            /*_bodies =*/ _bodies.sort(sortPositionZ);
            /*_collisionSystem.collBody =*/ _collisionSystem.collBody.sort(sortPositionZ);
        }
        
        var setImmovable : Bool;
        var gotOne : Bool = true;
        var info : CollisionInfo;
        var body0 : RigidBody;
        var body1 : RigidBody;
        
        for (body in _bodies)
        {
            if (body.movable) 
            {
                if (body.collisions.length == 0 || !body.isActive) 
                {
                    body.internalSetImmovable();
                }
                else 
                {
                    setImmovable = false;
                    for (info in body.collisions)
                    {
                        body0 = info.objInfo.body0;
                        body1 = info.objInfo.body1;
                        
                        if ((body0 == body && (body1 == null || !body1.movable)) || (body1 == body && (body0 == null || !body0.movable))) 
                        {
                            preProcessCollisionFn(info, dt);
                            processCollisionForShock(info, dt);
                            setImmovable = true;
                        }
                    }
                    
                    if (setImmovable) 
                    {
                        body.internalSetImmovable();
                    }
                }
            }
        }
        
        for (body in _bodies)
        {
            body.internalRestoreImmovable();
        }
    }
    
    private function updateContactCache() : Void
    {
        _cachedContacts = new Array<ContactData>();
        
        var fricImpulse : Vector3D;
        var body0 : RigidBody;
        var body1 : RigidBody;
        var contact : ContactData;
        var collInfo_objInfo : CollDetectInfo;
        var collInfo_pointInfo : Array<CollPointInfo>;
        var id1 : Int;
        for (collInfo in _collisions)
        {
            collInfo_objInfo = collInfo.objInfo;
            body0 = collInfo_objInfo.body0;
            body1 = collInfo_objInfo.body1;
            
            collInfo_pointInfo = collInfo.pointInfo;
            //_cachedContacts.fixed = false;
            //_cachedContacts.length += collInfo_pointInfo.length;
            //_cachedContacts.fixed = true;
            
            for (ptInfo in collInfo_pointInfo) {

                id1 = -1;
                if (body1 != null) id1 = body1.id;

                if (body0.id > id1) {
                    fricImpulse = ptInfo.accumulatedFrictionImpulse;
                }
                else {
                    fricImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);
                }

                contact = new ContactData();
                _cachedContacts.push(contact);
                contact.pair = new BodyPair(body0, body1, ptInfo.r0, ptInfo.r1);
                contact.impulse = new CachedImpulse(ptInfo.accumulatedNormalImpulse, ptInfo.accumulatedNormalImpulseAux, ptInfo.accumulatedFrictionImpulse);
            }
        }
    }
    
    private function handleAllConstraints(dt : Float, iter : Int, forceInelastic : Bool) : Void
    {
        var origNumCollisions : Int = _collisions.length;
        var iteration : Int = JConfig.numConstraintIterations;
        var step : Int;
        var i : Int;
        var len : Int;
        var collInfo : CollisionInfo;
        var constraint : JConstraint;
        var flag : Bool;
        var gotOne : Bool;
        
        if (_constraints.length > 0) 
        {
            for (constraint in _constraints)
                constraint.preApply(dt);
            
            for (step in 0...iteration){
                gotOne = false;
                for (constraint in _constraints)
                {
                    if (!constraint.satisfied) 
                    {
                        flag = constraint.apply(dt);
                        gotOne = gotOne || flag;
                    }
                }
                if (!gotOne) 
                    break;
            }
        }
        
        if (forceInelastic) 
        {
            for (collInfo in _collisions)
            {
                preProcessContactFn(collInfo, dt);
                collInfo.mat.restitution = 0;
                collInfo.satisfied = false;
            }
        }
        else 
        {
            for (collInfo in _collisions)
            preProcessCollisionFn(collInfo, dt);
        }
        
        for (step in 0...iter){
            gotOne = true;
            
            for (collInfo in _collisions)
            {
                if (!collInfo.satisfied) 
                {
                    if (forceInelastic) 
                        flag = processContactFn(collInfo, dt)
                    else 
                    flag = processCollisionFn(collInfo, dt);
                    
                    gotOne = gotOne || flag;
                }
            }
            
            len = _collisions.length;
            if (forceInelastic) 
            {
                for (i in origNumCollisions...len){
                    collInfo = _collisions[i];
                    collInfo.mat.restitution = 0;
                    collInfo.satisfied = false;
                    preProcessContactFn(collInfo, dt);
                }
            }
            else 
            {
                for (i in origNumCollisions...len) {
                    preProcessCollisionFn(_collisions[i], dt);
                }
            }
            
            origNumCollisions = len;
            
            if (!gotOne) 
                break;
        }
    }
    
    public function activateObject(body : RigidBody) : Void
    {
        if (!body.movable || body.isActive) return;
        
        if (_activeBodies.indexOf(body) < 0) {
            body.setActive();
            //_activeBodies.fixed = false;
            _activeBodies.push(body);
            //_activeBodies.fixed = true;
        }
    }
    
    private function tryToActivateAllFrozenObjects() : Void
    {
        for (body in _bodies)
        {
            if (!body.isActive) 
            {
                if (body.getShouldBeActive()) 
                {
                    activateObject(body);
                }
                else 
                {
                    body.setLineVelocity(new Vector3D());
                    body.setAngleVelocity(new Vector3D());
                }
            }
        }
    }
    
    private function tryToFreezeAllObjects(dt : Float) : Void
    {
        for (activeBody in _activeBodies){
            activeBody.dampForDeactivation();
            activeBody.tryToFreeze(dt);
        }
    }
    
    private function activateAllFrozenObjectsLeftHanging() : Void
    {
        var other_body : RigidBody;
        var body_collisions : Array<CollisionInfo>;
        
        for (body in _activeBodies)
        {
            body.doMovementActivations(this);
            body_collisions = body.collisions;
            if (body_collisions.length > 0) 
            {
                for (collisionInfo in body_collisions)
                {
                    other_body = collisionInfo.objInfo.body0;
                    if (other_body == body) 
                        other_body = collisionInfo.objInfo.body1;
                    
                    if (!other_body.isActive) 
                        body.addMovementActivation(body.currentState.position, other_body);
                }
            }
        }
    }
    
    private function updateAllController(dt : Float) : Void
    {
        for (controller in _controllers)
            controller.updateController(dt);
    }
    
    private function updateAllVelocities(dt : Float) : Void
    {
        for (activeBody in _activeBodies)
            activeBody.updateVelocity(dt);
    }
    
    private function notifyAllPostPhysics(dt : Float) : Void
    {
        for (activeBody in _activeBodies)
            activeBody.postPhysics(dt);
    }
    
    private function detectAllCollisions(dt : Float) : Void
    {
        for (body in _bodies){
            if (body.isActive) {
                body.storeState();
                body.updateVelocity(dt);
                body.updatePositionWithAux(dt);
            }
            //body.collisions.length = 0;
            body.collisions.splice(0, body.collisions.length);
        }
        
        //_collisions.length = 0;
        _collisions.splice(0, _collisions.length);
        _collisionSystem.detectAllCollisions(_activeBodies, _collisions);
        
        for (activeBody in _activeBodies)
            activeBody.restoreState();
    }
    
    private function findAllActiveBodiesAndCopyStates() : Void
    {
        _activeBodies = new Array<RigidBody>();
        
        for (body in _bodies)
        {
            // findAllActiveBodies
            if (body.isActive) 
            {
                //_activeBodies[Std.int(i++)] = body;
                //i++;
                _activeBodies.push(body);
                body.copyCurrentStateToOld();
            }
        }
    }
    
    // Integrates the system forwards by dt - the caller is
    // responsible for making sure that repeated calls to this use
    // the same dt (if desired)
    public function integrate(dt : Float) : Void
    {
        _doingIntegration = true;
        
        findAllActiveBodiesAndCopyStates();
        updateAllController(dt);
        detectAllCollisions(dt);
        handleAllConstraints(dt, JConfig.numCollisionIterations, false);
        updateAllVelocities(dt);
        handleAllConstraints(dt, JConfig.numContactIterations, true);
        
        if (JConfig.doShockStep) doShockStep(dt);
        
        tryToActivateAllFrozenObjects();
        tryToFreezeAllObjects(dt);
        activateAllFrozenObjectsLeftHanging();
        
        notifyAllPostPhysics(dt);
        
        if (JConfig.solverType == "ACCUMULATED") 
            updateContactCache();
        
        _doingIntegration = false;
    }
}
