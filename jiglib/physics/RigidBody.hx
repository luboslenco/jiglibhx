package jiglib.physics;

#if JIGLIB_FLASH_EVENTS
import flash.events.EventDispatcher;
#end

import jiglib.cof.JConfig;
import jiglib.collision.CollisionInfo;
import jiglib.collision.CollisionSystemAbstract;
import jiglib.collision.CollisionSystemGridEntry;
import jiglib.data.CollOutData;
import jiglib.geometry.JAABox;
import jiglib.geometry.JSegment;
import jiglib.math.*;
import jiglib.events.JCollisionEvent;
import jiglib.physics.constraint.JConstraint;
import jiglib.plugin.ISkin3D;

#if JIGLIB_FLASH_EVENTS
class RigidBody extends EventDispatcher
#else
class RigidBody
#end
{
    public var rotationX(get, set) : Float;
    public var rotationY(get, set) : Float;
    public var rotationZ(get, set) : Float;
    public var x(get, set) : Float;
    public var y(get, set) : Float;
    public var z(get, set) : Float;
    public var mass(get, set) : Float;
    public var movable(get, set) : Bool;
    public var currentState(get, never) : PhysicsState;
    public var oldState(get, never) : PhysicsState;
    public var id(get, never) : Int;
    public var type(get, never) : String;
    public var skin(get, never) : ISkin3D;
    public var boundingSphere(get, never) : Float;
    public var boundingBox(get, never) : JAABox;
    public var force(get, never) : Vector3D;
    public var invMass(get, never) : Float;
    public var worldInertia(get, never) : Matrix3D;
    public var worldInvInertia(get, never) : Matrix3D;
    public var nonCollidables(get, never) : Array<RigidBody>;
    public var constraints(get, never) : Array<JConstraint>;
    public var linVelocityDamping(get, set) : Vector3D;
    public var rotVelocityDamping(get, set) : Vector3D;
    public var maxLinVelocities(get, set) : Vector3D;
    public var maxRotVelocities(get, set) : Vector3D;
    public var material(get, never) : MaterialProperties;
    public var restitution(get, set) : Float;
    public var friction(get, set) : Float;
    public var onCollisionStart(get, set) : JCollisionEvent->Void;
    public var onCollisionEnd(get, set) : JCollisionEvent->Void;

    private static var idCounter : Int = 0;
    
    private var _id : Int;
    private var _skin : ISkin3D;
    
    private var _type : String;
    private var _boundingSphere : Float;
    private var _boundingBox : JAABox;
    
    private var _currState : PhysicsState;
    
    private var _oldState : PhysicsState;
    private var _storeState : PhysicsState;
    private var _invOrientation : Matrix3D;
    private var _currLinVelocityAux : Vector3D;
    private var _currRotVelocityAux : Vector3D;
    
    private var _mass : Float;
    private var _invMass : Float;
    private var _bodyInertia : Matrix3D;
    private var _bodyInvInertia : Matrix3D;
    private var _worldInertia : Matrix3D;
    private var _worldInvInertia : Matrix3D;
    
    private var _force : Vector3D;
    private var _torque : Vector3D;
    
    private var _linVelDamping : Vector3D;
    private var _rotVelDamping : Vector3D;
    private var _maxLinVelocities : Vector3D;
    private var _maxRotVelocities : Vector3D;
    
    private var _movable : Bool;
    private var _origMovable : Bool;
    private var _inactiveTime : Float;
    
    // The list of bodies that need to be activated when we move away from our stored position
    private var _bodiesToBeActivatedOnMovement : Array<RigidBody>;
    
    private var _storedPositionForActivation : Vector3D;  // The position stored when we need to notify other bodies  
    private var _lastPositionForDeactivation : Vector3D;  // last position for when trying the deactivate  
    private var _lastOrientationForDeactivation : Matrix3D;  // last orientation for when trying to deactivate  
    
    private var _material : MaterialProperties;
    
    private var _rotationX : Float = 0;
    private var _rotationY : Float = 0;
    private var _rotationZ : Float = 0;
    private var _useDegrees : Bool;
    
    private var _nonCollidables : Array<RigidBody>;
    private var _collideBodies : Array<RigidBody>;
    private var _constraints : Array<JConstraint>;
    
    // Bypass rapid call to PhysicsSystem, will update only when dirty.
    private var _gravity : Vector3D;
    private var _gravityAxis : Int;
    // Calculate only when gravity is dirty or mass is dirty.
    private var _gravityForce : Vector3D;

    // Additions in Haxe version for event dispatching fallback
    private var _onCollisionStart:JCollisionEvent->Void;
    private var _onCollisionEnd:JCollisionEvent->Void;
    
    public var collisions : Array<CollisionInfo>;  //store all collision info of this body  
    public var externalData : CollisionSystemGridEntry;  // used when collision system is grid  
    public var collisionSystem : CollisionSystemAbstract;
    
    public function new(skin : ISkin3D)
    {
        #if JIGLIB_FLASH_EVENTS
        super();
        #end

        _useDegrees = ((JConfig.rotationType == "DEGREES")) ? true : false;
        
        _id = idCounter++;
        
        _skin = skin;
        _material = new MaterialProperties();
        
        _bodyInertia = new Matrix3D();
        _bodyInvInertia = JMatrix3D.getInverseMatrix(_bodyInertia);
        
        _currState = new PhysicsState();
        _oldState = new PhysicsState();
        _storeState = new PhysicsState();
        _currLinVelocityAux = new Vector3D();
        _currRotVelocityAux = new Vector3D();
        
        _force = new Vector3D();
        _torque = new Vector3D();
        
        _invOrientation = JMatrix3D.getInverseMatrix(_currState.orientation);
        _linVelDamping = new Vector3D(0.999, 0.999, 0.999);
        _rotVelDamping = new Vector3D(0.999, 0.999, 0.999);
        _maxLinVelocities = new Vector3D(JMath3D.NUM_HUGE, JMath3D.NUM_HUGE, JMath3D.NUM_HUGE);
        _maxRotVelocities = new Vector3D(JMath3D.NUM_HUGE, JMath3D.NUM_HUGE, JMath3D.NUM_HUGE);

        _inactiveTime = 0;
        isActive = true;
        _movable = true;
        _origMovable = true;
        
        collisions = new Array<CollisionInfo>();
        _constraints = new Array<JConstraint>();
        _nonCollidables = new Array<RigidBody>();
        _collideBodies = new Array<RigidBody>();

        _storedPositionForActivation = new Vector3D();
        _bodiesToBeActivatedOnMovement = new Array<RigidBody>();
        _lastPositionForDeactivation = _currState.position.clone();
        _lastOrientationForDeactivation = _currState.orientation.clone();
        
        _type = "Object3D";
        _boundingSphere = JMath3D.NUM_HUGE;
        _boundingBox = new JAABox();
        
        externalData = null;

        _onCollisionStart = null;
        _onCollisionEnd = null;
    }
    
    private function radiansToDegrees(rad : Float) : Float
    {
        return rad * 180 / Math.PI;
    }
    
    private function degreesToRadians(deg : Float) : Float
    {
        return deg * Math.PI / 180;
    }
    
    //update rotation values (rotationX, rotationY, rotationZ) to the actual current values,
    // these values are not updated based on real-time physics changes
    // This is currently not done in real-time for performance reasons
    // Useful once you have a physics objects that is no longer in the physics system but you want to manipulate it's rotation.
    // Only needs to be called once after disabling from physics system
    public function updateRotationValues() : Void
    {
        var rotationVector : Vector3D = _currState.orientation.decompose()[1];
        _rotationX = formatRotation(radiansToDegrees(rotationVector.x));
        _rotationY = formatRotation(radiansToDegrees(rotationVector.y));
        _rotationZ = formatRotation(radiansToDegrees(rotationVector.z));
    }
    
    //from FIVe3D library - InternalUtils (MIT License)
    private static function formatRotation(angle : Float) : Float
    {
        if (angle >= -180 && angle <= 180) 
            return angle;
        
        var angle2 : Float = angle % 360;
        if (angle2 < -180) 
            return angle2 + 360;
        
        if (angle2 > 180) 
            return angle2 - 360;
        
        return angle2;
    }
    
    private function get_rotationX() : Float
    {
        return _rotationX;
    }
    
    private function get_rotationY() : Float
    {
        return _rotationY;
    }
    
    private function get_rotationZ() : Float
    {
        return _rotationZ;
    }
    
    /**
		 * px - angle in Radians or Degrees
		 */
    private function set_rotationX(px : Float) : Float
    {
        //var rad:Number = (_useDegrees) ? degreesToRadians(px) : px;
        _rotationX = px;
        setOrientation(createRotationMatrix());
        return px;
    }
    
    /**
		 * py - angle in Radians or Degrees
		 */
    private function set_rotationY(py : Float) : Float
    {
        //var rad:Number = (_useDegrees) ? degreesToRadians(py) : py;
        _rotationY = py;
        setOrientation(createRotationMatrix());
        return py;
    }
    
    /**
		 * pz - angle in Radians or Degrees
		 */
    private function set_rotationZ(pz : Float) : Float
    {
        //var rad:Number = (_useDegrees) ? degreesToRadians(pz) : pz;
        _rotationZ = pz;
        setOrientation(createRotationMatrix());
        return pz;
    }
    
    public function pitch(rot : Float) : Void
    {
        setOrientation(JMatrix3D.getAppendMatrix3D(currentState.orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.X_AXIS)));
    }
    
    public function yaw(rot : Float) : Void
    {
        setOrientation(JMatrix3D.getAppendMatrix3D(currentState.orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Y_AXIS)));
    }
    
    public function roll(rot : Float) : Void
    {
        setOrientation(JMatrix3D.getAppendMatrix3D(currentState.orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Z_AXIS)));
    }
    
    private function createRotationMatrix() : Matrix3D
    {
        var matrix3D : Matrix3D = new Matrix3D();
        matrix3D.appendRotation(_rotationX, Vector3D.X_AXIS);
        matrix3D.appendRotation(_rotationY, Vector3D.Y_AXIS);
        matrix3D.appendRotation(_rotationZ, Vector3D.Z_AXIS);
        return matrix3D;
    }
    
    public function setOrientation(orient : Matrix3D) : Void
    {
        _currState.orientation = orient.clone();
        updateInertia();
        updateState();
    }
    
    private function get_x() : Float
    {
        return _currState.position.x;
    }
    
    private function get_y() : Float
    {
        return _currState.position.y;
    }
    
    private function get_z() : Float
    {
        return _currState.position.z;
    }
    
    private function set_x(px : Float) : Float
    {
        _currState.position.x = px;
        updateState();
        return px;
    }
    
    private function set_y(py : Float) : Float
    {
        _currState.position.y = py;
        updateState();
        return py;
    }
    
    private function set_z(pz : Float) : Float
    {
        _currState.position.z = pz;
        updateState();
        return pz;
    }
    
    public function moveTo(pos : Vector3D) : Void
    {
        _currState.position = pos.clone();
        updateState();
    }
    
    private function updateState() : Void
    {
        _currState.linVelocity.setTo(0, 0, 0);
        _currState.rotVelocity.setTo(0, 0, 0);
        copyCurrentStateToOld();
        updateBoundingBox();  // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?  
        updateObject3D();
        
        if (collisionSystem != null) {
            collisionSystem.collisionSkinMoved(this);
        }
    }
    
    public function setLineVelocity(vel : Vector3D) : Void
    {
        _currState.linVelocity = vel.clone();
    }
    
    public function setAngleVelocity(angVel : Vector3D) : Void
    {
        _currState.rotVelocity = angVel.clone();
    }
    
    public function setLineVelocityAux(vel : Vector3D) : Void
    {
        _currLinVelocityAux = vel.clone();
    }
    
    public function setAngleVelocityAux(angVel : Vector3D) : Void
    {
        _currRotVelocityAux = angVel.clone();
    }
    
    /**
		 * CallBack from PhysicsSystem to update gravity force only when gravity or mass is dirty.
		 * @param gravity
		 */
    public function updateGravity(gravity : Vector3D, gravityAxis : Int) : Void
    {
        _gravity = gravity;
        _gravityAxis = gravityAxis;
        
        _gravityForce = JNumber3D.getScaleVector(_gravity, _mass);
    }
    
    public function addWorldTorque(t : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
        {
            return;
        }
        _torque = _torque.add(t);
        
        if (active)             setActive();
    }
    
    public function addBodyTorque(t : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
            return;
        
        addWorldTorque(_currState.orientation.transformVector(t), active);
    }
    
    // functions to add forces in the world coordinate frame
    public function addWorldForce(f : Vector3D, p : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
            return;
        
        _force = _force.add(f);
        addWorldTorque(p.subtract(_currState.position).crossProduct(f));
        
        if (active)             setActive();
    }
    
    // functions to add forces in the body coordinate frame
    public function addBodyForce(f : Vector3D, p : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
            return;
        
        f = _currState.orientation.transformVector(f);
        p = _currState.orientation.transformVector(p);
        addWorldForce(f, _currState.position.add(p), active);
    }
    
    // This just sets all forces etc to zero.
    public function clearForces() : Void
    {
        _force.setTo(0, 0, 0);
        _torque.setTo(0, 0, 0);
    }
    
    // functions to add impulses in the world coordinate frame
    public function applyWorldImpulse(impulse : Vector3D, pos : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
        {
            return;
        }
        _currState.linVelocity = _currState.linVelocity.add(JNumber3D.getScaleVector(impulse, _invMass));
        
        var rotImpulse : Vector3D = pos.subtract(_currState.position).crossProduct(impulse);
        rotImpulse = _worldInvInertia.transformVector(rotImpulse);
        _currState.rotVelocity = _currState.rotVelocity.add(rotImpulse);
        
        if (active)             setActive();
    }
    
    public function applyWorldImpulseAux(impulse : Vector3D, pos : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
        {
            return;
        }
        _currLinVelocityAux = _currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, _invMass));
        
        var rotImpulse : Vector3D = pos.subtract(_currState.position).crossProduct(impulse);
        rotImpulse = _worldInvInertia.transformVector(rotImpulse);
        _currRotVelocityAux = _currRotVelocityAux.add(rotImpulse);
        
        if (active)             setActive();
    }
    
    // functions to add impulses in the body coordinate frame
    public function applyBodyWorldImpulse(impulse : Vector3D, delta : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
        {
            return;
        }
        _currState.linVelocity = _currState.linVelocity.add(JNumber3D.getScaleVector(impulse, _invMass));
        
        var rotImpulse : Vector3D = delta.crossProduct(impulse);
        rotImpulse = _worldInvInertia.transformVector(rotImpulse);
        _currState.rotVelocity = _currState.rotVelocity.add(rotImpulse);
        
        if (active)             setActive();
    }
    
    public function applyBodyWorldImpulseAux(impulse : Vector3D, delta : Vector3D, active : Bool = true) : Void
    {
        if (!_movable) 
        {
            return;
        }
        _currLinVelocityAux = _currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, _invMass));
        
        var rotImpulse : Vector3D = delta.crossProduct(impulse);
        rotImpulse = _worldInvInertia.transformVector(rotImpulse);
        _currRotVelocityAux = _currRotVelocityAux.add(rotImpulse);
        
        if (active)             setActive();
    }
    
    // implementation updates the velocity/angular rotation with the force/torque.
    public function updateVelocity(dt : Float) : Void
    {
        if (!_movable || !isActive) 
            return;
        
        _currState.linVelocity = _currState.linVelocity.add(JNumber3D.getScaleVector(_force, _invMass * dt));
        
        var rac : Vector3D = JNumber3D.getScaleVector(_torque, dt);
        rac = _worldInvInertia.transformVector(rac);
        _currState.rotVelocity = _currState.rotVelocity.add(rac);
    }
    
    /*
		   // implementation updates the position/orientation with the current velocties.
		   public function updatePosition(dt:Number):void
		   {
		   if (!_movable || !isActive)
		   {
		   return;
		   }

		   var angMomBefore:Vector3D = _currState.rotVelocity.clone();
		   angMomBefore = _worldInertia.transformVector(angMomBefore);

		   _currState.position = _currState.position.add(JNumber3D.getScaleVector(_currState.linVelocity, dt));

		   var dir:Vector3D = _currState.rotVelocity.clone();
		   var ang:Number = dir.length;
		   if (ang > 0)
		   {
		   dir.normalize();
		   ang *= dt;
		   var rot:JMatrix3D = JMatrix3D.rotationMatrix(dir.x, dir.y, dir.z, ang);
		   _currState.orientation = JMatrix3D.getMatrix3D(JMatrix3D.multiply(rot, JMatrix3D.getJMatrix3D(_currState.orientation)));
		   updateInertia();
		   }

		   angMomBefore = _worldInvInertia.transformVector(angMomBefore);
		   _currState.rotVelocity = angMomBefore.clone();
		   }
		 */
    
    // Updates the position with the auxiliary velocities, and zeros them
    public function updatePositionWithAux(dt : Float) : Void
    {
        if (!_movable || !isActive) 
        {
            _currLinVelocityAux.setTo(0, 0, 0);
            _currRotVelocityAux.setTo(0, 0, 0);
            return;
        }
        
        var ga : Int = _gravityAxis;
        if (ga != -1) 
        {
            var arr : Array<Float> = JNumber3D.toArray(_currLinVelocityAux);
            arr[(ga + 1) % 3] *= 0.1;
            arr[(ga + 2) % 3] *= 0.1;
            JNumber3D.copyFromArray(_currLinVelocityAux, arr);
        }
        
        _currState.position = _currState.position.add(JNumber3D.getScaleVector(_currState.linVelocity.add(_currLinVelocityAux), dt));
        
        var dir : Vector3D = _currState.rotVelocity.add(_currRotVelocityAux);
        var ang : Float = dir.length * 180 / Math.PI;
        if (ang > 0) 
        {
            dir.normalize();
            ang *= dt;
            
            
            var rot : Matrix3D = JMatrix3D.getRotationMatrix(dir.x, dir.y, dir.z, ang);
            _currState.orientation = JMatrix3D.getAppendMatrix3D(_currState.orientation, rot);
            
            updateInertia();
        }
        
        _currLinVelocityAux.setTo(0, 0, 0);
        _currRotVelocityAux.setTo(0, 0, 0);
    }
    
    // function provided for the use of Physics system
    public function tryToFreeze(dt : Float) : Void
    {
        if (!_movable || !isActive) 
        {
            return;
        }
        
        if (_currState.position.subtract(_lastPositionForDeactivation).length > JConfig.posThreshold) 
        {
            _lastPositionForDeactivation = _currState.position.clone();
            _inactiveTime = 0;
            return;
        }
        
        var ot : Float = JConfig.orientThreshold;
        var deltaMat : Matrix3D = JMatrix3D.getSubMatrix(_currState.orientation, _lastOrientationForDeactivation);
        
        var cols : Array<Vector3D> = JMatrix3D.getCols(deltaMat);
        
        if (cols[0].length > ot || cols[1].length > ot || cols[2].length > ot) 
        {
            _lastOrientationForDeactivation = _currState.orientation.clone();
            _inactiveTime = 0;
            return;
        }
        
        if (getShouldBeActive()) 
        {
            return;
        }
        
        _inactiveTime += dt;
        if (_inactiveTime > JConfig.deactivationTime) 
        {
            _lastPositionForDeactivation = _currState.position.clone();
            _lastOrientationForDeactivation = _currState.orientation.clone();
            setInactive();
        }
    }
    
    public function postPhysics(dt : Float) : Void
    {
        if (!_movable || !isActive) 
        {
            return;
        }
        
        limitVel();
        limitAngVel();
        
        updatePositionWithAux(dt);
        updateBoundingBox();  // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?  
        updateObject3D();
        
        if (collisionSystem != null) {
            collisionSystem.collisionSkinMoved(this);
        }
        
        clearForces();
        
        //add gravity
        _force = _force.add(_gravityForce);
    }
    
    private function set_mass(m : Float) : Float
    {
        _mass = m;
        _invMass = 1 / m;
        setInertia(getInertiaProperties(m));
        
        // mass is dirty have to recalculate gravity force
        var physicsSystem : PhysicsSystem = PhysicsSystem.getInstance();
        updateGravity(physicsSystem.gravity, physicsSystem.gravityAxis);
        return m;
    }
    
    public function setInertia(matrix3D : Matrix3D) : Void
    {
        _bodyInertia = matrix3D.clone();
        _bodyInvInertia = JMatrix3D.getInverseMatrix(_bodyInertia.clone());
        
        updateInertia();
    }
    
    public function updateInertia() : Void
    {
        _invOrientation = JMatrix3D.getTransposeMatrix(_currState.orientation);
        
        _worldInertia = JMatrix3D.getAppendMatrix3D(_invOrientation, JMatrix3D.getAppendMatrix3D(_currState.orientation, _bodyInertia));
        
        _worldInvInertia = JMatrix3D.getAppendMatrix3D(_invOrientation, JMatrix3D.getAppendMatrix3D(_currState.orientation, _bodyInvInertia));
    }
    
    // for deactivation
    public var isActive : Bool;
    
    // prevent velocity updates etc
    private function get_movable() : Bool
    {
        return _movable;
    }
    
    private function set_movable(mov : Bool) : Bool
    {
        if (_type == "PLANE" || _type == "TERRAIN" || _type == "TRIANGLEMESH") 
            return false;
        
        _movable = mov;
        isActive = mov;
        _origMovable = mov;
        return mov;
    }
    
    public function internalSetImmovable() : Void
    {
        _origMovable = _movable;
        _movable = false;
    }
    
    public function internalRestoreImmovable() : Void
    {
        _movable = _origMovable;
    }
    
    public function setActive() : Void
    {
        if (_movable) 
        {
            if (isActive)                 return;
            _inactiveTime = 0;
            isActive = true;
        }
    }
    
    public function setInactive() : Void
    {
        if (_movable) {
            _inactiveTime = JConfig.deactivationTime;
            isActive = false;
        }
    }
    
    // Returns the velocity of a point at body-relative position
    public function getVelocity(relPos : Vector3D) : Vector3D
    {
        return _currState.linVelocity.add(_currState.rotVelocity.crossProduct(relPos));
    }
    
    // As GetVelocity but just uses the aux velocities
    public function getVelocityAux(relPos : Vector3D) : Vector3D
    {
        return _currLinVelocityAux.add(_currRotVelocityAux.crossProduct(relPos));
    }
    
    // indicates if the velocity is above the threshold for freezing
    public function getShouldBeActive() : Bool
    {
        return ((_currState.linVelocity.length > JConfig.velThreshold) || (_currState.rotVelocity.length > JConfig.angVelThreshold));
    }
    
    public function getShouldBeActiveAux() : Bool
    {
        return ((_currLinVelocityAux.length > JConfig.velThreshold) || (_currRotVelocityAux.length > JConfig.angVelThreshold));
    }
    
    // damp movement as the body approaches deactivation
    public function dampForDeactivation() : Void
    {
        _currState.linVelocity.x *= _linVelDamping.x;
        _currState.linVelocity.y *= _linVelDamping.y;
        _currState.linVelocity.z *= _linVelDamping.z;
        _currState.rotVelocity.x *= _rotVelDamping.x;
        _currState.rotVelocity.y *= _rotVelDamping.y;
        _currState.rotVelocity.z *= _rotVelDamping.z;
        
        _currLinVelocityAux.x *= _linVelDamping.x;
        _currLinVelocityAux.y *= _linVelDamping.y;
        _currLinVelocityAux.z *= _linVelDamping.z;
        _currRotVelocityAux.x *= _rotVelDamping.x;
        _currRotVelocityAux.y *= _rotVelDamping.y;
        _currRotVelocityAux.z *= _rotVelDamping.z;
        
        var r : Float = 0.5;
        var frac : Float = _inactiveTime / JConfig.deactivationTime;
        if (frac < r) 
            return;
        
        var scale : Float = 1 - ((frac - r) / (1 - r));
        if (scale < 0) 
        {
            scale = 0;
        }
        else if (scale > 1) 
        {
            scale = 1;
        }
        
        _currState.linVelocity.scaleBy(scale);
        _currState.rotVelocity.scaleBy(scale);
    }
    
    // function provided for use of physics system. Activates any
    // body in its list if it's moved more than a certain distance,
    // in which case it also clears its list.
    public function doMovementActivations(physicsSystem : PhysicsSystem) : Void
    {
        if (_bodiesToBeActivatedOnMovement.length == 0 || _currState.position.subtract(_storedPositionForActivation).length < JConfig.posThreshold) 
            return;
        
        for (body in _bodiesToBeActivatedOnMovement)
        {
            physicsSystem.activateObject(body);
        }
        
        //_bodiesToBeActivatedOnMovement.length = 0;
        _bodiesToBeActivatedOnMovement.splice(0, _bodiesToBeActivatedOnMovement.length);
    }
    
    // adds the other body to the list of bodies to be activated if
    // this body moves more than a certain distance from either a
    // previously stored position, or the position passed in.
    public function addMovementActivation(pos : Vector3D, otherBody : RigidBody) : Void
    {
        if (_bodiesToBeActivatedOnMovement.indexOf(otherBody) > -1) 
            return;
        
        if (_bodiesToBeActivatedOnMovement.length == 0) 
            _storedPositionForActivation = pos;
        
        _bodiesToBeActivatedOnMovement.push(otherBody);
    }
    
    // Marks all constraints/collisions as being unsatisfied
    public function setConstraintsAndCollisionsUnsatisfied() : Void
    {
        for (_constraint in _constraints)
        _constraint.satisfied = false;
        
        for (_collision in collisions)
        _collision.satisfied = false;
    }
    
    public function segmentIntersect(out : CollOutData, seg : JSegment, state : PhysicsState) : Bool
    {
        return false;
    }
    
    public function getInertiaProperties(m : Float) : Matrix3D
    {
        return new Matrix3D();
    }
    
    private function updateBoundingBox() : Void
    {
        
    }
    
    public function hitTestObject3D(obj3D : RigidBody) : Bool
    {
        var num1 : Float;
        var num2 : Float;
        num1 = _currState.position.subtract(obj3D.currentState.position).length;
        num2 = _boundingSphere + obj3D.boundingSphere;
        
        if (num1 <= num2) 
            return true;
        
        return false;
    }
    
    //disable the collision between two bodies
    public function disableCollisions(body : RigidBody) : Void
    {
        if (_nonCollidables.indexOf(body) < 0) 
            _nonCollidables.push(body);
    }
    
    //enable the collision between disabled bodies
    public function enableCollisions(body : RigidBody) : Void
    {
        if (_nonCollidables.indexOf(body) >= 0) 
            _nonCollidables.splice(_nonCollidables.indexOf(body), 1);
    }
    
    
    //do not call this function youself, this just used in collision system.
    public function addCollideBody(body : RigidBody) : Void{
        if (_collideBodies.indexOf(body) < 0) {
            
            _collideBodies.push(body);
            
            var event : JCollisionEvent = new JCollisionEvent(JCollisionEvent.COLLISION_START);
            event.body = body;
            #if JIGLIB_FLASH_EVENTS
            this.dispatchEvent(event);
            #end
            if (_onCollisionStart != null)
                _onCollisionStart(event);
        }
    }
    
    //do not call this function youself, this just used in collision system.
    public function removeCollideBodies(body : RigidBody) : Void{
        var i : Int = _collideBodies.indexOf(body);
        if (i >= 0) {
            
            _collideBodies.splice(i, 1);
            
            var event : JCollisionEvent = new JCollisionEvent(JCollisionEvent.COLLISION_END);
            event.body = body;
            #if JIGLIB_FLASH_EVENTS
            this.dispatchEvent(event);
            #end
            if (_onCollisionEnd != null)
                _onCollisionEnd(event);
        }
    }
    
    //add a constraint to this rigid body, do not need call this function yourself,
    //this just used in constraint system
    public function addConstraint(constraint : JConstraint) : Void
    {
        if (_constraints.indexOf(constraint) < 0) 
        {
            _constraints.push(constraint);
        }
    }
    
    //remove a constraint from this rigid body, do not need call this function yourself,
    //this just used in constraint system
    public function removeConstraint(constraint : JConstraint) : Void
    {
        if (_constraints.indexOf(constraint) >= 0) 
        {
            _constraints.splice(_constraints.indexOf(constraint), 1);
        }
    }
    
    
    // copies the current position etc to old - normally called only by physicsSystem.
    public function copyCurrentStateToOld() : Void
    {
        _oldState.position = _currState.position.clone();
        _oldState.orientation = _currState.orientation.clone();
        _oldState.linVelocity = _currState.linVelocity.clone();
        _oldState.rotVelocity = _currState.rotVelocity.clone();
    }
    
    // Copy our current state into the stored state
    public function storeState() : Void
    {
        _storeState.position = _currState.position.clone();
        _storeState.orientation = _currState.orientation.clone();
        _storeState.linVelocity = _currState.linVelocity.clone();
        _storeState.rotVelocity = _currState.rotVelocity.clone();
    }
    
    // restore from the stored state into our current state.
    public function restoreState() : Void
    {
        _currState.position = _storeState.position.clone();
        _currState.orientation = _storeState.orientation.clone();
        _currState.linVelocity = _storeState.linVelocity.clone();
        _currState.rotVelocity = _storeState.rotVelocity.clone();
        
        updateInertia();
    }
    
    // the "working" state
    private function get_currentState() : PhysicsState
    {
        return _currState;
    }
    
    // the previous state - copied explicitly using copyCurrentStateToOld
    private function get_oldState() : PhysicsState
    {
        return _oldState;
    }
    
    private function get_id() : Int
    {
        return _id;
    }
    
    private function get_type() : String
    {
        return _type;
    }
    
    private function get_skin() : ISkin3D
    {
        return _skin;
    }
    
    private function get_boundingSphere() : Float
    {
        return _boundingSphere;
    }
    
    private function get_boundingBox() : JAABox
    {
        return _boundingBox;
    }
    
    // force in world frame
    private function get_force() : Vector3D
    {
        return _force;
    }
    
    // torque in world frame
    private function get_mass() : Float
    {
        return _mass;
    }
    
    private function get_invMass() : Float
    {
        return _invMass;
    }
    
    // inertia tensor in world space
    private function get_worldInertia() : Matrix3D
    {
        return _worldInertia;
    }
    
    // inverse inertia in world frame
    private function get_worldInvInertia() : Matrix3D
    {
        return _worldInvInertia;
    }
    
    private function get_nonCollidables() : Array<RigidBody>
    {
        return _nonCollidables;
    }
    
    private function get_constraints() : Array<JConstraint>{
        return _constraints;
    }
    
    //every dimension should be set to 0-1;
    private function set_linVelocityDamping(vel : Vector3D) : Vector3D
    {
        _linVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
        _linVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
        _linVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);
        return vel;
    }
    
    private function get_linVelocityDamping() : Vector3D
    {
        return _linVelDamping;
    }
    
    //every dimension should be set to 0-1;
    private function set_rotVelocityDamping(vel : Vector3D) : Vector3D
    {
        _rotVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
        _rotVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
        _rotVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);
        return vel;
    }
    
    private function get_rotVelocityDamping() : Vector3D
    {
        return _rotVelDamping;
    }
    
    //limit the max value of body's line velocity
    private function set_maxLinVelocities(vel : Vector3D) : Vector3D
    {
        _maxLinVelocities = new Vector3D(Math.abs(vel.x), Math.abs(vel.y), Math.abs(vel.z));
        return vel;
    }
    
    private function get_maxLinVelocities() : Vector3D
    {
        return _maxLinVelocities;
    }
    
    //limit the max value of body's angle velocity
    private function set_maxRotVelocities(vel : Vector3D) : Vector3D
    {
        _maxRotVelocities = new Vector3D(Math.abs(vel.x), Math.abs(vel.y), Math.abs(vel.z));
        return vel;
    }
    
    private function get_maxRotVelocities() : Vector3D
    {
        return _maxRotVelocities;
    }
    
    private function limitVel() : Void
    {
        _currState.linVelocity.x = JMath3D.getLimiteNumber(_currState.linVelocity.x, -_maxLinVelocities.x, _maxLinVelocities.x);
        _currState.linVelocity.y = JMath3D.getLimiteNumber(_currState.linVelocity.y, -_maxLinVelocities.y, _maxLinVelocities.y);
        _currState.linVelocity.z = JMath3D.getLimiteNumber(_currState.linVelocity.z, -_maxLinVelocities.z, _maxLinVelocities.z);
    }
    
    private function limitAngVel() : Void
    {
        var fx : Float = Math.abs(_currState.rotVelocity.x) / _maxRotVelocities.x;
        var fy : Float = Math.abs(_currState.rotVelocity.y) / _maxRotVelocities.y;
        var fz : Float = Math.abs(_currState.rotVelocity.z) / _maxRotVelocities.z;
        var f : Float = Math.max(fx, Math.max(fy, fz));
        
        if (f > 1) 
            _currState.rotVelocity = JNumber3D.getDivideVector(_currState.rotVelocity, f);
    }
    
    public function getTransform() : Matrix3D
    {
        return (_skin != null) ? _skin.transform : null;
    }
    
    //update skin
    private function updateObject3D() : Void
    {
        if (_skin != null)
        {
            _skin.transform = JMatrix3D.getAppendMatrix3D(_currState.orientation, JMatrix3D.getTranslationMatrix(_currState.position.x, _currState.position.y, _currState.position.z));
        }
    }
    
    private function get_material() : MaterialProperties
    {
        return _material;
    }
    
    //coefficient of elasticity
    private function get_restitution() : Float
    {
        return _material.restitution;
    }
    
    private function set_restitution(restitution : Float) : Float
    {
        _material.restitution = JMath3D.getLimiteNumber(restitution, 0, 1);
        return restitution;
    }
    
    //coefficient of friction
    private function get_friction() : Float
    {
        return _material.friction;
    }
    
    private function set_friction(friction : Float) : Float
    {
        _material.friction = JMath3D.getLimiteNumber(friction, 0, 1);
        return friction;
    }

    private function get_onCollisionStart() : JCollisionEvent->Void
    {
        return _onCollisionStart;
    }
    private function set_onCollisionStart(v:JCollisionEvent->Void) : JCollisionEvent->Void
    {
        return _onCollisionStart = v;
    }

    private function get_onCollisionEnd() : JCollisionEvent->Void
    {
        return _onCollisionEnd;
    }
    private function set_onCollisionEnd(v:JCollisionEvent->Void) : JCollisionEvent->Void
    {
        return _onCollisionEnd = v;
    }
}
