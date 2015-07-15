package jiglib.physics.constraint;

import jiglib.math.*;
import jiglib.physics.RigidBody;
import jiglib.physics.PhysicsSystem;
import jiglib.collision.CollisionInfo;

// Constrains a point within a rigid body to remain at a fixed world point
class JConstraintWorldPoint extends JConstraint
{
    public var worldPosition(get, set) : Vector3D;

    
    private inline var minVelForProcessing : Float = 0.001;
    private inline var allowedDeviation : Float = 0.01;
    private inline var timescale : Float = 4;
    
    private var _body : RigidBody;
    private var _pointOnBody : Vector3D;
    private var _worldPosition : Vector3D;
    
    // pointOnBody is in body coords
    public function new(body : RigidBody, pointOnBody : Vector3D, worldPosition : Vector3D)
    {
        super();
        _body = body;
        _pointOnBody = pointOnBody;
        _worldPosition = worldPosition;
        
        _constraintEnabled = false;
        enableConstraint();
    }
    
    private function set_WorldPosition(pos : Vector3D) : Vector3D{
        _worldPosition = pos;
        return pos;
    }
    
    private function get_WorldPosition() : Vector3D{
        return _worldPosition;
    }
    
    override public function enableConstraint() : Void
    {
        if (_constraintEnabled) 
        {
            return;
        }
        _constraintEnabled = true;
        _body.addConstraint(this);
        PhysicsSystem.getInstance().addConstraint(this);
    }
    
    override public function disableConstraint() : Void
    {
        if (!_constraintEnabled) 
        {
            return;
        }
        _constraintEnabled = false;
        _body.removeConstraint(this);
        PhysicsSystem.getInstance().removeConstraint(this);
    }
    
    override public function apply(dt : Float) : Bool{
        this.satisfied = true;
        
        var deviationDistance : Float;
        var normalVel : Float;
        var denominator : Float;
        var normalImpulse : Float;
        var dot : Float;
        var worldPos : Vector3D;
        var R : Vector3D;
        var currentVel : Vector3D;
        var desiredVel : Vector3D;
        var deviationDir : Vector3D;
        var deviation : Vector3D;
        var N : Vector3D;
        var tempV : Vector3D;
        
        worldPos = _body.currentState.orientation.transformVector(_pointOnBody);
        worldPos = worldPos.add(_body.currentState.position);
        R = worldPos.subtract(_body.currentState.position);
        currentVel = _body.currentState.linVelocity.add(_body.currentState.rotVelocity.crossProduct(R));
        
        deviation = worldPos.subtract(_worldPosition);
        deviationDistance = deviation.length;
        if (deviationDistance > allowedDeviation) {
            deviationDir = JNumber3D.getDivideVector(deviation, deviationDistance);
            desiredVel = JNumber3D.getScaleVector(deviationDir, (allowedDeviation - deviationDistance) / (timescale * dt));
        }
        else {
            desiredVel = new Vector3D();
        }
        
        N = currentVel.subtract(desiredVel);
        normalVel = N.length;
        if (normalVel < minVelForProcessing) {
            return false;
        }
        N = JNumber3D.getDivideVector(N, normalVel);
        
        tempV = R.crossProduct(N);
        tempV = _body.worldInvInertia.transformVector(tempV);
        denominator = _body.invMass + N.dotProduct(tempV.crossProduct(R));
        
        if (denominator < JMath3D.NUM_TINY) {
            return false;
        }
        
        normalImpulse = -normalVel / denominator;
        
        _body.applyWorldImpulse(JNumber3D.getScaleVector(N, normalImpulse), worldPos, false);
        
        _body.setConstraintsAndCollisionsUnsatisfied();
        this.satisfied = true;
        
        return true;
    }
}

