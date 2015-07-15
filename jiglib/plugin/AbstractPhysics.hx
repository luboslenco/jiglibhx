package jiglib.plugin;

import jiglib.physics.PhysicsSystem;
import jiglib.physics.RigidBody;

class AbstractPhysics {

    private var initTime        : Int;
    private var stepTime        : Int;
    private var speed           : Float;
    private var deltaTime       : Float = 0;
    private var physicsSystem   : PhysicsSystem;
    private var _frameTime      : UInt = 0; // time need for one frame physics step

    public var frameTime(get,never):UInt;
    public var engine(get,never):PhysicsSystem;

    public function new(speed:Float = 5) {
        this.speed = speed;
        initTime = getTimer();
        physicsSystem = PhysicsSystem.getInstance();
        physicsSystem.setCollisionSystem(false); // bruteforce
        //physicsSystem.setCollisionSystem(true,20,20,20,200,200,200); // grid
    }

    public function addBody(body:RigidBody):Void {
        physicsSystem.addBody(body);
    }

    public function removeBody(body:RigidBody):Void {
        physicsSystem.removeBody(body);
    }

    public function afterPause():Void {
        initTime = getTimer();
    }

    //if dt>0 use the static time step,otherwise use the dynamic time step
    public function step(dt:Float=0):Void {
        stepTime = getTimer();
        deltaTime = ((stepTime - initTime) / 1000) * speed;
        initTime = stepTime;
        physicsSystem.integrate((dt>0)?dt:deltaTime);
        _frameTime = getTimer()-initTime;
    }

    private function getTimer():Int {
        return Std.int(haxe.Timer.stamp() * 1000);
    }

    private function get_engine():PhysicsSystem {
        return physicsSystem;
    }

    private function get_frameTime():UInt {
        return _frameTime;
    }
}
