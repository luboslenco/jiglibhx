package jiglib.events;


//import flash.events.Event;
import jiglib.physics.RigidBody;

class JCollisionEvent //extends Event
{
    //called when the body occur a new collision
    public static inline var COLLISION_START : String = "collisionStart";
    
    //called when the body lose a collision
    public static inline var COLLISION_END : String = "collisionEnd";
    
    
    public var body : RigidBody;
    
    public function new(type : String)
    {
        //super(type);
    }
}

