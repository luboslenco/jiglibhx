package jiglib.events;

#if JIGLIB_FLASH_EVENTS
import flash.events.Event;
#end

import jiglib.physics.RigidBody;

#if JIGLIB_FLASH_EVENTS
class JCollisionEvent extends Event
#else
class JCollisionEvent
#end
{
    //called when the body occur a new collision
    public static inline var COLLISION_START : String = "collisionStart";
    
    //called when the body lose a collision
    public static inline var COLLISION_END : String = "collisionEnd";
    
    
    public var body : RigidBody;
    
    public function new(type : String)
    {
        #if JIGLIB_FLASH_EVENTS
        super(type);
        #end
    }
}

