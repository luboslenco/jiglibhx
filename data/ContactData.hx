package jiglib.data;

import jiglib.physics.BodyPair;
import jiglib.physics.CachedImpulse;

class ContactData
{
    public var pair : BodyPair;
    public var impulse : CachedImpulse;

    public function new()
    {
    }
}
