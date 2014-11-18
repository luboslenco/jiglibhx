package jiglib.physics;



class MaterialProperties
{
    
    public var restitution : Float;
    public var friction : Float;
    
    public function new(_restitution : Float = 0.2, _friction : Float = 0.5)
    {
        restitution = _restitution;
        friction = _friction;
    }
}

