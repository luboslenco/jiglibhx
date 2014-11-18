package jiglib.physics;



class PhysicsController
{
    public var controllerEnabled(get, never) : Bool;

    
    private var _controllerEnabled : Bool;
    
    public function new()
    {
        _controllerEnabled = false;
    }
    
    // implement this to apply whatever forces are needed to the objects this controls
    public function updateController(dt : Float) : Void
    {
        
    }
    
    // register with the physics system
    public function enableController() : Void
    {
        
    }
    
    // deregister from the physics system
    public function disableController() : Void
    {
        
    }
    
    // are we registered with the physics system?
    private function get_controllerEnabled() : Bool
    {
        return _controllerEnabled;
    }
}

