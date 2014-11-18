package jiglib.physics.constraint;



class JConstraint
{
    public var constraintEnabled(get, never) : Bool;

    
    public var satisfied : Bool;
    private var _constraintEnabled : Bool;
    
    public function new()
    {
        
    }
    
    // prepare for applying constraints - the subsequent calls to
    // apply will all occur with a constant position i.e. precalculate
    // everything possible
    public function preApply(dt : Float) : Void
    {
        satisfied = false;
    }
    
    // apply the constraint by adding impulses. Return value
    // indicates if any impulses were applied. If impulses were applied
    // the derived class should call SetConstraintsUnsatisfied() on each
    // body that is involved.
    public function apply(dt : Float) : Bool
    {
        return false;
    }
    
    // register with the physics system
    public function enableConstraint() : Void
    {
        
    }
    
    // deregister from the physics system
    public function disableConstraint() : Void
    {
        
    }
    
    // are we registered with the physics system?
    private function get_constraintEnabled() : Bool
    {
        return _constraintEnabled;
    }
}


