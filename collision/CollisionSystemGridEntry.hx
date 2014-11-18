package jiglib.collision;


import jiglib.physics.RigidBody;

class CollisionSystemGridEntry
{
    public var collisionBody : RigidBody;
    public var previous : CollisionSystemGridEntry;
    public var next : CollisionSystemGridEntry;
    public var gridIndex : Int;
    
    public function new(collisionBody : RigidBody)
    {
        this.collisionBody = collisionBody;
        this.previous = this.next = null;
    }
    
    
    /*
		* Removes the entry by updating its neighbours. Also zaps the prev/next
		* pointers in the entry, to help debugging
		*/
    public static function removeGridEntry(entry : CollisionSystemGridEntry) : Void
    {
        // link the previous to the next (may be 0)
        entry.previous.next = entry.next;
        // link the next (if it exists) to the previous.
        if (entry.next != null) 
            entry.next.previous = entry.previous;  // tidy up this entry
        
        entry.previous = entry.next = null;
        entry.gridIndex = -2;
    }
    
    /*
		* Inserts an entry after prev, updating all links
		* @param entry prev
		*/
    public static function insertGridEntryAfter(entry : CollisionSystemGridEntry, prev : CollisionSystemGridEntry) : Void
    {
        var next : CollisionSystemGridEntry = prev.next;
        prev.next = entry;
        entry.previous = prev;
        entry.next = next;
        if (next != null) 
            next.previous = entry;
        entry.gridIndex = prev.gridIndex;
    }
}
