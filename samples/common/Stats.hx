package;

import h2d.Scene;
import h2d.Text;
import h2d.Font;
import h3d.Engine;
import jiglib.collision.CollisionSystemGrid;
import jiglib.physics.PhysicsSystem;

class Stats
{
	private var parent : Scene;
	private var engine : Engine;
	private var physics : HeapsPhysics;
	private var grid : Bool;

	private var mem_max : Float;
	private var timer : Float;

	private var textFpsLabel : Text;
	private var textFps : Text;
	private var textMsLabel : Text;
	private var textMs : Text;
	private var textBottomLeft : Text;
	private var textBottomRight : Text;
	private var textBottom : Text;

	/**
	 * Stats FPS, 3D, JIGLIB, MS and MEM, etc.
	 * CDC is Collision Dection Checks
	 * Rigid bodies - active (bodies) is bodies sleep
	 * info UPDATES once per second
	 */
	public function new(parent : Scene, engine : Engine, physics : HeapsPhysics) {
		this.parent = parent;
		this.physics = physics;
		this.engine = engine;
		this.grid = Std.is(physics.engine.getCollisionSystem(), CollisionSystemGrid);

		var font = hxd.res.FontBuilder.getFont("Arial", 10);

		textFpsLabel = makeText(font, parent, 0xffffff, Left, 1, 10, 20);
		textFps = makeText(font, parent, 0x1abfff, Left, 1, 40, 20);
		textMsLabel = makeText(font, parent, 0xffffff, Left, 1, 96, 20);
		textMs = makeText(font, parent, 0xffcc1a, Left, 1, 138, 20, 44);
		textBottomLeft = makeText(font, parent, 0xaaaaaa, Right, 6, 8, 75, 100);
		textBottomRight = makeText(font, parent, 0xaaaaaa, Left, 6, 108, 75, 100);
		textBottom = makeText(font, parent, 0xaaaaaa, Center, 0, 8, 111, 160);

		textFpsLabel.text = "FPS:\nCDC:\nTRI.:";
		textMsLabel.text = "TOTAL:\nJIGLIB:\n3D:";
		textBottom.text = "CDT BRUTEFORCE";

		timer = haxe.Timer.stamp();
	}

	public function update(dt:Float) {
		var newTimer = haxe.Timer.stamp();
		if (newTimer - timer >= 0.5) {
			timer = newTimer;

			var stage = hxd.Stage.getInstance();
			var fps = Std.int(hxd.Timer.fps());
			var numCollisionsChecks = physics.engine.getCollisionSystem().numCollisionsChecks;
			var renderedFacesCount = engine.drawTriangles;

			var delta = hxd.Timer.deltaT * 1000;
			var deltaMS = Std.int(delta);
			var deltaMS3D = Std.int(delta - physics.frameTime);

			#if flash
			var mem = flash.system.System.totalMemory * 0.000000954;
			#else
			var mem = engine.mem.stats().managedMemory * 0.000000954;
			#end
			var rigidbodies = physics.engine.bodies.length;
			var active = physics.engine.activeBodies.length;
			mem_max = mem_max > mem ? mem_max : mem;

			textFps.text = '$fps / ${stage.getFrameRate()}\n$numCollisionsChecks\n$renderedFacesCount';
			textMs.text = '$deltaMS ms\n${physics.frameTime} ms\n$deltaMS3D ms';
			textBottomLeft.text = 'MEM ${toFixed(mem,2)}\nRIGIDB. $rigidbodies';
			textBottomRight.text = '/ MAX ${toFixed(mem_max,2)}\n/ ACTIVE $active';

			textBottom.text = grid ? "CDT GRID" : "CDT BRUTEFORCE";
		}
	}

	private function makeText(font:Font, parent:Scene, colorText:UInt, alignText:h2d.Text.Align, leading:Int=0, xPos:Int=0, yPos:Int=0, widthText:Int=52):Text {
		var text = new Text(font, parent);
		text.x = xPos;
		text.y = yPos;
		text.textColor = colorText;
		text.textAlign = alignText;
		text.lineSpacing = leading;
		text.maxWidth = widthText;

		return text;
	}

	private function toFixed(v:Float, fractionDigits:Int):String {
		var s = Std.string(v);
		var ptIndex = s.indexOf('.');
		return (ptIndex >= 0) ? s.substring(0, ptIndex+fractionDigits+1) : s;
	}
}
