jiglibhx
========

A Haxe port of the [JigLibFlash](https://code.google.com/p/jiglibflash/) ActionScript 3 3D Physics Engine

Sports a good set of features but needs documentation (classic), examples and general clean up.
Haxe really could use at least one decent fully working and documented 3D physics solution - any help to make this happen is appreciated!

Compilation options
===================
Compile with `-D JIGLIB_FLASH_EVENTS` to enable the use of the Flash event
dispatching mechanism (which can be emulated via [OpenFL](https://github.com/openfl/openfl))
for listening to collision events.

Otherwise, a simple callback-based fallback (`RigidBody.onCollisionStart` and `RigidBody.onCollisionEnd`) can be used instead.
