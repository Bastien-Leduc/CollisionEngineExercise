Bugs:
	- Sat collision dectection sometimes is not working when a shape is colliding the left side of another shape.
		- This can be seen in the last sceen and activating debug mode (F4)
		- This is due to the point clipping eliminating all the potential collision points because the clipping point function detects that all the points are behind the collision edge.
	
	- Manifold is not generated correctly
		- This is due to not being fully implemented, only one point count in the collision response calculation.

	- Some rotation are incorect
		- This is due to a collision point wrongly calculated.
		- Because of that, some shape will rotate on itself, creating a Guauss-Canon phenomena and launching at high speed outside of the app view.
		- This is due to the point clipping eliminating all the potential collision points because the clipping point function detects that all the points are behind the collision edge.
