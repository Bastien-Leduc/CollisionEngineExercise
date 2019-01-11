Bugs:
	- Sat collision dectection sometimes is not working when a shape is colliding the left side of another shape.
		- This can be seen in the last sceen and activating debug mode (F4)
		- This is due to the point clipping eliminating all the potential collision points because the clipping point function detects that all the points are behind the collision edge.