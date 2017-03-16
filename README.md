# bezier
A cubic Bezier class for Unity. This code is a conversion of the Tacent C++ code for the tBezierCurve and tBezierPath classes which can be browsed here:

[Browse Tacent Source](http://upperboundsinteractive.com/Tacent/Modules/index.html)

The Bezier.cs file contains two classes, a simple BezierCurve class that implements the standard Bernstein polynomials, and a BezierPath class that essentially joins the curves together to create a longer path. Both classes implement functions to find the tangent for any t value.




### BezierCurve

In most cases you won't need to use this class directly, but you can if you like. Give it your 4 CVs, and it will let you get a point on the curve for a particular t value, get the tangent for a particular t value, and find the t value of the closest point on the curve to a point you specify.


### BezierPath
This one is much more powerful. It allows you to only construct it with the knots you want interpolated. It will compute the necessary extra CVs internally so that things remain smooth between the curve sections. You must give it 2 or more knots (I use the term knot to mean interpolated CV... as in, the path will go right through the point). For 2 knots you just get a straight-line segment. For 3 it'll be curvy.

BezierPath supports open and closed (looping) paths. You can work with t values E [0,1] for the whole path, or t E [0, numSegments]. When you see the word 'Norm' it means t E [0,1]. For open paths t will be clamped, for closed paths it will just loop around if you pass it a large t value.

The class also lets you compute what t value will result in a position on the path closest to an arbitrary position you specify. See ComputeClosestParam(). When you see the word 'Compute', worry about efficiency. ComputeClosestParam is a recursive call that terminates when a user-specified tolerance is met.



