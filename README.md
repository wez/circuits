[![Build Status](https://travis-ci.org/wez/circuits.svg?branch=master)](https://travis-ci.org/wez/circuits)

# Geom crates to evaluate:

https://github.com/georust/geo

https://docs.rs/parrot/0.1.0/parrot/geom/struct.Poly.html

https://github.com/aeickhoff/descartes


# Thoughts on handling footprints

Always try to normalize the positioning so that we consider the center point of
the physical portions of the footprint.  eg: find the centroid of the hull
described by the elements on the copper and edge cuts layers.  This should make
it a little more sane to model operations like flipping or mirroring?
Alternatively, do perform this compensation operation at the time of a flip
or mirror operation.

# Thoughts on routing traces

We already know the nets from the circuit construction, which are equivalent
to the Components that we compute in the pcbautorouter crate.

Can we do something simple and dumb for my primary use case?  For example,
compute the CDT then assign a preferred layer for that path between any
two points depending on how horizontal or vertical it is.

If multiple nets traverse the same segment of the graph we can either
just make them all follow that path but space them out (sorting them
by some metric), or we can generate a new point in the graph to split
the gap between the nodes for each path.


