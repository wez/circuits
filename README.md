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
