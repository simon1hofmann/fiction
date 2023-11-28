Coordinate Systems
==================

**Header:** ``fiction/layouts/coordinates.hpp``

Coordinate types identify positions in a coordinate system, e.g., a Cartesian or hexagonal grid. This file provides implementations for various types of coordinates.

Offset coordinates
------------------

An offset coordinate is a coordinate that defines a location via an offset from a fixed point (origin). Cartesian coordinates are offset coordinates.

.. doxygenstruct:: fiction::offset::ucoord_t

Cube coordinates
----------------

Cube coordinates are used as a way to identify faces in a hexagonal grid. A wonderful resource on the topic is: https://www.redblobgames.com/grids/hexagons/#coordinates-cube
At the same time, they can be used to address 3-dimensional grids.

.. doxygenstruct:: fiction::cube::coord_t

SiQAD coordinates
-----------------

SiQAD coordinates are used to describe locations of Silicon Dangling Bonds on the H-Si(100) 2x1 surface were dimer columns and rows are identified by x and y values, respectively,
while the z value (0,1) points to the top or bottom Si atom in the dimer. The coordinates are originally used in the SiQAD simulator (https://github.com/siqad).

.. doxygenstruct:: fiction::siqad::coord_t

Coordinate iterator
-------------------

An iterator type that allows to enumerate coordinates in order within a boundary.

.. doxygenclass:: fiction::coord_iterator

Utility functions
-----------------

.. doxygenfunction:: fiction::area(const CoordinateType& coord) noexcept
.. doxygenfunction:: fiction::volume(const CoordinateType& coord) noexcept

.. doxygenfunction:: fiction::siqad::to_fiction_coord
.. doxygenfunction:: fiction::siqad::to_siqad_coord
