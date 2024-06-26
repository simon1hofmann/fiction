Coordinate Systems
==================

**Header:** ``fiction/layouts/coordinates.hpp``

Coordinate types identify positions in a coordinate system, e.g., a Cartesian or hexagonal grid. This file provides implementations for various types of coordinates.

Offset coordinates
------------------

An offset coordinate is a coordinate that defines a location via an offset from a fixed point (origin). Cartesian coordinates are offset coordinates.

.. tabs::
    .. tab:: C++
        .. doxygenstruct:: fiction::offset::ucoord_t
    .. tab:: Python
        .. autoclass:: mnt.pyfiction.offset_coordinate

Cube coordinates
----------------

Cube coordinates are used as a way to identify faces in a hexagonal grid. A wonderful resource on the topic is: https://www.redblobgames.com/grids/hexagons/#coordinates-cube
At the same time, they can be used to address 3-dimensional grids.

.. tabs::
    .. tab:: C++
        .. doxygenstruct:: fiction::cube::coord_t
    .. tab:: Python
        .. autoclass:: mnt.pyfiction.cube_coordinate

SiQAD coordinates
-----------------

SiQAD coordinates are used to describe locations of Silicon Dangling Bonds on the H-Si(100) 2x1 surface were dimer columns and rows are identified by x and y values, respectively,
while the z value (0,1) points to the top or bottom Si atom in the dimer. The coordinates are originally used in the SiQAD simulator (https://github.com/siqad).

.. tabs::
    .. tab:: C++
        .. doxygenstruct:: fiction::siqad::coord_t
    .. tab:: Python
        .. autoclass:: mnt.pyfiction.siqad_coordinate

Coordinate iterator
-------------------

An iterator type that allows to enumerate coordinates in order within a boundary.

.. doxygenclass:: fiction::coord_iterator

Utility functions
-----------------

.. tabs::
    .. tab:: C++
        .. doxygenfunction:: fiction::area(const CoordinateType& coord) noexcept
        .. doxygenfunction:: fiction::volume(const CoordinateType& coord) noexcept

        .. doxygenfunction:: fiction::siqad::to_fiction_coord
        .. doxygenfunction:: fiction::siqad::to_siqad_coord

    .. tab:: Python
        .. autofunction:: mnt.pyfiction.offset_area
        .. autofunction:: mnt.pyfiction.cube_area
        .. autofunction:: mnt.pyfiction.siqad_area

        .. autofunction:: mnt.pyfiction.offset_volume
        .. autofunction:: mnt.pyfiction.cube_volume
        .. autofunction:: mnt.pyfiction.siqad_volume

        .. autofunction:: mnt.pyfiction.to_offset_coord
        .. autofunction:: mnt.pyfiction.to_cube_coord

        .. autofunction:: mnt.pyfiction.to_siqad_coord
