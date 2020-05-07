Loop Functions XML Configuration
================================

The following root XML tags are defined under ``<loop_functions>`` in addition to the ones specified in :xref:`COSM`.

+-------------------------+----------------------------+-------------------------------------------------------------------------------------------------------------+
| Root XML tag            | Mandatory For?             | Description                                                                                                 |
+-------------------------+----------------------------+-------------------------------------------------------------------------------------------------------------+
| ``structure3D_builder`` | All controllers            | Parameters for the structure builder.                                                                       |
+-------------------------+----------------------------+-------------------------------------------------------------------------------------------------------------+
| ``construct_targets``   | All controllers            | Parameters defining the shape/size of the structures to be built.                                           |
+-------------------------+----------------------------+-------------------------------------------------------------------------------------------------------------+

Any of the following attributes can be added under the ``metrics`` tag in place
of one of the ``<append>,<create>,<truncate>`` tags, in addition to the ones
specified in :xref:`COSM`. Not defining them disables metric collection of the
given type.

+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| XML attribute                                  | Description                                                                   | Additional Notes                                 |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``block_manipulation``                         | Free block pickup/block placement counts/penalties.                           |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``structure_progress``                         | Counts of block placement for the within the structure.                       |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``structure_state``                            | 3D occupancy map of cells within the structure with block placement status.   |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+
| ``structure_subtargets``                       | Counts of block placement/progress for all subtargets within the structure.   |                                                  |
+------------------------------------------------+-------------------------------------------------------------------------------+--------------------------------------------------+


``structure3D_builder``
-----------------------

- Required by: all.
- Required child attributes if present: [ ``build_src`` ].
- Required child tags if present: none.
- Optional child attributes: [ ``static_build_interval``,
  ``static_build_interval_count`` ].
- Optional child tags: [ ``usage_penalty`` ].

XML configuration:

.. code-block:: XML

   <structure3D_builder
       build_src="loop|robot"
       static_bulid_interval="INT"
       static_build_interval_count="INT"
   />

- ``build_src`` - The source of block placements for the structure. Valid values
  are:

  - ``loop`` - The structure will be built statically each timestep by the loop
    functions without robot involvement. Mainly intended as a debugging tool for
    initial bring up and structure invariant testing.

  - ``robot`` - The structure will be built by robots.

- ``static_build_interval`` - How many timesteps between invocations of the
  static builder in the loop functions. Only used if ``build_src`` is
  ``loop``. Defaults to 1 if omitted.

- ``static_build_count`` - How blocks to place on the structure when the builder
  is invoked at the start of an interval. Only used if ``build_src`` is
  ``loop``. Defaults to 1 if omitted.


``construct_targets``
---------------------
- Required by: all.
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``ramp``, ``rectprism`` ].

XML configuration:

.. code-block:: XML

   <construct_targets>
       <ramp>
           ...
       </ramp>
       <rectprism>
           ...
       </rectprism>
       ...
   </construct_targets>


- ``ramp`` - Defines a ramp subtarget.

- ``rectprism`` - Defines a rectangular prism subtarget.

``construct_targets/ramp``
^^^^^^^^^^^^^^^^^^^^^^^^^^

- Required by: [none].
- Required child attributes if present: [ ``anchor``, ``grid``, ``id``,
  ``orientation`` ].
- Required child tags if present: [ ``ramp_blocks``, ``cube_blocks`` ].
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <construct_targets>
       ...
       <ramp anchor="FLOAT,FLOAT,FLOAT"
             id="ramp0"
             orientation="FLOAT">
           <grid>
               ...
           </grid>
           <ramp_blocks>
               ...
           </ramp_blocks>
           <cube_blocks>
               ...
           </cube_blocks>
       </ramp>
       ...
   </construct_targets>

- ``anchor`` - X,Y,Z coordinates of the lower left hand corner of the structure
  specifying its absolute location in the arena.

- ``id`` - A UUID for the structure.

- ``orientation`` - The angle in radians between the X axis of the structure and
  the X axis of the arena. Can be 0 or pi/2; other values will cause an error.

``construct_targets/ramp/grid``
"""""""""""""""""""""""""""""""

- Required by: all.
- Required child attributes if present: [ ``resolution``, ``size`` ].
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <ramp>
       ...
       <grid
           resolution="FLOAT"
           size="X, Y, Z"/>
       ...
   </ramp>

- ``resolution`` - The resolution that the structure will be represented at, in
  terms of the size of grid cells. Must be the same as the value passed to the
  robot controllers.

- ``size`` - The size of the bounding box containing the ramp structure.

``construct_targets/ramp/ramp_blocks``
""""""""""""""""""""""""""""""""""""""

- Required by: [none].
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``ramp_block`` ].

XML configuration:

.. code-block:: XML

   <ramp>
       ...
       <ramp_blocks>
           <ramp_block cell="INT,INT,INT" id="0">
           <ramp_block cell="INT,INT,INT" id="1">
           ...
       </ramp_blocks>
       ...
   </ramp>


Defines the ramp blocks needed to construct the specified ramp construction
target. The X,Y,Z coordinates for each ``cell`` cell attribute are RELATIVE to
the structure ``anchor`` (0,0,0 by convention).

``construct_targets/ramp/cube_blocks``
""""""""""""""""""""""""""""""""""""""

- Required by: [none].
- Required child attributes if present: none.
- Required child tags if present: none.
- Optional child attributes: none.
- Optional child tags: [ ``cube_block`` ].

XML configuration:

.. code-block:: XML

   <ramp>
       ...
       <cube_blocks>
           <cube_block cell="INT,INT,INT" id="0">
           <cube_block cell="INT,INT,INT" id="1">
           ...
       </cube_blocks>
       ...
   </ramp>


Defines the cube blocks needed to construct the specified ramp construction
target. The X,Y,Z coordinates for each cube block ``cell`` attribute are RELATIVE to
the structure ``anchor`` (0,0,0 by convention).

``construct_targets/cube/cube_blocks``
""""""""""""""""""""""""""""""""""""""

Same as for ``construct_targets/ramp/cube_blocks``.
