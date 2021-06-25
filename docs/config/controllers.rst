============================
Controller XML Configuration
============================

The following controllers are available:

 .. list-table::
    :widths: 25,50,50
    :header-rows: 1

    * - Controller

      - Required Loop Functions

      - Notes


 * - fcrw_bst

   - construction

   - CRW Foraging + single target building.


The following root XML tags are defined under ``<params>``.

.. list-table::
    :widths: 25,50,50
    :header-rows: 1

    * - Root XML Tag

      - Mandatory For?

      - Description


 * - ``output``

   -  All controllers

   - See :xref:`COSM` docs.

 * - ``sensing_subsystem2D``

   - All controllers

   - See :xref:`COSM` docs.

 * - ``actuation_subsystem2D``

   - All controllers

   - See :xref:`COSM` docs.

 * - ``lane_alloc``

   - All controllers

   - Parameters for construction lane allocation.

``lane_alloc``
--------------

- Required child attributes if present: ``policy``.
- Required child tags if present: none.
- Optional child attributes: [ ``interference_window`` ].
- Optional child tags: none.

XML configuration:

.. code-block:: XML

   <lane_alloc
       policy="random|lru|closest|min_interference">
       interference_window="INTEGER"
   />

``policy`` - The lane allocation policy to use. Valid values are:

  - ``random`` - Choose a random lane each time.

  - ``lru`` - Choose the least recently visited lane each time by allocating
    lanes in a round robin fashion. Initialized to a random lane for each robot
    to prevent crowding at the start of simulation.

  - ``closest`` - Choose the lane closest to the robot's current location when
    the allocation algorithm is run.

  - ``min_interference`` - Choose the lane which the robot has experienced the
    minimum average interference (i.e., having to wait for other robots) while
    in the lane. The average interference is calculated using a sliding window
    of ``interference_window`` timesteps. If multiple lanes have the same average, a
    random one is selected.
