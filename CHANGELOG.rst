^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package map_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.10 (2024-03-02)
------------------
* Removes unneeded opencv dependency
* Contributors: Wennie Tabib

0.0.9 (2020-03-27)
------------------
* Only insert a change into the changeset when the value actually changes (prevents tracking unchanged, clamped cells)
* Adding configurable (sim only) option to lock cells as occupied once they reach max_clamp (Fixes #14)
* Contributors: Micah Corah

0.0.8 (2019-02-11)
------------------
* volume function
* function to compute partial set difference between this bounding box and
  overlap of this and that bounding box
* Updates to intersection checks: naming, comments, checking lengths, and a bit of organization
* replace overlap with intersect function and return a Grid3D::Ptr with an empty map.
* add cfloat header to Grid3DPoint to enable compilation on Linux and Mac with c++17
* Ensure monotonically increasing timestamps in case messages are ever processed out of order
* Adding method to directly resize the map
* Removing extreneous checks
* drop the last miss from an overlength ray as the occupied portion of the cell might be past the max sensor range
* break if edge cases cause the ray to get too long, should add one cell length at most
* proper value clamping, should fix bugs that would arise from large updates
* helper to filter pointclouds by value
* Initialize the load time to a reasonable value
* Adding normalize method
* Adding ray intersection test to bounding box class
* Adding point-cloud publisher templated on filter function
* Adding toggle to publish entire map
* fixing num cells calculation
* modifying clamping behavior to permit changing occupancy values
* Contributors: Cormac OMeadhra, Micah Corah, Wennie Tabib

0.0.7 (2016-12-02)
------------------
* Adding checks for degenerate bounding box and to save grid to image
* Contributors: Vibhav Ganesh

0.0.6 (2016-04-22)
------------------
* Merge branch 'feature/ray_termination' into 'develop'
  Feature/ray termination
  See merge request !6
* Merge branch 'feature/fast_submap' into feature/ray_termination
* fast submaps in grid3d
* Merge branch 'develop' into feature/ray_termination
* including stdio.h in files with printf statements, was relying on other files to include this header
* removing terminate-on-occupied condition
* Merge branch 'feature/bugfix_extend_3d' into 'develop'
  Feature/bugfix extend 3d
  See merge request !5
* Add checks to ensure extension is required in specific directions
* Merge branch 'feature/no_csqmi' into 'develop'
  Feature/no csqmi
* consistency in getRay functions and restored terminate-on-occupied in Grid3D
* const methods
* lineSearch in Grid2D no longer reimplements raytracing as is in Grid3D
* addRay in Grid2D uses getRay to mimic Grid3D and to remove reimplementation of raytracing
* formatting and fixing differences between Grid2D and Grid3D
* removing csqmi/information components (to be moved to information_utils and refactored)
* Templatize get method
* Templatize entropy functions
* Updating additional boost constants
* boxed entropy calculation
* reserving space for cells in raycast
* removing unnecessary conditionals from raycast
* fixing bug where lower boundary cells would be pushed back repeatedly without advancing the index because the end was out of bounds and had wrapped around
* fixed bug in Grid3D in getRay where cells were being added before the bounds check
* formatting and fixing raycast issues
* Making linesearch function const
* Adding an extra 2D raycasting function.
* Contributors: Erik, Erik Nelson, Micah Corah, Nathan Michael, Wennie Tabib

0.0.5 (2015-06-15)
------------------
* Merge branch 'feature/changesets' into 'develop'
* Merge branch 'bugfix/extend' into 'feature/changesets'
* Fixing bug in extend on Grid2D.h
* Break from loop if out of map (guard against queries that would loop forever)
* Ensure boost version >= 1.50.0 when using unordered set reserve
* kdtree Ptr and ConstPtr typedef
* Use getRay method and remove redundant code in lineSearch and addRay methods
* Handle incremental cloud (map_msgs/PointCloud2Update)
* Support incremental change tracking through change sets
* Mark appropriate functions as const
* Contributors: Erik Nelson, Nathan Michael

0.0.4 (2014-11-16)
------------------
* End point checks on addRay and lineSearch methods
* Ensure that the sensor origin is in the bbx before ray query
* Contributors: Nathan Michael

0.0.3 (2014-11-04)
------------------
* Merge branch 'feature/registered_clouds' into 'develop'
  Feature/registered clouds
  See merge request !1
* Generalize to support registered clouds. Require positive max_range.
* Adding inefficient cloud insertion
* Bugfix: error in end point definition
* AddPointCloud method
* Contributors: Nathan Michael

0.0.2 (2014-10-16)
------------------
* Grid3D KDTree struct
* Grid3D line search method
* Adding missing header
* Safer bbx constructor via two points. inQ returns true if inclusive or bbx is equal
* Adding line search function
* Contributors: Nathan Michael

0.0.1 (2014-09-16)
------------------
* Merge branch 'develop' of nmichael.frc.ri.cmu.edu:utilities/map_utils into develop
* Move from powf to direct multiply operation
* Going back to boost from c++11 as linux support is limited
* Moving to c++11. Adding index free and occupied query functions. Fixing cv image load.
* Block extend support. Map extension will increase dimensions based on block size and adapt the origin accordingly. block_size = 1 is equivalent to prior approach.
* Fixing probability function and slight speed-up on getIndex and get methods
* Using row accessor for online speed-up
* Adding support for 3D maps. Similar in form to 2D structure. Note that slice is the inner loop for rapid z-traversal for a fixed x,y
* Making ifdef blocks more specific to package
* Correct range mismatch from end - start magnitude that can arise in practice
* Fixing submap issue when row or col were on max boundary. Cleaning up ray cast logic
* Query row versus individual pixels
* Removing float isless for operator
* Initial commit
* Contributors: Nathan Michael
