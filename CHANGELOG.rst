^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2021-03-16)
------------------
* Noetic-compatible
* + Repredictor
* + Custom Timer library
* + Vector converter
* ... and athor 250 commits with major improvements ;-)

0.0.5 (2020-02-26)
------------------
* fixed lkf for building without optimalization (static const to static constexpr)
* Contributors: Matouš Vrba

0.0.4 (2020-02-18)
------------------
* [lkf]: fix for dynamic-sized state vector, [geometry_utils]: fixed normalize_angle!
* udpated ukf - now works for dynamic-sized observation, transition and observation models can be changed after object initialization
* Transformer: added quiet param to getTransform()
* mrs odom update
* change odometry kalman to three states
* [nckf]: small fix for dynamic-sized number of states
* [nckf]: fixed some warnings
* [drmgr]: fixed string type dynamically reconfigurable parameters
* Adding encoding for depth maps into the image publisher
* remove old ukf from build pipeline
* [transformer]: added transformHeaderless for messages without a header, added transformImpl specialization for geometry_msgs::Point
* [transformer]: reverted transforming of Eigen objects, which was causing more trouble than it solved
* [transformer]: fixed eigen matrix transformations
* [transformer]: added transformVecs for transforming Eigen stuff and getTransformEigen to mrs_lib::TransformStamped
* [transformer]: added the inverse() method to mrs_lib::TransformStamped
* playing around with the documentation
* [utils]: added wrapper for containerToString without iterators
* [utils]: changed vectorToString to containerToString, changed it to use templated iterators to be more universal
* Adding functions for easily printing contents of vectors (also works with boost::array used in ros messages)
* small fix in safety zone - arguments are now properly marked as const
* [geometry_utils]: added normalize_angle functions
* small fix in safety zone - arguments are now properly marked as const
* added missing image_transport dependency
* renamed imageTransform lib's so
* SafetyZone: added height to point obstacles
* [rheiv]: more user-friendly constructors
* [rheiv]: added optional timeout
* Consolidating image publisher topics under the debug_topics header
* Transformer: made resolveFrameName public
* changed image publisher to use smart pointers
* Changed the throttle in image publisher from macro to method. This makes it less flaky with multiple image streams from the same object
* Throttling for the image publisher now works. Additonal refinements added.
* Adding a tool for automatic publishing of image straeams for vision development
* [transformer]: added the nodiscard attribute to the respective methods
* [Transformer]: cleanup of unused methods in h file
* fixed missing check for nullopt
* Transformer: removed timeout and caching, formatting
* Transformer: fixes in latlon transform
* removed bool-returning variants of Transformer class
* fixed transformations from/to latlon frames when uav prefix is not autodeduced
* rewrote transformer to use templaates and be a bit more consistent
* updated attitude_cmd
* Transformer: throttled prints
* mrs_lib: removed caching, fixed name resolver
* [ParamLoader]: load_param now returns true if value was fetched from rosparam and false if it failed
* Transformer: updated caching time stamps
* Transformer: fixed tf cache time
* [mutex]: minor change to arguments to be consistent
* [mutex]: documented the new function
* [mutex]: added get_set_mutexed
* [drmgr]: fixed bug, now multiple namespaces can be used in parameter names
* [ParamLoader]: added loading of ros::Duration
* SafetyZone: added height to getters
* added latlon transform
* generalized the transforms
* change transformer target name
* Transformer: updated constructors
* Profiler: updated constructors and the rate type
* Transformer: removed debug prints
* Transformer: added more constructors and a check for missing uav_name\_
* added class comment to the transformer
* added the tf transformer wrapper
* updated docs
* added doxygen header to mutex.h
* added documentation
* overloaded set_mutexed to return the new values
* added set_mutexed()
* simplified get_mutexed()
* added Mutex.h with templated get_mutexed()
* added get_mutexed()
* butified cmakelists
* [shandler]: fixed printout of topic remapping
* [shandler]: fixed bug when using default construction for callbacks
* Contributors: Matej Petrlik, Matouš Vrba, Petr Štibinger, Tomas Baca, Tomáš Báča, Viktor Walter, Vojtech Spurny, afzal

0.0.3 (2019-10-25)
------------------
* [shandler]: fixed bug which caused message timeout to only be called once
* [shandler]: last_message_time() now returns even if no message was received yet
* [shandler]: put back the last_message_time() method (dunno why I put it away)
* [shandler]: added some convenient factory methods
* [shandler]: added the peek_data() method
* [shandler]: added the last_message_time() method
* added angle_between() specialization for 2D vectors, fixed some documentation
* fixed back with point obstacle intersection
* [param loader]: documented load_matrix_array methods
* [param loader]: fixed loading of array of matrices
* [param loader]: matrix loading now works also for empty matrices
* [rheiv]: updated to enable non-constant dzdx jacobian
* added function to calculate angle between two vectors
* [RHEIV]: added some convenience methods
* added some more convenience methods etc
* RHEIV: beautified the class, added documentation and some foolproofing
* [shandler]: removed unnecessary includes
* shander: removed unnecessary remove_const
* shandler: changed stuff to explicitely use ConstPtrs
* shandler: added helper macro (look into replacing it with metaprogramming)
* shandler: fixes in time_consistency
* shandler: time_consistent now seems to work!
* shandler: compilable version including time consistency
* SubscribeHandler: updating documentation, adding potentially useful methods
* SubscribeHandler: fixed small issues with example.cpp, adding docs
* added example for subscribe_handler
* rewriting to pimpl
* enabled dynamic number of states for lkf
* working on subscribe_handler
* [Subscribe handler]: added possibility to specify timeout callback
* moar documentation to kfs
* adding moar documentation (to nclkf mostly)
* added documentation to new kf and lkf implementations, added example usage for new lkf
* started documenting kf methods
* partially norm-constrained LKF tested and seems to be working nicely
* fixed param loader loading of Eigen::MatrixXd with known dimensions to be backwards compatible
* added printing of XmlRpcValues to param loader
* writing Paramloader printing of XmlRpcValue params
* fixed NCUKF correction implementation
* fixes and code cleanup in KFs
* implemented NCUKF - norm-constrained variant of UKF
* added norm-constrained lkf implementation
* Q is now scaled by dt in lkf implementation
* rewrote static matrix loading to use templates to enable loading of matrices with one zero dimension and compile-time checks
* loading of namespaced parameters from rosparam server now works automatically (with _\_ instead of /)
* added load_param2 to dynrecmgr
* changed the weight generation according to https://www.cs.ubc.ca/~murphyk/Papers/Julier_Uhlmann_mar04.pdf
* comparison of old and new UKF implementations
* added default constructor to the UKF class
* added UKF documentation and example, some refactoring
* an idiot tries to fix a bug in his code for two days. a clever man fixes the bug in his testing code. I am an idiot
* tests tend to produce nans in UKF when squaring... need to look into this
* compilable, needs a testing program to compare with old implementation
* playing around with gitlab ci
* modified the gitlab CI script to automatically rename Doxy project, now using ROS Doxyfile
* added gitlab CI integration files
* Add a constructor that takes Matrixes
* Added check for path between current position ang goto position
* Fixed visualization 0,0 bug
* + SafetyZone library
* Contributors: Andriy Dmytruk, Markiian, Matej Petrlik, Matouš Vrba, Tomas Baca, Viktor Walter, Vojtech Spurny

0.0.2 (2019-07-01)
------------------
* loading of arrays of matrices seem to work
* working on loading of vector of matrices - so far only same size matrices can be loaded
* fixed profiler's threshold bug
* Adding description of the constructor arguments
* Contributors: Matouš Vrba, Tomas Baca, Viktor Walter

0.0.1 (2019-05-20)
------------------
