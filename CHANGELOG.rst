^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2023-01-20)
------------------
* Remove all warnings (Eigen and pcl) (`#40 <https://github.com/ctu-mrs/mrs_lib/issues/40>`_)
  * Remove all warnings (Eigen and pcl)
  * changed pcl_include_dirs to pcl_ros_include_dir
  Co-authored-by: Matouš Vrba <vrbamato@fel.cvut.cz>
  BatchVisualizer - add automated timeout
* batch visualizer timeout update
* add timeout option to all batch visualizer objects
* [transformer]: added transformation of PointStamped
  Add missing tf_conversions dependency
* Add missing tf_conversions dependency
* [repredictor]: dumb repredictor hotfixes
* [repredictor]: dumb repredictor hotfixes
* [repredictor]: added the possibility to act as a dumb LKF for evaluation purposes
* [DynamicPublisher]: added message type checking to avoid silent fails
* [DynamicPublisher]: added documentation and example
* first version of DynamicPublisher
* minor refactor of gemoetry conversions
* [paramloader]: refactored Eigen matrix loading to enable loading of different matrix types and easier default value specification
* [subscribe handler]: peekMsg is now const
  Aloamgarm and repredictor merge
* [dynrec]: added the publish_descriptions method
* geometry updates (`#38 <https://github.com/ctu-mrs/mrs_lib/issues/38>`_)
  * geometry: make all getters const
  * BatchVisualizer: const geometry, cleanup
* added comment
* created separate class for RepredictorAloamgarm inheriting from Repredictor
* scope timer: getLifetime() now returns floating point type
* scope timer: add getLifetime() which returns object lifetime in milliseconds
* [Transformer]: finishing touches for the Eigen::Vector3d-specific methods
* [Transformer]: added the transformAsVector and transformAsPoint methods for Eigen::Vector3
* [Transformer]: solving the ambiguity of Eigen::Vector3d (point or a vector?)
* polishing aloamgarm code
* aloamgarm - polishing code
* aloamgarm - reverted filtering of all altitude measurements, removed redundant code
* aloamgarm - filtering jumps from all height measurements
* aloamgarm update - changing nis threshold
* disabled publishers in aloamgarm jlkf
* testing some lkf modifications for aloamgarm
* Contributors: Martin Pecka, Matouš Vrba, Pavel Petracek, Tomas Baca, Tomáš Báča, Vaclav Pritzl, Vojtech Spurny, Václav Pritzl, stibipet

1.0.3 (2022-05-09)
------------------
* [Transformer]: fixed some dependencies
* [Transformer]: now with cv::Point3\_<T> support!
* [cyclic]: hotfix
* [cyclic]: added the subtraction operator
* [cyclic]: added the addition operator to cyclic
* [MedianFilter]: minor refactor to be consistent with previous implementation
* [MedianFilter]: added missing header, added the initialized() method
* [MedianFilter]: fix for Melodic
* [MedianFilter]: rewrote the implementation to be clearer and consistent with the rest of mrs_lib, added documentation and tests
* [dynrec]: added possibility of using a user callback
* [Transformer]: hotfix for thread-safe copy operator
* [Math]: minor cleanup
* [Math]: added the math library, so far containing the implementation of the inverse CDF for a standard normal distribution
* [Transformer]: can now set buffer time, [ParamLoader]: added loadParam2 for raw XmlRpcValue
* added the rotateCovariance function to geometry/misc, minor cleanup of Transformer
* [Transformer]: added move assignment operator, fixed some documentation. [Geometry]: added the toEigenMatrix conversion function
* added missing test files
* updated publisher handler, added pub handler test
* updated publisher handler
* fixing overloads in publisher handler
* [transformer]: fixed some minor issues and some refactoring
* [transformer]: reintroduced a mutex for thread-safety
* [transformer]: prints throttled error when frame id is empty, other minor fixes
* added publisher handler
* [transformer]: fixed YCM warning about incomplete type
* [transformer]: final fixes after fixes
* [transformer]: fixed prefix behaviour with empty prefixes
* [transformer]: fixing switched from and to frames convention
* adding transformer example
* added publisher handler
* AttitudeConverter: test, fixed chrono
* uncommented commented tests
* improved the setHeading() test
* fixed attitude converter and transformer tests
* fixed tests
* AttitudeConverter: fixed setHeading(), removed setHeadingByYaw()
* fixed intrinsic eulers in attitude converter
* + install in cakelists
* [transformer]: updated documentation
* fixes, cleanup, added tests and documentation
* [transformer]: SFINAE magic fixed
* [transformer]: SFINAE magic not working
* [transformer]: refactoring wip
* small speedup in scope timer
* removed catkin profile from CI install.sh
  Transform component extraction
  adding transformer for Eigen::Vector3d
* Addin convenience functions to extract translation and rotation from the obtained transform in the Eigen lib format
* formatting (needed to trigger CI)
* adding transformer for Eigen::Vector3d
* removed custom build flags with libasan
* scope_timer: add ScopeTimerLogger; use chrono::steady_clock insteady of chrono::system_clock (`#34 <https://github.com/ctu-mrs/mrs_lib/issues/34>`_)
* Contributors: Matouš Vrba, Pavel Petracek, Pavel Petráček, Tomas Baca, Tomáš Báča, Viktor Walter

1.0.2 (2021-10-03)
------------------
* [shandler]: minor update (will no longer print error if getMsg before callback is called, will just quietly return nullptr!
* fixed an error in transformer: the last message was published with ros::Time::now() time stamp, not with it's real stamp. (`#32 <https://github.com/ctu-mrs/mrs_lib/issues/32>`_)
  oops, forgot to merge it :D
* [shandler]: fixed docs for simple_example
* removed weird stuff from mrs lkf model
* added header guard to transform_broadcaster
* [shandler]: added simple example to better demonstrate basic functionality and documented empty constructor
* updated attitude convertor's exception text
* [ParamLoader]: added links to ROS roslaunch documentation for parameter loading
* [timer]: added more tests, fixes in tests and implementation (`#31 <https://github.com/ctu-mrs/mrs_lib/issues/31>`_)
  hopefully this will fix stuff
* fixed transformer logging
* [cyclic] added comparison and ostream operators
* Enable SubscribeHandler to use either ROSTimer or ThreadTimer (`#30 <https://github.com/ctu-mrs/mrs_lib/issues/30>`_)
  * [shandler + timer]: reimplemented the custom thread-based timer, shandler can now use either ROS Timer or the STL thread-based timer
  * [timer]: added documentation and testing
  * [timer]: made the test a bit stricter
* Contributors: Matej Petrlik, Matouš Vrba, Mykola Morgunenko, Tomas Baca

1.0.1 (2021-05-16)
------------------
  Updating CI - added sanitizers to debug build type, removed PCL-dependent flags (`-march=native` etc)
* [ScopeTimer]: added possibility to throttle output
* fixed important bug in shandler
* fixing some minor buggos, added support for custom-built PCL
* --reuse-master in CI rostest
* ServiceClientHandler: added option for repeat_delay
* fixes to subscribe handler, other minor changes
* [shandler]: fixed minor problem preventing compilation
* [shandler]: deadlock due to timer stop() blocking until all callbacks return should be fixed not
* [paramloader]: added the option to set a parameter name prefix
* [shandler]: added a default constructor (with no parameters) for backward compatibility
* [BatchVisualizer]: remove ros spinning
* service client handler not dependent on mrs_lib anymore
* updated service client docs
* added service client handler test and docs
* ServiceClientHandler: compiles, works
* [shandler]: removed stuff related to time consistency since nobody seems to be using it
* minor refactor of subscribe handler
* reverting to single frequency notch filter
* iir and notch filter done
* Contributors: Daniel Hert, Matouš Vrba, Tomas Baca, Tomáš Báča, stibipet

1.0.0 (2021-03-18)
------------------
* Major release

0.0.6 (2021-03-16)
------------------
* Noetic-compatible
* + Repredictor
* + Custom Timer library
* + Vector converter
* + Attitude converter
* + Subscribe handler
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
