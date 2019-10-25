^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
