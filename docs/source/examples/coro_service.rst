Using coroutines with service calls
===================================

- Repo Link: https://github.com/ctu-mrs/mrs_lib/tree/ros2/examples/coro_service
- Expected prerequisites:

   - ROS2 Packages
   - ROS2 Nodes
   - ROS2 Services

Coroutines allow suspending functions and resuming them later.
In context of ROS2 programs, this is very useful when calling services.
If you do not use coroutines, the options used are usually either blocking on the future (cannot be done in single threaded context) or manually checking it (requires timers for checking readiness) or specifying callbacks.
Coroutines solve this by writing code, that looks almost like the blocking case, while being functionally similar to writing the rest of the function as a callback.

.. note::
   
   This tutorial covers how to use coroutines specifically with service client in mrs_lib.
   If you want more general information and how they work, see eg. https://cppreference.com/cpp/language/coroutines.


Important parts
---------------

In this section, we will walk through the important parts of the example.
The full source code can be found at the bottom of this page or in the examples folder of mrs_lib repo.

Coroutines
~~~~~~~~~~

Syntactically, coroutines have two major differences normal C++ functions.

1. Firstly, they must contain at least one of the ``co_*`` keywords (``co_await``, ``co_return`` or ``co_yield``).
2. Secondly, the return type must be a coroutine return type.
   For our usecase, it is enough to know that ``mrs_lib::Task<T>`` is such a type.
   The template parameter ``T`` is the type returned from the coroutine when awaited (defaults to void).

An example coroutine is shown in the following listing.
The coroutine returns ``std::optional<std::shared_ptr<ServiceType::Response>>`` which is wrapped in ``mrs_lib::Task``.
It takes parameters just like a normal function.

Instead of the normal ``return`` keyword, we must use ``co_return`` (just coroutine version of the same).
The ``co_await`` keyword in front of the client call tells the language to suspend the current coroutine here and resume once the awaited result is ready.
In this case, it means it sends the service request, suspends the coroutine, and resumes it once the server's response arives.

.. literalinclude:: ../../../examples/coro_service/coro_service.cpp
   :language: c++
   :linenos:
   :lineno-match:
   :start-after: BEGIN CORO
   :end-before: END CORO
   :caption: Example coroutine.
   :dedent:

As you can see, the coroutine function looks almost like a normal function, just with some ``co_*`` keywords sprinkled in.

Next we will have a look at a second coroutine in our program.
This time, it will be a timer callback.

.. literalinclude:: ../../../examples/coro_service/coro_service.cpp
   :language: c++
   :linenos:
   :lineno-match:
   :start-after: BEGIN CORO CALLBACK
   :end-before: END CORO CALLBACK
   :caption: Example coroutine callback.
   :dedent:

Once again, you can see it uses the ``co_await`` keyword.
This is required whenever you are calling another coroutine, even if it returns void.

You may also notice that there is no co_return this time.
Similarly to normal functions, if the coroutine returns void, you can omit the ``co_return`` at the end.
However, you still have to have at least one ``co_*`` keyword in it to make it a coroutine.

.. warning::

   The coroutines implemented in mrs_lib are so called *lazy coroutines*.
   This means they do nothing until you await them (eg. ``co_await my_coro()``).
   As a result, if you forget to await a coroutine, it will not run!

Coroutines as callbacks
~~~~~~~~~~~~~~~~~~~~~~~

You can use coroutines as callbacks, but there are currently some additional restrictions:

1. The object which calling the callbacks must support calling coroutines.
   (See API reference for the individual callback sources if they have overloads for coroutine callbacks.)
2. The rclcpp::CallbackGroup used must be reentrant [#why_reentrant]_.

In the following listing, you can see the constructor of the example node.
Firstly, we initialize the parent node, a local copy of the logger, the required reentrant callback group and the service client.
Next we initialize the timer with our coroutine callback.
For timer, we use a constructor overload taking ``mrs_lib::TimerHandlerOptions`` (contains node and callback group), rate and the callback.
The coroutine callback is passed as two parts -- the method pointer and a pointer to the instance with which it is invoked.
Finally, we also initialize another timer to see the executor is not blocked during the service call.


.. literalinclude:: ../../../examples/coro_service/coro_service.cpp
   :language: c++
   :linenos:
   :lineno-match:
   :start-after: BEGIN CTOR
   :end-before: END CTOR
   :caption: Constructor of the example node.
   :dedent:


How to run
----------

The example is automatically built when building mrs_lib with tests enabled.
Alternatively, you can copy both the example and ``CMakeLists.txt`` file from the repo to your package.

To run the example, you have to start both the server and the client.
(Don't forget to source your workspace.)

.. code-block:: console
   :caption: Running server.

   $ ros2 run mrs_lib example_coro_service_server
   [INFO] [...]: Created service: /my_service
   [INFO] [...]: Received service call. Starting work...
   [INFO] [...]: Work done. Sending response...
   [INFO] [...]: Received service call. Starting work...
   [INFO] [...]: Work done. Sending response...

.. code-block:: console
   :caption: Running client.
   
   $ ros2 run mrs_lib example_coro_service 
   [INFO] [...]: Created client 'my_service' -> '/my_service'
   [INFO] [...]: Chattering ... (0)
   [INFO] [...]: Chattering ... (1)
   [INFO] [...]: Chattering ... (2)
   [INFO] [...]: Chattering ... (3)
   [INFO] [...]: Calling service...
   [INFO] [...]: Chattering ... (4)
   [INFO] [...]: Chattering ... (5)
   [INFO] [...]: Service response:
   success: 'true'
   message: 'Response to service with value: 'true''
   [INFO] [...]: Chattering ... (6)
   [INFO] [...]: Chattering ... (7)
   [INFO] [...]: Chattering ... (8)
   [INFO] [...]: Calling service...
   [INFO] [...]: Chattering ... (9)
   [INFO] [...]: Chattering ... (10)
   [INFO] [...]: Service response:
   success: 'true'
   message: 'Response to service with value: 'true''
   [INFO] [...]: Chattering ... (11)

From the output of the client, we can see that the chattering timer is running even during the service call.
During the processing of the service call, the coroutine is suspended and thus it can handle the other callbacks, such as the chatter.
When the service server finishes around a second later, the coroutine is resumed and it prints the response.


Full Source code
----------------

This is the full source code for the example.
You can also find it in the examples directory of mrs_lib repo.

.. literalinclude:: ../../../examples/coro_service/coro_service.cpp
   :language: c++
   :linenos:
   :lineno-match:
   :caption: Full code for the example.


.. rubric:: Footnotes

.. [#why_reentrant]

   The reentrant callback group requirement is a caused by limitations when working with rclcpp callback groups.
   When the coroutine gets suspended, the calling function exits and thus the callback looks finished to rclcpp, thus unlocking the callback group.
   Since it is not possible to keep the group locked, and another callback may start, reentrant callback groups requirement is there to expose this to the user.
