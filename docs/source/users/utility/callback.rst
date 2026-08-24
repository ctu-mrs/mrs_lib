#################
Callback wrappers
#################

This page contains documentation for CoroCallback and related classes.

The CoroCallback is a wrapper used for passing coroutines as callbacks to other interfaces in mrs_lib.
Since coroutines do not work well with mutually exclusive callback groups,
it also provides policies for handling concurrent calls to the same callback.

.. contents:: Table of contents
   :depth: 2
   :local:

mrs_lib::CoroCallback
=====================

.. doxygenclass:: mrs_lib::CoroCallback< CallbackRetType(CallbackArgs...)>

.. doxygenclass:: mrs_lib::CoroCallback

Policies
========

.. doxygentypedef:: mrs_lib::CoroReentrantPolicyVariant

.. doxygennamespace:: mrs_lib::coro_callback_tags
   :desc-only:

.. note::
   
   All of the policy tags are in the ``mrs_lib::coro_callback_tags`` namespace.

mrs_lib::coro_callback_tags::Reentrant
--------------------------------------

.. doxygenclass:: mrs_lib::coro_callback_tags::Reentrant


mrs_lib::coro_callback_tags::CancelNew
--------------------------------------

.. doxygenclass:: mrs_lib::coro_callback_tags::CancelNew

.. doxygenclass:: mrs_lib::coro_callback_tags::CancelNew< void >

mrs_lib::coro_callback_tags::CancelNewDefault
---------------------------------------------

.. doxygenclass:: mrs_lib::coro_callback_tags::CancelNewDefault


Example
=======

.. literalinclude:: /../../test/utility/callback_test.cpp
   :language: c++
   :linenos:
   :start-after: DOCS: BEGIN EXAMPLE P1
   :end-before: DOCS: END EXAMPLE P1
   :caption: Example of the coro callback usage (p1).
   :dedent:

.. literalinclude:: /../../test/utility/callback_test.cpp
   :language: c++
   :linenos:
   :start-after: DOCS: BEGIN EXAMPLE P2
   :end-before: DOCS: END EXAMPLE P2
   :caption: Example of the coro callback usage (p2).
   :dedent:
