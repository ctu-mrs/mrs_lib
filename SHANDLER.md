# Desired upgrades:

1) Rewrite so that `create_handler` doesn't have to return a pointer
2) Get rid of SubscribeHandler template
3) Simplify the implementation

# Reasons why these are not achievable without loss of functionality:

1) Rewrite so that `create_handler` doesn't have to return a pointer
  - I don't know how to keep `new_data` flag correctly updated when message callback is used - how to tell whether the user used the data or not without him calling `get_data` (for which the user needs to have access to the SubscribeHandler object, for which he needs a pointer to it).
  - If only the received message would be passed to the message callback, information about e.g. from which topic it came would be lost.
  - Possible solution - some kind of a meta-object with topic info, which would be held in the SubscribeHandler and passed to the callback? (is this really a solution? - again a pointer)
  - [Update]: Alternative solution - just reset the flag to false when the `message_callback` is called (provide the possibility to use a message callback with no parameters, which wouldn't reset the flag? provide an alternative including the name of the topic?).

2) Get rid of SubscribeHandler template
  - This would require runtime dynamic casting, which I'd rather avoid. Otherwise I don't know how to implement this and it might be impossible.
  - [Update]: I've improved the template situation at least a bit using the generic function `construct_object` to construct new shandlers (such as in the case of Nodelets when the handler is a member variable). This removes the need for specifying the template parameter twice.

3) Simplify the implementation
  - I'll think about this, but so far I don't see a way to do this without loss of functionality or ease of use for the user.
  - [Update]: Got rid of the `SubscribeMgr` class, `SubscribeHandler` is now constructed using a constructor or the helper `construct_object` function.
