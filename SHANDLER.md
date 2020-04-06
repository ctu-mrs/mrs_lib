# Desired upgrades:

1) Rewrite so that `create_handler` doesn't have to return a pointer
2) Get rid of SubscribeHandler template
3) Simplify the implementation

# Reasons why these are not achievable without loss of functionality:

1) Rewrite so that `create_handler` doesn't have to return a pointer
  - I don't know how to keep `new_data` flag correctly updated when message callback is used - how to tell whether the user used the data or not without him calling `get_data` (for which the user needs to have access to the SubscribeHandler object, for which he needs a pointer to it).
  - If only the received message would be passed to the message callback, information about e.g. from which topic it came would be lost.
  - Possible solution - some kind of a meta-object with topic info, which would be held in the SubscribeHandler and passed to the callback? (is this really a solution? - again a pointer)
2) This would require runtime dynamic casting, which I'd rather avoid. Otherwise I don't know how to implement this and it might be impossible.
3) I'll think about this, but so far I don't see a way to do this without loss of functionality or ease of use for the user.
