# Fixed-size POD payload for the latency benchmark.
#
# The sequence number is carried in the payload (not only in the rmw message
# info) so the subscriber can detect loss and the final sample with any rmw
# implementation - message-info sequence numbers are optional in the rmw spec.

uint64 sequence
