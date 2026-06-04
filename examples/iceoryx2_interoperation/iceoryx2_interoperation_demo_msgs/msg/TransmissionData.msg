# Fixed-size POD message for the rmw_iceoryx2 <-> vanilla iceoryx2 cross-language
# example. Mirrors the `TransmissionData` type used by the iceoryx2 cross-language
# examples so the two ecosystems exchange byte-identical samples over shared memory.
#
# Contains only fixed-size fields: no strings or sequences, which would compile to
# structs holding pointers that are invalid across a process boundary.

int32   x
int32   y
float64 funky
