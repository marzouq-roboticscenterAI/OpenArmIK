# OpenArm CAN diagnostics provenance

This independent C11 implementation uses documented DaMiao MIT framing facts and
OpenArm protocol evidence from `enactic/openarm_can` commit
`c32ecd31da267967f0c913c2118c843177d88b91`, licensed Apache-2.0.  It does not
copy upstream source code.  No vendor artwork or firmware is included.

The expected-ID probe and supplied in-memory transport are diagnostics-only.  The
separate codec can construct MIT and enable frames, but no physical transport is
included.  The module contains no frame for zero-position flash, register writes,
CAN-ID/bitrate/mode changes, firmware flash, or automatic motor enablement.
