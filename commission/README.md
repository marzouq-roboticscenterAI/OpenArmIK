# OpenArm commissioning sessions

`openarm_commission` is a standalone C++17 library exposed through the versioned
ISO-C header `openarm_commission.h`. It consumes caller-supplied DaMiao
output-shaft encoder samples in radians. It never opens a CAN interface, emits a
motor frame, changes an interface, enables a drive, saves motor zero, or writes
flash.

The manual session collects fresh torque-disabled encoder samples at one or two
fixture-defined joint angles. A known sign permits one reference; otherwise two
distinct references establish the sign. The resulting software-only mapping is
`q_model = a*q_output + b`, where `a` is exactly `-1` or `+1`. Velocity and
output-torque mappings are consequently `dq_model = a*dq_output` and
`tau_model = tau_output/a`; no gearbox ratio is applied a second time.

The recipe session is a supervised, caller-driven hard-stop state machine. Its
outputs are bounded next-action requests, not hardware commands. It enforces
fresh encoder feedback, explicit E-stop/deadman state, prior travel, low velocity
plus torque contact evidence, time/travel/speed/torque/temperature/contact-energy
ceilings, retreat/reapproach repeatability, review, and software-only commit.
Every fault or caller abort latches `OA_RECIPE_ABORT` and makes commit impossible.

Arm-joint recipes are rejected unless the caller provides an explicit nonzero
hardware-qualification revision and record. A gripper recipe may be marked
simulation-only, but it still passes the same sample, interlock, ceiling,
repeatability, review, and commit gates. Simulation never qualifies hardware.

Build and test independently from the repository root:

```sh
cmake -S commission -B commission/build -DCMAKE_BUILD_TYPE=Debug
cmake --build commission/build --parallel
ctest --test-dir commission/build --output-on-failure
```

AddressSanitizer and UndefinedBehaviorSanitizer are enabled by default with GCC
and Clang. The test suite includes a strict C11 ABI consumer, manual calibration,
the complete recipe flow, fault injection, commit exclusion after failure, and
abort from every nonterminal recipe state.
