cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED INPUT_URDF OR NOT DEFINED OUTPUT_URDF)
  message(FATAL_ERROR "INPUT_URDF and OUTPUT_URDF are required")
endif()

set(expected_source_sha256
  "dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55")
file(SHA256 "${INPUT_URDF}" source_sha256)
if(NOT source_sha256 STREQUAL expected_source_sha256)
  message(FATAL_ERROR
    "Refusing to derive the Stage-A visualization URDF from unexpected input: "
    "expected ${expected_source_sha256}, got ${source_sha256}")
endif()

file(READ "${INPUT_URDF}" urdf)

function(replace_exactly variable old new expected_count label)
  set(probe "${${variable}}")
  set(count 0)
  while(TRUE)
    string(FIND "${probe}" "${old}" offset)
    if(offset EQUAL -1)
      break()
    endif()
    math(EXPR count "${count} + 1")
    string(LENGTH "${old}" old_length)
    math(EXPR remainder_start "${offset} + ${old_length}")
    string(SUBSTRING "${probe}" ${remainder_start} -1 probe)
  endwhile()
  if(NOT count EQUAL expected_count)
    message(FATAL_ERROR
      "Expected ${expected_count} exact ${label} block(s), found ${count}")
  endif()
  string(REPLACE "${old}" "${new}" replaced "${${variable}}")
  set(${variable} "${replaced}" PARENT_SCOPE)
endfunction()

set(positive_finger_inertial [=[    <inertial>
      <origin rpy="0 0 0" xyz="0.0064528 0.01702 0.0219685"/>
      <mass value="0.03602545343277134"/>
      <inertia ixx="2.3749999999999997e-06" ixy="1e-06" ixz="1e-06" iyy="2.3749999999999997e-06" iyz="1e-06" izz="7.5e-07"/>
    </inertial>
]=])
set(negative_finger_inertial [=[    <inertial>
      <origin rpy="0 0 0" xyz="0.0064528 -0.01702 0.0219685"/>
      <mass value="0.03602545343277134"/>
      <inertia ixx="2.3749999999999997e-06" ixy="1e-06" ixz="1e-06" iyy="2.3749999999999997e-06" iyz="1e-06" izz="7.5e-07"/>
    </inertial>
]=])
replace_exactly(urdf "${positive_finger_inertial}" "" 2
  "positive-y finger inertial")
replace_exactly(urdf "${negative_finger_inertial}" "" 2
  "negative-y finger inertial")

foreach(side IN ITEMS left right)
  set(source_joint [=[  <joint name="openarm_@SIDE@_finger_joint1" type="prismatic">
    <parent link="openarm_@SIDE@_link7"/>
    <child link="openarm_@SIDE@_right_finger"/>
    <origin rpy="0 0 0" xyz="0.0 -0.005 0.1025"/>
    <axis xyz="0 -1 0"/>
    <limit effort="333" lower="0.0" upper="0.044" velocity="10.0"/>
  </joint>]=])
  set(fixed_source_joint [=[  <joint name="openarm_@SIDE@_finger_joint1" type="fixed">
    <parent link="openarm_@SIDE@_link7"/>
    <child link="openarm_@SIDE@_right_finger"/>
    <origin rpy="0 0 0" xyz="0.0 -0.005 0.1025"/>
  </joint>]=])
  set(mimic_joint [=[  <joint name="openarm_@SIDE@_finger_joint2" type="prismatic">
    <parent link="openarm_@SIDE@_link7"/>
    <child link="openarm_@SIDE@_left_finger"/>
    <origin rpy="0 0 0" xyz="0.0 0.005 0.1025"/>
    <axis xyz="0 1 0"/>
    <limit effort="333" lower="0.0" upper="0.044" velocity="10.0"/>
    <mimic joint="openarm_@SIDE@_finger_joint1"/>
  </joint>]=])
  set(fixed_mimic_joint [=[  <joint name="openarm_@SIDE@_finger_joint2" type="fixed">
    <parent link="openarm_@SIDE@_link7"/>
    <child link="openarm_@SIDE@_left_finger"/>
    <origin rpy="0 0 0" xyz="0.0 0.005 0.1025"/>
  </joint>]=])
  foreach(variable IN ITEMS source_joint fixed_source_joint mimic_joint fixed_mimic_joint)
    string(REPLACE "@SIDE@" "${side}" ${variable} "${${variable}}")
  endforeach()
  replace_exactly(urdf "${source_joint}" "${fixed_source_joint}" 1
    "${side} source finger joint")
  replace_exactly(urdf "${mimic_joint}" "${fixed_mimic_joint}" 1
    "${side} mimic finger joint")
endforeach()

set(robot_open "<robot name=\"openarm\">\n")
set(visualization_notice [=[<robot name="openarm">
  <!-- Stage-A visualization/TF convention only. Derived from canonical SHA-256
       dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55.
       Motor-8 finger state is not measured: all four finger joints are fixed at
       their canonical q=0 m closed transforms. Invalid finger inertials are
       omitted. Do not use this description for dynamics or collision safety. -->
]=])
replace_exactly(urdf "${robot_open}" "${visualization_notice}" 1 "robot opening")

get_filename_component(output_directory "${OUTPUT_URDF}" DIRECTORY)
file(MAKE_DIRECTORY "${output_directory}")
file(WRITE "${OUTPUT_URDF}" "${urdf}")
