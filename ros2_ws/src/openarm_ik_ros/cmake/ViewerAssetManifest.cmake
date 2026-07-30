# SPDX-License-Identifier: Apache-2.0
function(openarm_configure_viewer_assets description_share stage_urdf output_dir)
  set(license_source
    "${CMAKE_CURRENT_SOURCE_DIR}/licenses/openarm_description-LICENSE.txt")
  file(SIZE "${license_source}" license_size)
  file(SHA256 "${license_source}" license_hash)
  if(NOT license_size EQUAL 11357 OR
     NOT license_hash STREQUAL "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4")
    message(FATAL_ERROR "The pinned openarm_description Apache-2.0 license changed")
  endif()
  set(entries
    "body_link0_symp.stl|assets/robot/openarm_v1.0/mesh/body/collision/body_link0_symp.stl|293284|6c18bbf7e86b03e3faf802e61e8eb438b38dcbcf146d97cffe6e808c65e9a72a|5864"
    "link0_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link0_symp.stl|40284|baf52578e1d9e6225f3818cae82b6074a0b948d3cef8e9a3e6dfafca78507590|804"
    "link1_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link1_symp.stl|17784|066113d13d5cc85098609003bc7ebb73c570015350877f5ed7162ef1b6601852|354"
    "link2_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link2_symp.stl|13384|382ab32e4ae0880e8a1512e7a6ca6ce1f478a6c125db4efa977429ffb1d6b02a|266"
    "link3_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link3_symp.stl|156984|00c908cefab152c00416a570a48bf9aafed1549085f19ff2d882dc3f355d9f59|3138"
    "link4_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link4_symp.stl|1139984|b54883b8c7c96268a68a5879f95998a53ad0b0c4fe74325fad63a6caef669c73|22798"
    "link5_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link5_symp.stl|751484|678a2802906eff7b45a836d2f34a2d8e51def50b6599376968f888e05c72739e|15028"
    "link6_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link6_symp.stl|30084|95529bec23733476dfdbbb266c7db0d25a473a568de73c8337a82440fe4a9ac3|600"
    "link7_symp.stl|assets/robot/openarm_v1.0/mesh/arm/collision/link7_symp.stl|23884|434f207f21f75f5f0bd604e390b8e5bc7b62b619265222846770e06b3f9b5cfb|476"
    "hand.stl|assets/end_effector/parallel_link/meshes/collision/hand.stl|18284|8e5d373ebbd3fd001b506058644062ad71a68f1ced5ca5d5ed0f6de20137956b|364"
    "finger.stl|assets/end_effector/parallel_link/meshes/collision/finger.stl|13284|8e96e1314618cf434908f70df78f68dd2b049c03538964e8d41fc99abe41564d|264")
  file(MAKE_DIRECTORY "${output_dir}/mesh")
  set(manifest "{\n  \"schema\": 1,\n  \"upstream\": {\"repository\": \"https://github.com/enactic/openarm_description\", \"commit\": \"6c7b720f1ba48e8bafa3a3dc752c45f397b42221\", \"license\": \"Apache-2.0\", \"license_file\": \"viewer/openarm_description-LICENSE.txt\", \"license_sha256\": \"${license_hash}\"},\n  \"total_bytes\": 2498724,\n  \"total_triangles\": 49956,\n  \"meshes\": [")
  set(first TRUE)
  foreach(entry IN LISTS entries)
    string(REPLACE "|" ";" fields "${entry}")
    list(GET fields 0 name)
    list(GET fields 1 relative)
    list(GET fields 2 expected_size)
    list(GET fields 3 expected_hash)
    list(GET fields 4 expected_triangles)
    set(source "${description_share}/${relative}")
    if(NOT EXISTS "${source}")
      message(FATAL_ERROR "Pinned viewer asset is missing: ${source}")
    endif()
    file(SIZE "${source}" actual_size)
    file(SHA256 "${source}" actual_hash)
    math(EXPR expected_binary_size "84 + 50 * ${expected_triangles}")
    if(NOT actual_size EQUAL expected_size OR NOT actual_size EQUAL expected_binary_size OR
       NOT actual_hash STREQUAL expected_hash)
      message(FATAL_ERROR "Pinned viewer asset hash, size, or triangle count changed: ${source}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source}")
    configure_file("${source}" "${output_dir}/mesh/${name}" COPYONLY)
    if(first)
      set(first FALSE)
    else()
      string(APPEND manifest ",")
    endif()
    string(APPEND manifest "\n    {\"route\": \"/viewer/mesh/${name}\", \"source\": \"package://openarm_description/${relative}\", \"bytes\": ${expected_size}, \"sha256\": \"${expected_hash}\", \"triangles\": ${expected_triangles}}")
  endforeach()
  string(APPEND manifest "\n  ]\n}\n")
  file(WRITE "${output_dir}/manifest.json" "${manifest}")
  set(OPENARM_VIEWER_MANIFEST "${output_dir}/manifest.json" PARENT_SCOPE)
  set(OPENARM_VIEWER_STAGE_A_URDF "${output_dir}/stage_a.urdf" PARENT_SCOPE)
  set(OPENARM_VIEWER_MESH_DIRECTORY "${output_dir}/mesh" PARENT_SCOPE)
  set(OPENARM_VIEWER_LICENSE "${license_source}" PARENT_SCOPE)
  add_custom_command(
    OUTPUT "${output_dir}/stage_a.urdf"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${stage_urdf}" "${output_dir}/stage_a.urdf"
    DEPENDS "${stage_urdf}"
    VERBATIM)
endfunction()
