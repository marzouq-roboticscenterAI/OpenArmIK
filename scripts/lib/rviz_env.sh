# SPDX-License-Identifier: Apache-2.0
# Shared RViz presentation environment.
#
# Both the engineering launcher and the portal demo run RViz through the same
# XWayland/GLX path on this desktop, so the renderer selection and the HiDPI
# workaround live here rather than being duplicated and drifting apart.

# Removes the Snap runtime leakage a Snap-packaged terminal (VS Code) injects
# into its children. Those variables point Qt/GTK plugin discovery at
# /snap/core20, whose Ubuntu 20.04 glibc then loses to this host's:
#   rviz2: symbol lookup error: /snap/core20/.../libpthread.so.0:
#          undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
#
# Call this BEFORE sourcing any ROS setup.bash: it clears XDG_DATA_DIRS, which
# ROS then repopulates. Calling it afterwards would discard ROS's own entries.
openarm_sanitize_snap_environment() {
  local variable
  for variable in SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA \
    SNAP_INSTANCE_KEY SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REEXEC \
    SNAP_REVISION SNAP_USER_COMMON SNAP_USER_DATA GTK_PATH GTK_EXE_PREFIX \
    GTK_IM_MODULE_FILE GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE \
    GIO_MODULE_DIR QT_PLUGIN_PATH QT_QPA_PLATFORMTHEME XDG_DATA_DIRS; do
    unset "$variable" || true
  done
}

# Applies the Qt platform and GPU environment RViz needs on this host.
# Honours OPENARM_RVIZ_RENDERER=auto|nvidia|integrated|software.
# Returns 2 for an unrecognised renderer.
openarm_configure_rviz_environment() {
  # RViz/Ogre on this Wayland desktop requires XWayland/GLX.  Its Ogre render
  # window can flicker after a HiDPI resize when Qt uses devicePixelRatio=2, so
  # keep scaling off for this process only.  XWayland still reports 192 DPI and
  # preserves readable fonts without the unstable double-scaled render target.
  export QT_QPA_PLATFORM=xcb
  export QT_XCB_GL_INTEGRATION=xcb_glx
  export QT_ENABLE_HIGHDPI_SCALING=0
  export QT_SCREEN_SCALE_FACTORS=1
  unset QT_SCALE_FACTOR QT_AUTO_SCREEN_SCALE_FACTOR || true

  local renderer=${OPENARM_RVIZ_RENDERER:-auto}
  if [[ "$renderer" == "auto" ]]; then
    # Software GLX avoids resize flicker through XWayland, but the detailed
    # OpenArm meshes saturated roughly nine CPU cores while live joint states
    # were changing and froze the desktop. Integrated GLX rendered the same
    # live scene at about one third of one core. Prefer the responsive hardware
    # path; software remains available as an explicit troubleshooting choice.
    if [[ ${XDG_SESSION_TYPE:-} == "wayland" ]]; then
      renderer=integrated
    elif [[ -e /dev/nvidia0 ]] && command -v nvidia-smi >/dev/null 2>&1 && \
        nvidia-smi >/dev/null 2>&1; then
      renderer=nvidia
    else
      renderer=integrated
    fi
  fi

  case "$renderer" in
    nvidia)
      unset LIBGL_ALWAYS_SOFTWARE || true
      unset DRI_PRIME || true
      export __NV_PRIME_RENDER_OFFLOAD=1
      export __GLX_VENDOR_LIBRARY_NAME=nvidia
      export __VK_LAYER_NV_optimus=NVIDIA_only
      export __GL_SYNC_TO_VBLANK=1
      export __GL_GSYNC_ALLOWED=0
      export __GL_VRR_ALLOWED=0
      printf 'OpenArm RViz renderer: NVIDIA PRIME offload (XWayland/GLX)\n'
      ;;
    integrated)
      unset LIBGL_ALWAYS_SOFTWARE || true
      unset DRI_PRIME || true
      unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME \
        __VK_LAYER_NV_optimus __GL_SYNC_TO_VBLANK __GL_GSYNC_ALLOWED \
        __GL_VRR_ALLOWED || true
      printf 'OpenArm RViz renderer: integrated GPU (XWayland/GLX)\n'
      ;;
    software)
      unset DRI_PRIME || true
      unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME \
        __VK_LAYER_NV_optimus __GL_SYNC_TO_VBLANK __GL_GSYNC_ALLOWED \
        __GL_VRR_ALLOWED || true
      export LIBGL_ALWAYS_SOFTWARE=1
      printf 'OpenArm RViz renderer: Mesa software rasterizer (XWayland/GLX)\n'
      ;;
    *)
      printf 'OPENARM_RVIZ_RENDERER must be auto, nvidia, integrated, or software\n' >&2
      return 2
      ;;
  esac
  return 0
}
