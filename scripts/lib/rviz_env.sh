# SPDX-License-Identifier: Apache-2.0
# Shared RViz presentation environment.
#
# Both the engineering launcher and the portal demo run RViz through the same
# XWayland/GLX path on this desktop, so the renderer selection and the HiDPI
# workaround live here rather than being duplicated and drifting apart.

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
    # Hardware GLX presentation flickers during live resize through XWayland on
    # this HiDPI hybrid-GPU laptop.  The small OpenArm scene runs smoothly in
    # llvmpipe and remains stable while resizing.  Keep GPU acceleration for a
    # native X11 session or when explicitly requested.
    if [[ ${XDG_SESSION_TYPE:-} == "wayland" ]]; then
      renderer=software
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
      unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME \
        __VK_LAYER_NV_optimus __GL_SYNC_TO_VBLANK __GL_GSYNC_ALLOWED \
        __GL_VRR_ALLOWED || true
      printf 'OpenArm RViz renderer: integrated GPU (XWayland/GLX)\n'
      ;;
    software)
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
