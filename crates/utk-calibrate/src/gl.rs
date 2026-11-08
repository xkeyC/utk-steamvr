//! Headless OpenGL via EGL (pbuffer), rendering egui into a texture that is handed to an OpenVR
//! overlay. No window is ever created.

use std::sync::Arc;

use anyhow::{Context as _, Result, anyhow};
use egui_glow::glow::{self, HasContext};
use khronos_egl as egl;

pub struct Gl {
    egl: egl::Instance<egl::Static>,
    display: egl::Display,
    surface: egl::Surface,
    context: egl::Context,
    pub glow: Arc<glow::Context>,
}

impl Gl {
    pub fn new() -> Result<Self> {
        let egl = egl::Instance::new(egl::Static);
        // EGL_DEFAULT_DISPLAY guesses from the window system and can land on Mesa's llvmpipe on an
        // NVIDIA desktop; SteamVR cannot use a software-rendered texture. Pick the device directly.
        let display = match device_display(&egl) {
            Some(d) => d,
            None => unsafe { egl.get_display(egl::DEFAULT_DISPLAY) }
                .ok_or_else(|| anyhow!("no EGL display"))?,
        };
        egl.initialize(display).context("eglInitialize")?;
        egl.bind_api(egl::OPENGL_API)
            .context("eglBindAPI(OpenGL)")?;
        let attrs = [
            egl::SURFACE_TYPE,
            egl::PBUFFER_BIT,
            egl::RENDERABLE_TYPE,
            egl::OPENGL_BIT,
            egl::RED_SIZE,
            8,
            egl::GREEN_SIZE,
            8,
            egl::BLUE_SIZE,
            8,
            egl::ALPHA_SIZE,
            8,
            egl::NONE,
        ];
        let config = egl
            .choose_first_config(display, &attrs)
            .context("eglChooseConfig")?
            .ok_or_else(|| anyhow!("no suitable EGL config"))?;
        let surface = egl
            .create_pbuffer_surface(display, config, &[egl::WIDTH, 1, egl::HEIGHT, 1, egl::NONE])
            .context("eglCreatePbufferSurface")?;
        let ctx_attrs = [
            egl::CONTEXT_MAJOR_VERSION,
            3,
            egl::CONTEXT_MINOR_VERSION,
            3,
            egl::CONTEXT_OPENGL_PROFILE_MASK,
            egl::CONTEXT_OPENGL_CORE_PROFILE_BIT,
            egl::NONE,
        ];
        let context = egl
            .create_context(display, config, None, &ctx_attrs)
            .context("eglCreateContext (OpenGL 3.3 core)")?;
        egl.make_current(display, Some(surface), Some(surface), Some(context))
            .context("eglMakeCurrent")?;
        let glow = unsafe {
            glow::Context::from_loader_function(|name| {
                egl.get_proc_address(name)
                    .map_or(std::ptr::null(), |p| p as *const _)
            })
        };
        Ok(Self {
            egl,
            display,
            surface,
            context,
            glow: Arc::new(glow),
        })
    }
}

/// EGL_EXT_device_enumeration + EGL_EXT_platform_device: prefer the NVIDIA device (it exposes
/// EGL_NV_device_cuda), otherwise the first device.
fn device_display(egl: &egl::Instance<egl::Static>) -> Option<egl::Display> {
    use std::ffi::{CStr, c_char, c_void};
    type QueryDevices = unsafe extern "system" fn(i32, *mut *mut c_void, *mut i32) -> u32;
    type QueryDeviceString = unsafe extern "system" fn(*mut c_void, i32) -> *const c_char;
    type GetPlatformDisplay =
        unsafe extern "system" fn(u32, *mut c_void, *const i32) -> *mut c_void;
    const PLATFORM_DEVICE_EXT: u32 = 0x313F;
    const EXTENSIONS: i32 = 0x3055;
    const NONE: i32 = 0x3038;

    let query_devices: QueryDevices =
        unsafe { std::mem::transmute(egl.get_proc_address("eglQueryDevicesEXT")?) };
    let query_string: QueryDeviceString =
        unsafe { std::mem::transmute(egl.get_proc_address("eglQueryDeviceStringEXT")?) };
    let get_display: GetPlatformDisplay =
        unsafe { std::mem::transmute(egl.get_proc_address("eglGetPlatformDisplayEXT")?) };
    let mut devices = [std::ptr::null_mut(); 16];
    let mut n = 0;
    if unsafe { query_devices(devices.len() as i32, devices.as_mut_ptr(), &mut n) } == 0 || n <= 0 {
        return None;
    }
    let devices = &devices[..n as usize];
    let is_nvidia = |d: *mut c_void| {
        let ext = unsafe { query_string(d, EXTENSIONS) };
        !ext.is_null()
            && unsafe { CStr::from_ptr(ext) }
                .to_string_lossy()
                .contains("EGL_NV_device_cuda")
    };
    let device = devices
        .iter()
        .copied()
        .find(|&d| is_nvidia(d))
        .unwrap_or(devices[0]);
    let display = unsafe { get_display(PLATFORM_DEVICE_EXT, device, [NONE].as_ptr()) };
    (!display.is_null()).then(|| unsafe { egl::Display::from_ptr(display) })
}

impl Drop for Gl {
    fn drop(&mut self) {
        let _ = self.egl.make_current(self.display, None, None, None);
        let _ = self.egl.destroy_context(self.display, self.context);
        let _ = self.egl.destroy_surface(self.display, self.surface);
        let _ = self.egl.terminate(self.display);
    }
}

/// An RGBA8 texture with a framebuffer to render egui into.
pub struct Target {
    pub texture: glow::Texture,
    framebuffer: glow::Framebuffer,
    pub width: u32,
    pub height: u32,
}

impl Target {
    pub fn new(gl: &glow::Context, width: u32, height: u32) -> Result<Self> {
        unsafe {
            let texture = gl.create_texture().map_err(|e| anyhow!(e))?;
            gl.bind_texture(glow::TEXTURE_2D, Some(texture));
            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA8 as i32,
                width as i32,
                height as i32,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelUnpackData::Slice(None),
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            let framebuffer = gl.create_framebuffer().map_err(|e| anyhow!(e))?;
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(framebuffer));
            gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER,
                glow::COLOR_ATTACHMENT0,
                glow::TEXTURE_2D,
                Some(texture),
                0,
            );
            let status = gl.check_framebuffer_status(glow::FRAMEBUFFER);
            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
            if status != glow::FRAMEBUFFER_COMPLETE {
                return Err(anyhow!("framebuffer incomplete: {status:#x}"));
            }
            Ok(Self {
                texture,
                framebuffer,
                width,
                height,
            })
        }
    }

    pub fn bind(&self, gl: &glow::Context) {
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.framebuffer));
            gl.viewport(0, 0, self.width as i32, self.height as i32);
            gl.clear_color(0.0, 0.0, 0.0, 0.0);
            gl.clear(glow::COLOR_BUFFER_BIT);
        }
    }

    pub fn finish(&self, gl: &glow::Context) {
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
            gl.finish();
        }
    }

    /// Reads the framebuffer back as RGBA rows, bottom row first. Call between bind and finish.
    pub fn read_rgba(&self, gl: &glow::Context) -> Vec<u8> {
        let mut buf = vec![0u8; (self.width * self.height * 4) as usize];
        unsafe {
            gl.read_pixels(
                0,
                0,
                self.width as i32,
                self.height as i32,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                glow::PixelPackData::Slice(Some(&mut buf)),
            );
        }
        buf
    }

    /// The raw GL name SteamVR expects in Texture_t.
    pub fn gl_name(&self) -> u32 {
        self.texture.0.get()
    }
}
