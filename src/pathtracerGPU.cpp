#include "pathtracer.h"

/*** Optix ***/
// #include <glad/glad.h>  // Needs to be included before gl_interop

// #include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>

// #include <sampleConfig.h>

#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Camera.h>
#include <sutil/Exception.h>
#include <sutil/GLDisplay.h>
#include <sutil/Matrix.h>
#include <sutil/Trackball.h>
#include <sutil/sutil.h>
#include <sutil/vec_math.h>
#include <optix_stack_size.h>

// #include <GLFW/glfw3.h>

#include "optixPathTracer.h"

#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace CMU462 {

void PathTracer::do_raytracing_GPU() {
  PathTracerState state;
  state.params.width                             = 1024;
  state.params.height                            = 1024;
  sutil::CUDAOutputBufferType output_buffer_type = sutil::CUDAOutputBufferType::GL_INTEROP;

  initCameraState();

  //
  // Set up OptiX state
  //
  createContext( state );
  buildMeshAccel( state );
  createModule( state );
  createProgramGroups( state );
  createPipeline( state );
  createSBT( state );
  initLaunchParams( state );

  //
  // Render body
  //
  sutil::CUDAOutputBuffer<uchar4> output_buffer(
          output_buffer_type,
          state.params.width,
          state.params.height
          );

  /*** launch rendering ***/
  launchSubframe( output_buffer, state );

  sutil::ImageBuffer buffer;
  buffer.data         = output_buffer.getHostPointer();
  buffer.width        = output_buffer.width();
  buffer.height       = output_buffer.height();
  buffer.pixel_format = sutil::BufferImageFormat::UNSIGNED_BYTE4;

  // TODO
  // sutil::displayBufferFile( outfile.c_str(), buffer, false );
}

}  // namespace CMU462
