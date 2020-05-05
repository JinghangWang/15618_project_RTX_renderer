#include "pathtracer.h"

/*** Optix ***/
#include <glad/glad.h>  // Needs to be included before gl_interop

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>

#include <sampleConfig.h>

#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Camera.h>
#include <sutil/Exception.h>
#include <sutil/GLDisplay.h>
#include <sutil/Matrix.h>
#include <sutil/Trackball.h>
#include <sutil/sutil.h>
#include <sutil/vec_math.h>
#include <optix_stack_size.h>

#include <GLFW/glfw3.h>

#include "optixPathTracer.h"
#include "CMU462/vector3D.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"


#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <assert.h>

using namespace CMU462::StaticScene;

bool resize_dirty = false;

// Camera state
bool             camera_changed = true;
sutil::Camera    camera;
sutil::Trackball trackball;

// Mouse state
int32_t mouse_button = -1;

int32_t samples_per_launch = 16;

//------------------------------------------------------------------------------
//
// Local types
// TODO: some of these should move to sutil or optix util header
//
//------------------------------------------------------------------------------

template <typename T>
struct Record
{
    __align__( OPTIX_SBT_RECORD_ALIGNMENT ) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef Record<RayGenData>   RayGenRecord;
typedef Record<MissData>     MissRecord;
typedef Record<HitGroupData> HitGroupRecord;


struct VertexPosition
{
    float x, y, z, pad;
};


struct IndexedTriangle
{
    uint32_t v1, v2, v3, pad;
};


struct Instance
{
    float transform[12];
};


struct PathTracerState
{
    OptixDeviceContext context = 0;

    OptixTraversableHandle         gas_handle               = 0;  // Traversable handle for triangle AS
    CUdeviceptr                    d_gas_output_buffer      = 0;  // Triangle AS memory
    CUdeviceptr                    d_vertices               = 0;

    OptixModule                    ptx_module               = 0;
    OptixPipelineCompileOptions    pipeline_compile_options = {};
    OptixPipeline                  pipeline                 = 0;

    OptixProgramGroup              raygen_prog_group        = 0;
    OptixProgramGroup              radiance_miss_group      = 0;
    OptixProgramGroup              occlusion_miss_group     = 0;
    OptixProgramGroup              radiance_hit_group       = 0;
    OptixProgramGroup              occlusion_hit_group      = 0;

    CUstream                       stream                   = 0;
    Params                         params;
    Params*                        d_params;

    OptixShaderBindingTable        sbt                      = {};
};


//------------------------------------------------------------------------------
//
// Scene data
//
//------------------------------------------------------------------------------

// const int32_t TRIANGLE_COUNT = 32;
int32_t MAT_COUNT      = 0;

std::vector<VertexPosition> g_vertices;
std::vector<uint32_t> g_mat_indices;
std::vector<float3> g_emission_colors;
std::vector<float3> g_diffuse_colors;
/*
static std::array<VertexPosition, TRIANGLE_COUNT* 3> g_vertices =
{  {
    // Floor  -- white lambert
    {    0.0f,    0.0f,    0.0f, 0.0f },
    {    0.0f,    0.0f,  559.2f, 0.0f },
    {  556.0f,    0.0f,  559.2f, 0.0f },
    {    0.0f,    0.0f,    0.0f, 0.0f },
    {  556.0f,    0.0f,  559.2f, 0.0f },
    {  556.0f,    0.0f,    0.0f, 0.0f },

    // Ceiling -- white lambert
    {    0.0f,  548.8f,    0.0f, 0.0f },
    {  556.0f,  548.8f,    0.0f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },

    {    0.0f,  548.8f,    0.0f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },
    {    0.0f,  548.8f,  559.2f, 0.0f },

    // Back wall -- white lambert
    {    0.0f,    0.0f,  559.2f, 0.0f },
    {    0.0f,  548.8f,  559.2f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },

    {    0.0f,    0.0f,  559.2f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },
    {  556.0f,    0.0f,  559.2f, 0.0f },

    // Right wall -- green lambert
    {    0.0f,    0.0f,    0.0f, 0.0f },
    {    0.0f,  548.8f,    0.0f, 0.0f },
    {    0.0f,  548.8f,  559.2f, 0.0f },

    {    0.0f,    0.0f,    0.0f, 0.0f },
    {    0.0f,  548.8f,  559.2f, 0.0f },
    {    0.0f,    0.0f,  559.2f, 0.0f },

    // Left wall -- red lambert
    {  556.0f,    0.0f,    0.0f, 0.0f },
    {  556.0f,    0.0f,  559.2f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },

    {  556.0f,    0.0f,    0.0f, 0.0f },
    {  556.0f,  548.8f,  559.2f, 0.0f },
    {  556.0f,  548.8f,    0.0f, 0.0f },

    // Short block -- white lambert
    {  130.0f,  165.0f,   65.0f, 0.0f },
    {   82.0f,  165.0f,  225.0f, 0.0f },
    {  242.0f,  165.0f,  274.0f, 0.0f },

    {  130.0f,  165.0f,   65.0f, 0.0f },
    {  242.0f,  165.0f,  274.0f, 0.0f },
    {  290.0f,  165.0f,  114.0f, 0.0f },

    {  290.0f,    0.0f,  114.0f, 0.0f },
    {  290.0f,  165.0f,  114.0f, 0.0f },
    {  240.0f,  165.0f,  272.0f, 0.0f },

    {  290.0f,    0.0f,  114.0f, 0.0f },
    {  240.0f,  165.0f,  272.0f, 0.0f },
    {  240.0f,    0.0f,  272.0f, 0.0f },

    {  130.0f,    0.0f,   65.0f, 0.0f },
    {  130.0f,  165.0f,   65.0f, 0.0f },
    {  290.0f,  165.0f,  114.0f, 0.0f },

    {  130.0f,    0.0f,   65.0f, 0.0f },
    {  290.0f,  165.0f,  114.0f, 0.0f },
    {  290.0f,    0.0f,  114.0f, 0.0f },

    {   82.0f,    0.0f,  225.0f, 0.0f },
    {   82.0f,  165.0f,  225.0f, 0.0f },
    {  130.0f,  165.0f,   65.0f, 0.0f },

    {   82.0f,    0.0f,  225.0f, 0.0f },
    {  130.0f,  165.0f,   65.0f, 0.0f },
    {  130.0f,    0.0f,   65.0f, 0.0f },

    {  240.0f,    0.0f,  272.0f, 0.0f },
    {  240.0f,  165.0f,  272.0f, 0.0f },
    {   82.0f,  165.0f,  225.0f, 0.0f },

    {  240.0f,    0.0f,  272.0f, 0.0f },
    {   82.0f,  165.0f,  225.0f, 0.0f },
    {   82.0f,    0.0f,  225.0f, 0.0f },

    // Tall block -- white lambert
    {  423.0f,  330.0f,  247.0f, 0.0f },
    {  265.0f,  330.0f,  296.0f, 0.0f },
    {  314.0f,  330.0f,  455.0f, 0.0f },

    {  423.0f,  330.0f,  247.0f, 0.0f },
    {  314.0f,  330.0f,  455.0f, 0.0f },
    {  472.0f,  330.0f,  406.0f, 0.0f },

    {  423.0f,    0.0f,  247.0f, 0.0f },
    {  423.0f,  330.0f,  247.0f, 0.0f },
    {  472.0f,  330.0f,  406.0f, 0.0f },

    {  423.0f,    0.0f,  247.0f, 0.0f },
    {  472.0f,  330.0f,  406.0f, 0.0f },
    {  472.0f,    0.0f,  406.0f, 0.0f },

    {  472.0f,    0.0f,  406.0f, 0.0f },
    {  472.0f,  330.0f,  406.0f, 0.0f },
    {  314.0f,  330.0f,  456.0f, 0.0f },

    {  472.0f,    0.0f,  406.0f, 0.0f },
    {  314.0f,  330.0f,  456.0f, 0.0f },
    {  314.0f,    0.0f,  456.0f, 0.0f },

    {  314.0f,    0.0f,  456.0f, 0.0f },
    {  314.0f,  330.0f,  456.0f, 0.0f },
    {  265.0f,  330.0f,  296.0f, 0.0f },

    {  314.0f,    0.0f,  456.0f, 0.0f },
    {  265.0f,  330.0f,  296.0f, 0.0f },
    {  265.0f,    0.0f,  296.0f, 0.0f },

    {  265.0f,    0.0f,  296.0f, 0.0f },
    {  265.0f,  330.0f,  296.0f, 0.0f },
    {  423.0f,  330.0f,  247.0f, 0.0f },

    {  265.0f,    0.0f,  296.0f, 0.0f },
    {  423.0f,  330.0f,  247.0f, 0.0f },
    {  423.0f,    0.0f,  247.0f, 0.0f },

    // Ceiling light -- emmissive
    {  343.0f,  548.6f,  227.0f, 0.0f },
    {  213.0f,  548.6f,  227.0f, 0.0f },
    {  213.0f,  548.6f,  332.0f, 0.0f },

    {  343.0f,  548.6f,  227.0f, 0.0f },
    {  213.0f,  548.6f,  332.0f, 0.0f },
    {  343.0f,  548.6f,  332.0f, 0.0f }
} };

static std::array<uint32_t, TRIANGLE_COUNT> g_mat_indices = {{
    0, 0,                          // Floor         -- white lambert
    0, 0,                          // Ceiling       -- white lambert
    0, 0,                          // Back wall     -- white lambert
    1, 1,                          // Right wall    -- green lambert
    2, 2,                          // Left wall     -- red lambert
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Short block   -- white lambert
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // Tall block    -- white lambert
    3, 3                           // Ceiling light -- emmissive
}};



const std::array<float3, MAT_COUNT> g_emission_colors =
{ {
    {  0.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f },
    {  0.0f,  0.0f,  0.0f },
    { 15.0f, 15.0f,  5.0f }

} };


const std::array<float3, MAT_COUNT> g_diffuse_colors =
{ {
    { 0.80f, 0.80f, 0.80f },
    { 0.05f, 0.80f, 0.05f },
    { 0.80f, 0.05f, 0.05f },
    { 0.50f, 0.00f, 0.00f }
} };
*/

//------------------------------------------------------------------------------
//
// GLFW callbacks
//
//------------------------------------------------------------------------------

static void mouseButtonCallback( GLFWwindow* window, int button, int action, int mods )
{
    double xpos, ypos;
    glfwGetCursorPos( window, &xpos, &ypos );

    if( action == GLFW_PRESS )
    {
        mouse_button = button;
        trackball.startTracking( static_cast<int>( xpos ), static_cast<int>( ypos ) );
    }
    else
    {
        mouse_button = -1;
    }
}


static void cursorPosCallback( GLFWwindow* window, double xpos, double ypos )
{
    Params* params = static_cast<Params*>( glfwGetWindowUserPointer( window ) );

    if( mouse_button == GLFW_MOUSE_BUTTON_LEFT )
    {
        trackball.setViewMode( sutil::Trackball::LookAtFixed );
        trackball.updateTracking( static_cast<int>( xpos ), static_cast<int>( ypos ), params->width, params->height );
        camera_changed = true;
    }
    else if( mouse_button == GLFW_MOUSE_BUTTON_RIGHT )
    {
        trackball.setViewMode( sutil::Trackball::EyeFixed );
        trackball.updateTracking( static_cast<int>( xpos ), static_cast<int>( ypos ), params->width, params->height );
        camera_changed = true;
    }
}


static void windowSizeCallback( GLFWwindow* window, int32_t res_x, int32_t res_y )
{
    Params* params = static_cast<Params*>( glfwGetWindowUserPointer( window ) );
    params->width  = res_x;
    params->height = res_y;
    camera_changed = true;
    resize_dirty   = true;
}


static void keyCallback( GLFWwindow* window, int32_t key, int32_t /*scancode*/, int32_t action, int32_t /*mods*/ )
{
    if( action == GLFW_PRESS )
    {
        if( key == GLFW_KEY_Q || key == GLFW_KEY_ESCAPE )
        {
            glfwSetWindowShouldClose( window, true );
        }
    }
    else if( key == GLFW_KEY_G )
    {
        // toggle UI draw
    }
}


static void scrollCallback( GLFWwindow* window, double xscroll, double yscroll )
{
    if( trackball.wheelEvent( (int)yscroll ) )
        camera_changed = true;
}


//------------------------------------------------------------------------------
//
// Helper functions
// TODO: some of these should move to sutil or optix util header
//
//------------------------------------------------------------------------------

void printUsageAndExit( const char* argv0 )
{
    std::cerr << "Usage  : " << argv0 << " [options]\n";
    std::cerr << "Options: --file | -f <filename>      File for image output\n";
    std::cerr << "         --launch-samples | -s       Number of samples per pixel per launch (default 16)\n";
    std::cerr << "         --no-gl-interop             Disable GL interop for display\n";
    std::cerr << "         --help | -h                 Print this usage message\n";
    exit( 0 );
}


void initLaunchParams( PathTracerState& state )
{
    CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &state.params.accum_buffer ),
                state.params.width * state.params.height * sizeof( float4 )
                ) );
    state.params.frame_buffer = nullptr;  // Will be set when output buffer is mapped

    state.params.samples_per_launch = samples_per_launch;
    state.params.subframe_index     = 0u;

    state.params.light.emission = make_float3( 15.0f, 15.0f, 5.0f );
    state.params.light.corner   = make_float3( 343.0f, 548.5f, 227.0f );
    state.params.light.v1       = make_float3( 0.0f, 0.0f, 105.0f );
    state.params.light.v2       = make_float3( -130.0f, 0.0f, 0.0f );
    state.params.light.normal   = normalize( cross( state.params.light.v1, state.params.light.v2 ) );
    state.params.handle         = state.gas_handle;

    CUDA_CHECK( cudaStreamCreate( &state.stream ) );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &state.d_params ), sizeof( Params ) ) );

}


void handleCameraUpdate( Params& params )
{
    if( !camera_changed )
        return;
    camera_changed = false;

    camera.setAspectRatio( static_cast<float>( params.width ) / static_cast<float>( params.height ) );
    params.eye = camera.eye();
    camera.UVWFrame( params.U, params.V, params.W );
}


void handleResize( sutil::CUDAOutputBuffer<uchar4>& output_buffer, Params& params )
{
    if( !resize_dirty )
        return;
    resize_dirty = false;

    output_buffer.resize( params.width, params.height );

    // Realloc accumulation buffer
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( params.accum_buffer ) ) );
    CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &params.accum_buffer ),
                params.width * params.height * sizeof( float4 )
                ) );
}


void updateState( sutil::CUDAOutputBuffer<uchar4>& output_buffer, Params& params )
{
    // Update params on device
    if( camera_changed || resize_dirty )
        params.subframe_index = 0;

    handleCameraUpdate( params );
    handleResize( output_buffer, params );
}


void launchSubframe( sutil::CUDAOutputBuffer<uchar4>& output_buffer, PathTracerState& state )
{
    // Launch
    uchar4* result_buffer_data = output_buffer.map();
    state.params.frame_buffer  = result_buffer_data;
    CUDA_CHECK( cudaMemcpyAsync(
                reinterpret_cast<void*>( state.d_params ),
                &state.params, sizeof( Params ),
                cudaMemcpyHostToDevice, state.stream
                ) );

    OPTIX_CHECK( optixLaunch(
                state.pipeline,
                state.stream,
                reinterpret_cast<CUdeviceptr>( state.d_params ),
                sizeof( Params ),
                &state.sbt,
                state.params.width,   // launch width
                state.params.height,  // launch height
                1                     // launch depth
                ) );
    output_buffer.unmap();
    CUDA_SYNC_CHECK();
}


void displaySubframe( sutil::CUDAOutputBuffer<uchar4>& output_buffer, sutil::GLDisplay& gl_display, GLFWwindow* window )
{
    // Display
    int framebuf_res_x = 0;  // The display's resolution (could be HDPI res)
    int framebuf_res_y = 0;  //
    glfwGetFramebufferSize( window, &framebuf_res_x, &framebuf_res_y );
    gl_display.display(
            output_buffer.width(),
            output_buffer.height(),
            framebuf_res_x,
            framebuf_res_y,
            output_buffer.getPBO()
            );
}


static void context_log_cb( unsigned int level, const char* tag, const char* message, void* /*cbdata */ )
{
    std::cerr << "[" << std::setw( 2 ) << level << "][" << std::setw( 12 ) << tag << "]: " << message << "\n";
}


void initCameraState(CMU462::Camera* cmu_camera)
{
    // camera.setEye( make_float3( 0, 0, 0 ) );
    // camera.setLookat( make_float3( 0, 0, -100 ) );
    // camera.setUp( make_float3( 0.0f, 1.0f, 0.0f ) );
    // camera.setFovY( 28 );
    // camera_changed = true;

    // trackball.setCamera( &camera );
    // trackball.setMoveSpeed( 10.0f );
    // trackball.setReferenceFrame(
    //         make_float3( 1.0f, 0.0f, 0.0f ),
    //         make_float3( 0.0f, 0.0f, 1.0f ),
    //         make_float3( 0.0f, 1.0f, 0.0f )
    //         );
    // trackball.setGimbalLock( true );

    CMU462::Vector3D eye = cmu_camera->position();
    float3 eye_pos = make_float3(eye.x, eye.y, eye.z);
    CMU462::Vector3D target = cmu_camera->view_point();
    float3 target_pos = make_float3(target.x, target.y, target.z);

    camera.setEye( eye_pos );
    camera.setLookat( target_pos );
    camera.setUp( make_float3( 0.0f, 1.0f, 0.0f ) );
    camera.setFovY( 28 );
    camera_changed = true;

    trackball.setCamera( &camera );
    trackball.setMoveSpeed( 10.0f );
    trackball.setReferenceFrame(
            make_float3( 1.0f, 0.0f, 0.0f ),
            make_float3( 0.0f, 0.0f, 1.0f ),
            make_float3( 0.0f, 1.0f, 0.0f )
            );
    trackball.setGimbalLock( true );
}


void createContext( PathTracerState& state )
{
    // Initialize CUDA
    CUDA_CHECK( cudaFree( 0 ) );

    OptixDeviceContext context;
    CUcontext          cu_ctx = 0;  // zero means take the current context
    OPTIX_CHECK( optixInit() );
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction       = &context_log_cb;
    options.logCallbackLevel          = 4;
    OPTIX_CHECK( optixDeviceContextCreate( cu_ctx, &options, &context ) );

    state.context = context;
}


void buildMeshAccel( PathTracerState& state )
{
    //
    // copy mesh data to device
    //
    const size_t vertices_size_in_bytes = g_vertices.size() * sizeof( VertexPosition );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &state.d_vertices ), vertices_size_in_bytes ) );
    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( state.d_vertices ),
                g_vertices.data(), vertices_size_in_bytes,
                cudaMemcpyHostToDevice
                ) );

    CUdeviceptr  d_mat_indices             = 0;
    const size_t mat_indices_size_in_bytes = g_mat_indices.size() * sizeof( uint32_t );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_mat_indices ), mat_indices_size_in_bytes ) );
    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( d_mat_indices ),
                g_mat_indices.data(),
                mat_indices_size_in_bytes,
                cudaMemcpyHostToDevice
                ) );

    //
    // Build triangle GAS
    //
    uint32_t triangle_input_flags[MAT_COUNT] =  // One per SBT record for this build input
    {
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT,
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT
    };

    OptixBuildInput triangle_input                           = {};
    triangle_input.type                                      = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    triangle_input.triangleArray.vertexFormat                = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangle_input.triangleArray.vertexStrideInBytes         = sizeof( VertexPosition );
    triangle_input.triangleArray.numVertices                 = static_cast<uint32_t>( g_vertices.size() );
    triangle_input.triangleArray.vertexBuffers               = &state.d_vertices;
    triangle_input.triangleArray.flags                       = triangle_input_flags;
    triangle_input.triangleArray.numSbtRecords               = MAT_COUNT;
    triangle_input.triangleArray.sbtIndexOffsetBuffer        = d_mat_indices;
    triangle_input.triangleArray.sbtIndexOffsetSizeInBytes   = sizeof( uint32_t );
    triangle_input.triangleArray.sbtIndexOffsetStrideInBytes = sizeof( uint32_t );

    OptixAccelBuildOptions accel_options = {};
    accel_options.buildFlags             = OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
    accel_options.operation              = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes gas_buffer_sizes;
    OPTIX_CHECK( optixAccelComputeMemoryUsage(
                state.context,
                &accel_options,
                &triangle_input,
                1,  // num_build_inputs
                &gas_buffer_sizes
                ) );

    CUdeviceptr d_temp_buffer;
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_temp_buffer ), gas_buffer_sizes.tempSizeInBytes ) );

    // non-compacted output
    CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;
    size_t      compactedSizeOffset = roundUp<size_t>( gas_buffer_sizes.outputSizeInBytes, 8ull );
    CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &d_buffer_temp_output_gas_and_compacted_size ),
                compactedSizeOffset + 8
                ) );

    OptixAccelEmitDesc emitProperty = {};
    emitProperty.type               = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitProperty.result             = ( CUdeviceptr )( (char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset );

    OPTIX_CHECK( optixAccelBuild(
                state.context,
                0,                                  // CUDA stream
                &accel_options,
                &triangle_input,
                1,                                  // num build inputs
                d_temp_buffer,
                gas_buffer_sizes.tempSizeInBytes,
                d_buffer_temp_output_gas_and_compacted_size,
                gas_buffer_sizes.outputSizeInBytes,
                &state.gas_handle,
                &emitProperty,                      // emitted property list
                1                                   // num emitted properties
                ) );

    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_temp_buffer ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_mat_indices ) ) );

    size_t compacted_gas_size;
    CUDA_CHECK( cudaMemcpy( &compacted_gas_size, (void*)emitProperty.result, sizeof(size_t), cudaMemcpyDeviceToHost ) );

    if( compacted_gas_size < gas_buffer_sizes.outputSizeInBytes )
    {
        CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &state.d_gas_output_buffer ), compacted_gas_size ) );

        // use handle as input and output
        OPTIX_CHECK( optixAccelCompact( state.context, 0, state.gas_handle, state.d_gas_output_buffer, compacted_gas_size, &state.gas_handle ) );

        CUDA_CHECK( cudaFree( (void*)d_buffer_temp_output_gas_and_compacted_size ) );
    }
    else
    {
        state.d_gas_output_buffer = d_buffer_temp_output_gas_and_compacted_size;
    }
}


void createModule( PathTracerState& state )
{
    OptixModuleCompileOptions module_compile_options = {};
    module_compile_options.maxRegisterCount  = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
    module_compile_options.optLevel          = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    module_compile_options.debugLevel        = OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO;

    state.pipeline_compile_options.usesMotionBlur        = false;
    state.pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY;
    // state.pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    state.pipeline_compile_options.numPayloadValues      = 2;
    state.pipeline_compile_options.numAttributeValues    = 2;
    state.pipeline_compile_options.exceptionFlags        =
        OPTIX_EXCEPTION_FLAG_NONE;
        //OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
        //OPTIX_EXCEPTION_FLAG_DEBUG;
    state.pipeline_compile_options.pipelineLaunchParamsVariableName = "params";

    const std::string ptx = sutil::getPtxString( OPTIX_SAMPLE_NAME, "optixPathTracer.cu" );

    char   log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK_LOG( optixModuleCreateFromPTX(
                state.context,
                &module_compile_options,
                &state.pipeline_compile_options,
                ptx.c_str(),
                ptx.size(),
                log,
                &sizeof_log,
                &state.ptx_module
                ) );
}


void createProgramGroups( PathTracerState& state )
{
    OptixProgramGroupOptions  program_group_options = {};

    char   log[2048];
    size_t sizeof_log = sizeof( log );

    {
        OptixProgramGroupDesc raygen_prog_group_desc    = {};
        raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
        raygen_prog_group_desc.raygen.module            = state.ptx_module;
        raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__rg";

        OPTIX_CHECK_LOG( optixProgramGroupCreate(
                    state.context, &raygen_prog_group_desc,
                    1,  // num program groups
                    &program_group_options,
                    log,
                    &sizeof_log,
                    &state.raygen_prog_group
                    ) );
    }

    {
        OptixProgramGroupDesc miss_prog_group_desc  = {};
        miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
        miss_prog_group_desc.miss.module            = state.ptx_module;
        miss_prog_group_desc.miss.entryFunctionName = "__miss__radiance";
        sizeof_log                                  = sizeof( log );
        OPTIX_CHECK_LOG( optixProgramGroupCreate(
                    state.context, &miss_prog_group_desc,
                    1,  // num program groups
                    &program_group_options,
                    log, &sizeof_log,
                    &state.radiance_miss_group
                    ) );

        memset( &miss_prog_group_desc, 0, sizeof( OptixProgramGroupDesc ) );
        miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
        miss_prog_group_desc.miss.module            = nullptr;  // NULL miss program for occlusion rays
        miss_prog_group_desc.miss.entryFunctionName = nullptr;
        sizeof_log                                  = sizeof( log );
        OPTIX_CHECK_LOG( optixProgramGroupCreate(
                    state.context, &miss_prog_group_desc,
                    1,  // num program groups
                    &program_group_options,
                    log,
                    &sizeof_log,
                    &state.occlusion_miss_group
                    ) );
    }

    {
        OptixProgramGroupDesc hit_prog_group_desc        = {};
        hit_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
        hit_prog_group_desc.hitgroup.moduleCH            = state.ptx_module;
        hit_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__radiance";
        sizeof_log                                       = sizeof( log );
        OPTIX_CHECK_LOG( optixProgramGroupCreate(
                    state.context,
                    &hit_prog_group_desc,
                    1,  // num program groups
                    &program_group_options,
                    log,
                    &sizeof_log,
                    &state.radiance_hit_group
                    ) );

        memset( &hit_prog_group_desc, 0, sizeof( OptixProgramGroupDesc ) );
        hit_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
        hit_prog_group_desc.hitgroup.moduleCH            = state.ptx_module;
        hit_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__occlusion";
        sizeof_log                                       = sizeof( log );
        OPTIX_CHECK( optixProgramGroupCreate(
                    state.context,
                    &hit_prog_group_desc,
                    1,  // num program groups
                    &program_group_options,
                    log,
                    &sizeof_log,
                    &state.occlusion_hit_group
                    ) );
    }
}


void createPipeline( PathTracerState& state )
{
    OptixProgramGroup program_groups[] =
    {
        state.raygen_prog_group,
        state.radiance_miss_group,
        state.occlusion_miss_group,
        state.radiance_hit_group,
        state.occlusion_hit_group
    };

    OptixPipelineLinkOptions pipeline_link_options = {};
    pipeline_link_options.maxTraceDepth            = 3;
    pipeline_link_options.debugLevel               = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    pipeline_link_options.overrideUsesMotionBlur   = false;

    char   log[2048];
    size_t sizeof_log = sizeof( log );
    OPTIX_CHECK_LOG( optixPipelineCreate(
                state.context,
                &state.pipeline_compile_options,
                &pipeline_link_options,
                program_groups,
                sizeof( program_groups ) / sizeof( program_groups[0] ),
                log,
                &sizeof_log,
                &state.pipeline
                ) );

    // We need to specify the max traversal depth.  Calculate the stack sizes, so we can specify all
    // parameters to optixPipelineSetStackSize.
    OptixStackSizes stack_sizes = {};
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.raygen_prog_group,    &stack_sizes ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.radiance_miss_group,  &stack_sizes ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.occlusion_miss_group, &stack_sizes ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.radiance_hit_group,   &stack_sizes ) );
    OPTIX_CHECK( optixUtilAccumulateStackSizes( state.occlusion_hit_group,  &stack_sizes ) );

    uint32_t max_trace_depth = 1;
    uint32_t max_cc_depth = 0;
    uint32_t max_dc_depth = 0;
    uint32_t direct_callable_stack_size_from_traversal;
    uint32_t direct_callable_stack_size_from_state;
    uint32_t continuation_stack_size;
    OPTIX_CHECK( optixUtilComputeStackSizes(
                &stack_sizes,
                max_trace_depth,
                max_cc_depth,
                max_dc_depth,
                &direct_callable_stack_size_from_traversal,
                &direct_callable_stack_size_from_state,
                &continuation_stack_size
                ) );

    // This is 3 since the largest depth is IAS->IAS->GAS
    const uint32_t max_traversal_depth = 3;

    OPTIX_CHECK( optixPipelineSetStackSize(
                state.pipeline,
                direct_callable_stack_size_from_traversal,
                direct_callable_stack_size_from_state,
                continuation_stack_size,
                max_traversal_depth
                ) );
}


void createSBT( PathTracerState& state )
{
    CUdeviceptr  d_raygen_record;
    const size_t raygen_record_size = sizeof( RayGenRecord );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_raygen_record ), raygen_record_size ) );

    RayGenRecord rg_sbt = {};
    OPTIX_CHECK( optixSbtRecordPackHeader( state.raygen_prog_group, &rg_sbt ) );

    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( d_raygen_record ),
                &rg_sbt,
                raygen_record_size,
                cudaMemcpyHostToDevice
                ) );


    CUdeviceptr  d_miss_records;
    const size_t miss_record_size = sizeof( MissRecord );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &d_miss_records ), miss_record_size * RAY_TYPE_COUNT ) );

    MissRecord ms_sbt[2];
    OPTIX_CHECK( optixSbtRecordPackHeader( state.radiance_miss_group,  &ms_sbt[0] ) );
    ms_sbt[0].data.bg_color = make_float4( 0.0f );
    OPTIX_CHECK( optixSbtRecordPackHeader( state.occlusion_miss_group, &ms_sbt[1] ) );
    ms_sbt[1].data.bg_color = make_float4( 0.0f );

    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( d_miss_records ),
                ms_sbt,
                miss_record_size*RAY_TYPE_COUNT,
                cudaMemcpyHostToDevice
                ) );

    CUdeviceptr  d_hitgroup_records;
    const size_t hitgroup_record_size = sizeof( HitGroupRecord );
    CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &d_hitgroup_records ),
                hitgroup_record_size * RAY_TYPE_COUNT * MAT_COUNT
                ) );

    HitGroupRecord hitgroup_records[RAY_TYPE_COUNT * MAT_COUNT];
    for( int i = 0; i < MAT_COUNT; ++i )
    {
        {
            const int sbt_idx = i * RAY_TYPE_COUNT + 0;  // SBT for radiance ray-type for ith material

            OPTIX_CHECK( optixSbtRecordPackHeader( state.radiance_hit_group, &hitgroup_records[sbt_idx] ) );
            hitgroup_records[sbt_idx].data.emission_color = g_emission_colors[i];
            hitgroup_records[sbt_idx].data.diffuse_color  = g_diffuse_colors[i];
            hitgroup_records[sbt_idx].data.vertices       = reinterpret_cast<float4*>( state.d_vertices );
        }

        {
            const int sbt_idx = i * RAY_TYPE_COUNT + 1;  // SBT for occlusion ray-type for ith material
            memset( &hitgroup_records[sbt_idx], 0, hitgroup_record_size );

            OPTIX_CHECK( optixSbtRecordPackHeader( state.occlusion_hit_group, &hitgroup_records[sbt_idx] ) );
        }
    }

    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( d_hitgroup_records ),
                hitgroup_records,
                hitgroup_record_size*RAY_TYPE_COUNT*MAT_COUNT,
                cudaMemcpyHostToDevice
                ) );

    state.sbt.raygenRecord                = d_raygen_record;
    state.sbt.missRecordBase              = d_miss_records;
    state.sbt.missRecordStrideInBytes     = static_cast<uint32_t>( miss_record_size );
    state.sbt.missRecordCount             = RAY_TYPE_COUNT;
    state.sbt.hitgroupRecordBase          = d_hitgroup_records;
    state.sbt.hitgroupRecordStrideInBytes = static_cast<uint32_t>( hitgroup_record_size );
    state.sbt.hitgroupRecordCount         = RAY_TYPE_COUNT * MAT_COUNT;
}


void cleanupState( PathTracerState& state )
{
    OPTIX_CHECK( optixPipelineDestroy( state.pipeline ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.raygen_prog_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.radiance_miss_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.radiance_hit_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.occlusion_hit_group ) );
    OPTIX_CHECK( optixProgramGroupDestroy( state.occlusion_miss_group ) );
    OPTIX_CHECK( optixModuleDestroy( state.ptx_module ) );
    OPTIX_CHECK( optixDeviceContextDestroy( state.context ) );


    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.raygenRecord ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.missRecordBase ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.sbt.hitgroupRecordBase ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.d_vertices ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.d_gas_output_buffer ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.params.accum_buffer ) ) );
    CUDA_CHECK( cudaFree( reinterpret_cast<void*>( state.d_params ) ) );
}

namespace CMU462 {

void updateBuffer(const sutil::ImageBuffer& buffer, ImageBuffer& frame_buffer) {
  auto width = frame_buffer.w;
  auto height = frame_buffer.h;
  memcpy(&frame_buffer.data[0], (unsigned char*)buffer.data, width * height * 4);
}

std::vector<VertexPosition> padPositions(const std::vector<Vector3D> positions) {
  std::vector<VertexPosition> padded_positions(3);

  for (auto p: positions) {
    VertexPosition pp;
    pp.x = p.x;
    pp.y = p.y;
    pp.z = p.z;
    pp.pad = 0.0f;
    padded_positions.insert(padded_positions.end(), pp);
  }
  return padded_positions;
}

std::vector<VertexPosition> getAllVertexPositions(const std::vector<Primitive *> &obj_prims) {
  std::vector<VertexPosition> vertices;
  vertices.reserve(obj_prims.size() * 3);

  for (Primitive* primitive : obj_prims) {
    Triangle* triangle = static_cast<Triangle*>(primitive);
    auto positions = triangle->getVertexPositions();
    auto padded_positions = padPositions(positions);
    vertices.insert(vertices.end(), padded_positions.begin(), padded_positions.end());
  }
  return vertices;
}

void PathTracer::collect_primitives() {
  for (SceneObject *obj : scene->objects) {
    const std::vector<Primitive *> &obj_prims = obj->get_primitives();
    auto num_primitives = obj_prims.size();
    g_vertices.reserve(g_vertices.size() +  num_primitives * 3);

    // collect vertex positions
    const std::vector<VertexPosition> vertices = getAllVertexPositions(obj_prims);
    g_vertices.insert(g_vertices.end(), vertices.begin(), vertices.end());

    // assign material index for all inserted triangles
    uint32_t current_index = g_emission_colors.size();
    std::vector<uint32_t> indices(num_primitives * 3, current_index);
    g_mat_indices.reserve(g_vertices.size() + num_primitives * 3);
    g_mat_indices.insert(g_mat_indices.end(), indices.begin(), indices.end());

    // insert emission and diffuse colors
    DiffuseBSDF* bsdf = static_cast<DiffuseBSDF*>(obj->get_bsdf());
    Spectrum diffuse_spec = bsdf->f();
    float3 diffuse = make_float3(diffuse_spec.r, diffuse_spec.g, diffuse_spec.b);
    g_diffuse_colors.push_back(diffuse);

    Spectrum emission_spec = bsdf->get_emission();
    float3 emission = make_float3(emission_spec.r, emission_spec.g, emission_spec.b);
    printf("emission: %f, %f, %f\n", emission.x, emission.y, emission.z);
    printf("diffuse: %f, %f, %f\n", diffuse.x, diffuse.y, diffuse.z);
    g_emission_colors.push_back(emission);
  }

  MAT_COUNT = g_emission_colors.size();
  printf("%f, %f, %f\n", g_vertices[10].x, g_vertices[10].y, g_vertices[10].z);
}

void PathTracer::do_raytracing_GPU() {
  fprintf(stdout, "[PathTracer] Rendering... \n");
  fflush(stdout);
  Timer timer;
  timer.start();


  PathTracerState PTstate;
  PTstate.params.width                             = frameBuffer.w;
  PTstate.params.height                            = frameBuffer.h;
  sutil::CUDAOutputBufferType output_buffer_type = sutil::CUDAOutputBufferType::ZERO_COPY;


  initCameraState(this->camera);

  //
  // Set up OptiX state
  //
  createContext( PTstate );

  this->collect_primitives();
  
  buildMeshAccel( PTstate );
  createModule( PTstate );
  createProgramGroups( PTstate );
  createPipeline( PTstate );
  createSBT( PTstate );
  initLaunchParams( PTstate );

  //
  // Render body
  //
  sutil::CUDAOutputBuffer<uchar4> output_buffer(
          output_buffer_type,
          PTstate.params.width,
          PTstate.params.height
          );

  handleCameraUpdate( PTstate.params );
  handleResize( output_buffer, PTstate.params );
  /*** launch rendering ***/
  launchSubframe( output_buffer, PTstate );

  sutil::ImageBuffer buffer;
  buffer.data         = output_buffer.getHostPointer();
  buffer.width        = output_buffer.width();
  buffer.height       = output_buffer.height();
  buffer.pixel_format = sutil::BufferImageFormat::UNSIGNED_BYTE4;

  //
  // Output to frameBuffer
  //
  assert(buffer.pixel_format == sutil::BufferImageFormat::UNSIGNED_BYTE4);
  sutil::displayBufferFile("bufferfile.txt", buffer, false );
  updateBuffer(buffer, this->frameBuffer);

  timer.stop();
  fprintf(stdout, "Done! (%.4fs)\n", timer.duration());
  this->state = DONE;

  cleanupState( PTstate );
}

}  // namespace CMU462
