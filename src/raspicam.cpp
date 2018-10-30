
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>

#define VCOS_ALWAYS_WANT_LOGGING
#define VERSION_STRING "v1.2"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "std_srvs/srv/empty.hpp"

#include "RaspiCamControl.h"
#include "mmal_cxx_helper.h"

using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
using ImageMsg = sensor_msgs::msg::Image;
using EmptySrv = std_srvs::srv::Empty;


static constexpr int IMG_BUFFER_SIZE = 10 * 1024 * 1024;  // 10 MB

// Video format information
static constexpr int VIDEO_FRAME_RATE_DEN = 3;

// Video render needs at least 2 buffers.
static constexpr int VIDEO_OUTPUT_BUFFERS_NUM = 3;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE {
  RASPIVID_STATE()
    : camera_component(nullptr)
    , splitter_component(nullptr)
    , encoder_component(nullptr)
    , splitter_connection(nullptr)
    , encoder_connection(nullptr)
    , splitter_pool(nullptr, mmal::default_delete_pool)
    , encoder_pool(nullptr, mmal::default_delete_pool){};

  bool isInit;
  int width;      /// Requested width of image
  int height;     /// requested height of image
  int framerate;  /// Requested frame rate (fps)
  int quality;
  bool enable_raw_pub; // Enable Raw publishing

  int camera_id = 0;

  RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters

  mmal::component_ptr camera_component;
  mmal::component_ptr splitter_component;
  mmal::component_ptr encoder_component;

  mmal::connection_ptr splitter_connection;  /// Pointer to camera => preview
  mmal::connection_ptr encoder_connection;  /// Pointer to camera => encoder

  mmal::pool_ptr splitter_pool;   // Pointer buffer pool used by splitter (raw) output
  mmal::pool_ptr encoder_pool;  // Pointer buffer pool used by encoder (jpg) output
};

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct MMAL_PORT_USERDATA_T {
  MMAL_PORT_USERDATA_T(const RASPIVID_STATE& state) : pstate(state){};
  std::unique_ptr<uint8_t[]> buffer[2];  // Memory to write buffer data to.
  const RASPIVID_STATE& pstate;          // pointer to our state for use by callback
  bool abort;                            // Set to 1 in callback if an error occurs to attempt to abort
                                         // the capture
  int frame;
  int id;

  int frames_skipped = 0;
} PORT_USERDATA;



rclcpp::Node::SharedPtr g_node;

rclcpp::Publisher<CameraInfoMsg>::SharedPtr camera_info_pub;
CameraInfoMsg::SharedPtr c_info = std::make_shared<CameraInfoMsg>();
rclcpp::Publisher<ImageMsg>::SharedPtr image_pub;
ImageMsg::SharedPtr image_msg = std::make_shared<ImageMsg>();
rclcpp::Publisher<CompressedImageMsg>::SharedPtr compressed_image_pub;
CompressedImageMsg::SharedPtr compressed_image_msg = std::make_shared<CompressedImageMsg>();

std::string camera_frame_id;
int skip_frames = 0;



static void configure_parameters(RASPIVID_STATE& state) {

  state.width = 640;
  state.height = 480;
  state.quality = 80;
  state.framerate = 30;

  //state.camera_frame_id = "";
  state.enable_raw_pub = false;
  state.camera_id = 0;


  // Set up the camera_parameters to default
  raspicamcontrol_set_defaults(state.camera_parameters);

  bool temp = false;
  state.camera_parameters.hflip = temp;  // Hack for bool param => int variable
  state.camera_parameters.vflip = temp;  // Hack for bool param => int variable
  state.camera_parameters.shutter_speed = 0;
  
  state.isInit = false;

}


/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {
  // We pass our own memory and other stuff in via the userdata field.
  PORT_USERDATA* pData = port->userdata;

  if (pData && pData->pstate.isInit) {
    size_t bytes_written = buffer->length;
    if (buffer->length) {
      if (pData->id != INT_MAX) {
        if (pData->id + buffer->length > IMG_BUFFER_SIZE) {
          RCLCPP_ERROR(g_node->get_logger(), "pData->id (%d) + buffer->length (%d) > "
                    "IMG_BUFFER_SIZE (%d), skipping the frame",
                    pData->id, buffer->length, IMG_BUFFER_SIZE);
          pData->id = INT_MAX;  // mark this frame corrupted
        } else {
          mmal_buffer_header_mem_lock(buffer);
          memcpy(&(pData->buffer[pData->frame & 1].get()[pData->id]), buffer->data, buffer->length);
          pData->id += bytes_written;
          mmal_buffer_header_mem_unlock(buffer);
        }
      }
    }

    if (bytes_written != buffer->length) {
      vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      RCLCPP_ERROR(g_node->get_logger(),"Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      pData->abort = true;
    }

    bool complete = false;
    if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
      complete = true;

    if (complete) {
      if (pData->id != INT_MAX) {
        // RCLCPP_INFO(g_node->get_logger(),"Frame size %d", pData->id);
        if (skip_frames > 0 && pData->frames_skipped < skip_frames) {
          pData->frames_skipped++;
        } else {
          pData->frames_skipped = 0;

          //compressed_image_msg->header.seq = pData->frame;
          compressed_image_msg->header.frame_id = camera_frame_id;
          compressed_image_msg->header.stamp = g_node->now();
          compressed_image_msg->format = "jpg";

          auto start = pData->buffer[pData->frame & 1].get();
          auto end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
          compressed_image_msg->data.resize(pData->id);
          std::copy(start, end, compressed_image_msg->data.begin());
          compressed_image_pub->publish(compressed_image_msg);

          //c_info->header.seq = pData->frame;
          c_info->header.stamp = compressed_image_msg->header.stamp;
          c_info->header.frame_id = compressed_image_msg->header.frame_id;
          camera_info_pub->publish(c_info);
          pData->frame++;

        }
      }
      pData->id = 0;
    }
  }
  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status;

    MMAL_BUFFER_HEADER_T* new_buffer = mmal_queue_get(pData->pstate.encoder_pool->queue);

    if (new_buffer)
      status = mmal_port_send_buffer(port, new_buffer);

    if (!new_buffer || status != MMAL_SUCCESS) {
      vcos_log_error("Unable to return a buffer to the encoder port");
      RCLCPP_ERROR(g_node->get_logger(),"Unable to return a buffer to the encoder port");
    }
  }

}


static void splitter_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {
  // We pass our file handle and other stuff in via the userdata field.
  PORT_USERDATA* pData = port->userdata;
        RCLCPP_INFO(g_node->get_logger(),"-----> CALLBACK");

  if (pData && pData->pstate.isInit) {
    size_t bytes_written = buffer->length;
    if (buffer->length) {
      if (pData->id != INT_MAX) {
        if (pData->id + buffer->length > IMG_BUFFER_SIZE) {
          RCLCPP_ERROR(g_node->get_logger(), "pData->id (%d) + buffer->length (%d) > "
                    "IMG_BUFFER_SIZE (%d), skipping the frame",
                    pData->id, buffer->length, IMG_BUFFER_SIZE);
          pData->id = INT_MAX;  // mark this frame corrupted
        } else {
          mmal_buffer_header_mem_lock(buffer);
          memcpy(&(pData->buffer[pData->frame & 1].get()[pData->id]), buffer->data, buffer->length);
          pData->id += bytes_written;
          mmal_buffer_header_mem_unlock(buffer);
        }
      }
    }

    if (bytes_written != buffer->length) {
      vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      RCLCPP_ERROR(g_node->get_logger(),"Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      pData->abort = true;
    }

    int complete = false;
    if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
      complete = true;

    if (complete) {
      if (pData->id != INT_MAX) {
        // RCLCPP_INFO(g_node->get_logger(),"Frame size %d", pData->id);
        if (skip_frames > 0 && pData->frames_skipped < skip_frames) {
          pData->frames_skipped++;
        } else {
          pData->frames_skipped = 0;
          //image_msg->header.seq = pData->frame;
          image_msg->header.frame_id = camera_frame_id;
          image_msg->header.stamp = g_node->now();
          image_msg->encoding = "bgr8";
          image_msg->is_bigendian = false;
          image_msg->height = pData->pstate.height;
          image_msg->width = pData->pstate.width;
          image_msg->step = (pData->pstate.width * 3);
          auto start = pData->buffer[pData->frame & 1].get();
          auto end = &(pData->buffer[pData->frame & 1].get()[pData->id]);
          image_msg->data.resize(pData->id);
          std::copy(start, end, image_msg->data.begin());
          image_pub->publish(image_msg);
        }
      }
      pData->frame++;
      pData->id = 0;
    }

          RCLCPP_INFO(g_node->get_logger(),"----->DONE CALLBACK");

  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled) {
    MMAL_STATUS_T status;

    MMAL_BUFFER_HEADER_T* new_buffer = mmal_queue_get(pData->pstate.splitter_pool->queue);

    if (new_buffer)
      status = mmal_port_send_buffer(port, new_buffer);

    if (!new_buffer || status != MMAL_SUCCESS) {
      vcos_log_error("Unable to return a buffer to the splitter port");
      RCLCPP_ERROR(g_node->get_logger(),"Unable to return a buffer to the splitter port");
    }
  }
}



/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T* create_camera_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* camera = 0;
  MMAL_ES_FORMAT_T* format;
  MMAL_PORT_T *video_port = nullptr, *still_port = nullptr;
  MMAL_STATUS_T status;

  /* Create the component */
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Failed to create camera component");
    RCLCPP_ERROR(g_node->get_logger(),"Failed to create camera component");
    goto error;
  }

  if (!camera->output_num) {
    vcos_log_error("Camera doesn't have output ports");
    RCLCPP_ERROR(g_node->get_logger(),"Camera doesn't have output ports");
    goto error;
  }

  video_port = camera->output[mmal::camera_port::video];
  still_port = camera->output[mmal::camera_port::capture];

  //  set up the camera configuration
  {
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config;
    cam_config.hdr.id = MMAL_PARAMETER_CAMERA_CONFIG;
    cam_config.hdr.size = sizeof(cam_config);
    cam_config.max_stills_w = state.width;
    cam_config.max_stills_h = state.height;
    cam_config.stills_yuv422 = 0;
    cam_config.one_shot_stills = 0;
    cam_config.max_preview_video_w = state.width;
    cam_config.max_preview_video_h = state.height;
    cam_config.num_preview_video_frames = 3;
    cam_config.stills_capture_circular_buffer_height = 0;
    cam_config.fast_preview_resume = 0;
    cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC;

    mmal_port_parameter_set(camera->control, &cam_config.hdr);
  }

  // Select the camera to use
  {  
    MMAL_PARAMETER_INT32_T camera_num;
    camera_num.hdr.id = MMAL_PARAMETER_CAMERA_NUM;
    camera_num.hdr.size = sizeof(camera_num);
    camera_num.value = state.camera_id;

    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);
    if (status != MMAL_SUCCESS) {
      RCLCPP_ERROR(g_node->get_logger(),"Could not select camera : error %d", status);
      goto error;
    }
  }

  // Now set up the port formats

  // Set the encode format on the video  port

  format = video_port->format;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->encoding = MMAL_ENCODING_I420;
  format->es->video.width = state.width;
  format->es->video.height = state.height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state.width;
  format->es->video.crop.height = state.height;
  format->es->video.frame_rate.num = state.framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(video_port);

  if (status) {
    vcos_log_error("camera video format couldn't be set");
    RCLCPP_ERROR(g_node->get_logger(),"camera video format couldn't be set");
    goto error;
  }

  // Ensure there are enough buffers to avoid dropping frames
  if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Set the encode format on the still  port

  format = still_port->format;

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->es->video.width = state.width;
  format->es->video.height = state.height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state.width;
  format->es->video.crop.height = state.height;
  format->es->video.frame_rate.num = 1;
  format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(still_port);

  if (status) {
    vcos_log_error("camera still format couldn't be set");
    RCLCPP_ERROR(g_node->get_logger(),"camera still format couldn't be set");
    goto error;
  }

  video_port->buffer_num = video_port->buffer_num_recommended;
  /* Ensure there are enough buffers to avoid dropping frames */
  if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  /* Enable component */
  status = mmal_component_enable(camera);

  if (status) {
    vcos_log_error("camera component couldn't be enabled");
    RCLCPP_ERROR(g_node->get_logger(),"camera component couldn't be enabled");
    goto error;
  }

  raspicamcontrol_set_all_parameters(*camera, state.camera_parameters);

  state.camera_component.reset(camera);

  RCLCPP_DEBUG(g_node->get_logger(),"Camera component done\n");

  return camera;

error:

  if (camera)
    mmal_component_destroy(camera);
  return 0;
}


/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* encoder = 0;
  MMAL_PORT_T *encoder_input = nullptr, *encoder_output = nullptr;
  MMAL_STATUS_T status;
  MMAL_POOL_T* pool;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to create video encoder component");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to create video encoder component");
    goto error;
  }

  if (!encoder->input_num || !encoder->output_num) {
    status = MMAL_ENOSYS;
    vcos_log_error("Video encoder doesn't have input/output ports");
    RCLCPP_ERROR(g_node->get_logger(),"Video encoder doesn't have input/output ports");
    goto error;
  }

  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];

  // We want same format on input and output
  mmal_format_copy(encoder_output->format, encoder_input->format);

  // Only supporting H264 at the moment
  encoder_output->format->encoding = MMAL_ENCODING_JPEG;

  encoder_output->buffer_size = encoder_output->buffer_size_recommended;

  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  encoder_output->buffer_num = encoder_output->buffer_num_recommended;

  if (encoder_output->buffer_num < encoder_output->buffer_num_min)
    encoder_output->buffer_num = encoder_output->buffer_num_min;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on video encoder output port");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to set format on video encoder output port");
    goto error;
  }

  // Set the JPEG quality level
  status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state.quality);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set JPEG quality");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to set JPEG quality");
    goto error;
  }

  //  Enable component
  status = mmal_component_enable(encoder);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to enable video encoder component");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to enable video encoder component");
    goto error;
  }

  /* Create pool of buffer headers for the output port to consume */
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

  if (!pool) {
    vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
    RCLCPP_ERROR(g_node->get_logger(),"Failed to create buffer header pool for encoder output port %s", encoder_output->name);
  }

  state.encoder_pool = mmal::pool_ptr(pool, [encoder](MMAL_POOL_T* ptr) {
    if (encoder->output[0] && encoder->output[0]->is_enabled) {
      mmal_port_disable(encoder->output[0]);
    }
    mmal_port_pool_destroy(encoder->output[0], ptr);
  });
  state.encoder_component.reset(encoder);

  RCLCPP_DEBUG(g_node->get_logger(),"Encoder component done\n");

  return status;

error:
  if (encoder)
    mmal_component_destroy(encoder);

  return status;
}



/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE& state) {
  MMAL_COMPONENT_T* splitter = 0;
  MMAL_PORT_T *splitter_input = nullptr;
  MMAL_PORT_T *splitter_output_enc = nullptr, *splitter_output_raw = nullptr;
  MMAL_STATUS_T status;
  MMAL_POOL_T* pool;
  MMAL_ES_FORMAT_T *format;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to create video encoder component");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to create video encoder component");
    goto error;
  }

  if (!splitter->input_num) {
    status = MMAL_ENOSYS;
    RCLCPP_ERROR(g_node->get_logger(),"Video splitter doesn't have input ports");
    goto error;
  }

  if (splitter->output_num < 2) {
    status = MMAL_ENOSYS;
    RCLCPP_ERROR(g_node->get_logger(),"Video splitter doesn't have enough output ports");
    goto error;
  }

  /*** Input Port setup ***/

  splitter_input = splitter->input[0];

  // We want same format on input as camera output
  mmal_format_copy(splitter_input->format, state.camera_component->output[mmal::camera_port::video]->format);

  if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(splitter_input);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to set format on splitter input port");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to set format on splitter input port");
    goto error;
  }

  /*** Output to Encoder setup ***/

  splitter_output_enc = splitter->output[0];

  // Copy the format from the splitter input
  mmal_format_copy(splitter_output_enc->format, splitter_input->format);

  status = mmal_port_format_commit(splitter_output_enc);

  if (status != MMAL_SUCCESS) {
     vcos_log_error("Unable to set format on splitter output port for encoder");
     goto error;
  }


  /*** Output for raw ***/

  splitter_output_raw = splitter->output[1];

  // Copy the format from the splitter input
  mmal_format_copy(splitter_output_raw->format, splitter_input->format);

  // Use BGR24 (bgr8 in ROS)
  format = splitter_output_raw->format;
  format->encoding = MMAL_ENCODING_BGR24;
  format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */

  status = mmal_port_format_commit(splitter_output_raw);

  if (status != MMAL_SUCCESS) {
     vcos_log_error("Unable to set format on splitter output port for raw");
     goto error;
  }

  /*** Setup all other output ports ***/

  // start from 2
  for (unsigned int i = 2; i < splitter->output_num; i++) {

    mmal_format_copy(splitter->output[i]->format, splitter_input->format);

    status = mmal_port_format_commit(splitter->output[i]);

    if (status != MMAL_SUCCESS) {
       vcos_log_error("Unable to set format on splitter output port %d", i);
       goto error;
    }
  }

  /*** Enable component ***/

  status = mmal_component_enable(splitter);

  if (status != MMAL_SUCCESS) {
    vcos_log_error("Unable to enable splitter component");
    RCLCPP_ERROR(g_node->get_logger(),"Unable to enable splitter component");
    goto error;
  }

  /*** Create Pool ***/

  // Create pool of buffer headers for the raw output port to consume
  pool = mmal_port_pool_create(splitter_output_raw, splitter_output_raw->buffer_num, splitter_output_raw->buffer_size);

  if (!pool) {
    vcos_log_error("Failed to create buffer header pool for encoder output port %s", splitter_output_raw->name);
    RCLCPP_ERROR(g_node->get_logger(),"Failed to create buffer header pool for encoder output port %s", splitter_output_raw->name);
  }

  /*** Push to state struct ***/

  state.splitter_pool = mmal::pool_ptr(pool, [splitter](MMAL_POOL_T* ptr) {
    if (splitter->output[1] && splitter->output[1]->is_enabled) {
      mmal_port_disable(splitter->output[1]);
    }
    mmal_port_pool_destroy(splitter->output[1], ptr);
  });

  state.splitter_component.reset(splitter);

  RCLCPP_INFO(g_node->get_logger(),"splitter component done\n");

  return status;

error:
  if (splitter)
    mmal_component_destroy(splitter);

  return status;
}


/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function
 * successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T* output_port, MMAL_PORT_T* input_port,
                                   mmal::connection_ptr& connection) {
  MMAL_STATUS_T status;

  MMAL_CONNECTION_T* new_connection = nullptr;

  status = mmal_connection_create(&new_connection, output_port, input_port,
                                  MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

  if (status == MMAL_SUCCESS) {
    status = mmal_connection_enable(new_connection);
    if (status != MMAL_SUCCESS)
      mmal_connection_destroy(new_connection);
  }

  connection.reset(new_connection);

  return status;
}


/**
 * init_cam
 */
int init_cam(RASPIVID_STATE& state) {
  MMAL_STATUS_T status;
  MMAL_PORT_T* camera_video_port = nullptr;
  MMAL_PORT_T* splitter_input_port = nullptr;
  MMAL_PORT_T* splitter_output_enc = nullptr;
  MMAL_PORT_T* splitter_output_raw = nullptr;
  MMAL_PORT_T* encoder_input_port = nullptr;
  MMAL_PORT_T* encoder_output_port = nullptr;

  bcm_host_init();
  // Register our application with the logging system
  vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

  // OK, we have a nice set of parameters. Now set up our components
  // We have three components. Camera, splitter and encoder.

    RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM");



  if (!create_camera_component(state)) {
    RCLCPP_ERROR(g_node->get_logger(),"%s: Failed to create camera component", __func__);
  } else if ((status = create_encoder_component(state)) != MMAL_SUCCESS) {
    RCLCPP_ERROR(g_node->get_logger(),"%s: Failed to create encode component", __func__);
    state.camera_component.reset(nullptr);
  } else if ((status = create_splitter_component(state)) != MMAL_SUCCESS) {
    RCLCPP_ERROR(g_node->get_logger(),"%s: Failed to create splitter component", __func__);
    state.encoder_component.reset(nullptr);
    state.camera_component.reset(nullptr);
  } else {
    camera_video_port = state.camera_component->output[mmal::camera_port::video];
    splitter_input_port = state.splitter_component->input[0];
    splitter_output_enc = state.splitter_component->output[0];
    encoder_input_port = state.encoder_component->input[0];

        RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM");


    status = connect_ports(camera_video_port, splitter_input_port, state.splitter_connection);
    if (status != MMAL_SUCCESS) {
      RCLCPP_ERROR(g_node->get_logger(),"%s: Failed to connect camera video port to splitter input", __func__);
      return 1;
    }
    RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM");


    status = connect_ports(splitter_output_enc, encoder_input_port, state.encoder_connection);
    if (status != MMAL_SUCCESS) {
      RCLCPP_ERROR(g_node->get_logger(),"%s: Failed to connect camera splitter port to encoder input", __func__);
      return 1;
    }
    RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM");

    encoder_output_port = state.encoder_component->output[0];

    PORT_USERDATA* callback_data_enc = new PORT_USERDATA(state);
    callback_data_enc->buffer[0] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
    callback_data_enc->buffer[1] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
    // Set up our userdata - this is passed though to the callback where we
    // need the information.
    callback_data_enc->abort = false;
    callback_data_enc->id = 0;
    callback_data_enc->frame = 0;
    encoder_output_port->userdata = callback_data_enc;
    // Enable the encoder output port and tell it its callback function
    status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);
    if (status != MMAL_SUCCESS) {
      RCLCPP_ERROR(g_node->get_logger(),"Failed to setup encoder output");
      return 1;
    }
    RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM");

    if (state.enable_raw_pub) {
      splitter_output_raw = state.splitter_component->output[1];

      PORT_USERDATA* callback_data_raw = new PORT_USERDATA(state);
      callback_data_raw->buffer[0] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
      callback_data_raw->buffer[1] = std::make_unique<uint8_t[]>(IMG_BUFFER_SIZE);
      // Set up our userdata - this is passed though to the callback where we
      // need the information.
      callback_data_raw->abort = false;
      callback_data_raw->id = 0;
      callback_data_raw->frame = 0;
      splitter_output_raw->userdata = callback_data_raw;
      // Enable the encoder output port and tell it its callback function
      status = mmal_port_enable(splitter_output_raw, splitter_buffer_callback);
      if (status != MMAL_SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(),"Failed to setup encoder output");
        return 1;
      }
    }


    state.isInit = true;

    RCLCPP_INFO(g_node->get_logger(),"-----> INIT_CAM---> %d", state.isInit );

  }
  return 0;
}


int start_capture(RASPIVID_STATE& state) {

  if (state.isInit !=1){
    RCLCPP_ERROR(g_node->get_logger(), "Tried to start capture before camera is inited");
  }

  MMAL_PORT_T* camera_video_port = state.camera_component->output[mmal::camera_port::video];
  MMAL_PORT_T* encoder_output_port = state.encoder_component->output[0];
  MMAL_PORT_T* splitter_output_raw = state.splitter_component->output[1];
  RCLCPP_INFO(g_node->get_logger(),"Starting video capture (%d, %d, %d, %d)\n", state.width, state.height, state.quality, state.framerate);

  if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
    return 1;
  }
  // Send all the buffers to the encoder output port
  {
    int num = mmal_queue_length(state.encoder_pool->queue);
    int q;
    for (q = 0; q < num; q++) {
      MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.encoder_pool->queue);

      if (!buffer) {
        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        RCLCPP_ERROR(g_node->get_logger(),"Unable to get a required buffer %d from pool queue", q);
      }

      if (mmal_port_send_buffer(encoder_output_port, buffer) != MMAL_SUCCESS) {
        vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
        RCLCPP_ERROR(g_node->get_logger(),"Unable to send a buffer to encoder output port (%d)", q);
      }
    }
  }
  // Send all the buffers to the splitter output port
  if (state.enable_raw_pub) {
    int num = mmal_queue_length(state.splitter_pool->queue);
    int q;
    for (q = 0; q < num; q++) {
      MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(state.splitter_pool->queue);

      if (!buffer) {
        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
        RCLCPP_ERROR(g_node->get_logger(),"Unable to get a required buffer %d from pool queue", q);
      }

      if (mmal_port_send_buffer(splitter_output_raw, buffer) != MMAL_SUCCESS) {
        vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
        RCLCPP_ERROR(g_node->get_logger(),"Unable to send a buffer to splitter output port (%d)", q);
      }
    }
  }
  RCLCPP_INFO(g_node->get_logger(),"Video capture started\n");
  return 0;
}


int close_cam(RASPIVID_STATE& state) {
  if (state.isInit) {
    state.isInit = false;
    MMAL_COMPONENT_T* camera = state.camera_component.get();
    MMAL_COMPONENT_T* encoder = state.encoder_component.get();
    MMAL_COMPONENT_T* splitter = state.splitter_component.get();

    // Destroy encoder port connection
    state.encoder_connection.reset(nullptr);

    // Destroy splitter port connection
    state.splitter_connection.reset(nullptr);

    // Destroy encoder component
    if (encoder) {
      // Get rid of any port buffers first
      state.encoder_pool.reset(nullptr);
      // Delete callback structure
      delete encoder->output[0]->userdata;
      state.encoder_component.reset(nullptr);
    }

    // Destroy splitter component
    if (splitter) {
      // Get rid of any port buffers first
      state.splitter_pool.reset(nullptr);
      // Delete callback structure
      if (splitter->output[1]->userdata) {
        delete splitter->output[1]->userdata;
      }
      state.splitter_component.reset(nullptr);
    }

    // destroy camera component
    if (camera) {
      state.camera_component.reset(nullptr);
    }
    RCLCPP_INFO(g_node->get_logger(),"Video capture stopped\n");
    return 0;
  } else
    return 1;
}



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("raspicam_node");


  skip_frames = 0;



  std::string camera_info_url =  std::string("package://raspicam_node/camera_info/camera.yaml");
  std::string camera_name = std::string("camera");

  //camera_info_manager::CameraInfoManager c_info_man(g_node, camera_name, camera_info_url);

  RASPIVID_STATE state_srv;

  configure_parameters(state_srv);
  init_cam(state_srv);
    RCLCPP_INFO(g_node->get_logger(),"-----> AFTER INIT_CAM---> %d", state_srv.isInit );

/*
  if (!c_info_man.loadCameraInfo(camera_info_url)) {
    RCLCPP_INFO(g_node->get_logger(),"Calibration file missing. Camera not calibrated");
  } else {
    c_info = c_info_man.getCameraInfo();
    RCLCPP_INFO(g_node->get_logger(),"Camera successfully calibrated from default file");
  }

  if (!c_info_man.loadCameraInfo("")) {
    RCLCPP_INFO(g_node->get_logger(),"No device specifc calibration found");
  } else {
    c_info = c_info_man.getCameraInfo();
    RCLCPP_INFO(g_node->get_logger(),"Camera successfully calibrated from device specifc file");
  }
*/

  state_srv.enable_raw_pub = false;
  if (state_srv.enable_raw_pub){
    image_pub = g_node->create_publisher<ImageMsg>("image");
  }
  compressed_image_pub = g_node->create_publisher<CompressedImageMsg>("image/compressed");
  camera_info_pub = g_node->create_publisher<CameraInfoMsg>("camera_info");

  start_capture(state_srv);
  rclcpp::spin(g_node);
  close_cam(state_srv);
  rclcpp::shutdown();
}