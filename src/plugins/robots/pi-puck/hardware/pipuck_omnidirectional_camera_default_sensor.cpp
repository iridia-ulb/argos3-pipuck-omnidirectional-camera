
#include "pipuck_omnidirectional_camera_default_sensor.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>

#include <regex>
#include <chrono>

// --format YUYV --size 640x480 

#define FILE_VIDEO_DEVICE "/dev/camera1"

#define IMAGE_BYTES_PER_PIXEL 2u

#define TAG_SIDE_LENGTH 0.0235f

#define DETECT_LED_WIDTH 6u
#define DETECT_LED_HEIGHT 6u
#define DETECT_LED_THRES_Q1_MIN_V 140u
#define DETECT_LED_THRES_Q1_MIN_U 140u
#define DETECT_LED_THRES_Q2_MIN_V 140u
#define DETECT_LED_THRES_Q2_MAX_U 100u
#define DETECT_LED_THRES_Q3_MAX_V 110u
#define DETECT_LED_THRES_Q3_MAX_U 130u
#define DETECT_LED_THRES_Q4_MAX_V 100u
#define DETECT_LED_THRES_Q4_MIN_U 145u

namespace argos {

   /****************************************/
   /****************************************/
   
   CPiPuckOmnidirectionalCameraDefaultSensor::CPiPuckOmnidirectionalCameraDefaultSensor():
      m_ptImage(nullptr),
      m_ptTagFamily(nullptr),
      m_ptTagDetector(nullptr),
      m_nCameraHandle(-1),
      m_fTimestamp(0.0) {
      /* initialize the apriltag components */
      m_ptTagFamily = ::tag36h11_create();
      /* create the tag detector */
      m_ptTagDetector = ::apriltag_detector_create();
      /* add the tag family to the tag detector */
      ::apriltag_detector_add_family(m_ptTagDetector, m_ptTagFamily);
      /* configure the tag detector */
      m_ptTagDetector->quad_decimate = 1.0f;
      m_ptTagDetector->quad_sigma = 0.0f;
      m_ptTagDetector->refine_edges = 1;
      m_ptTagDetector->decode_sharpening = 0.25;
      m_ptTagDetector->nthreads = 1;
   }

   /****************************************/
   /****************************************/

   CPiPuckOmnidirectionalCameraDefaultSensor::~CPiPuckOmnidirectionalCameraDefaultSensor() {
      /* uninitialize the apriltag components */
      ::apriltag_detector_remove_family(m_ptTagDetector, m_ptTagFamily);
      /* destroy the tag detector */
      ::apriltag_detector_destroy(m_ptTagDetector);
      /* destroy the tag family */
      ::tag36h11_destroy(m_ptTagFamily);
   }
   
   /****************************************/
   /****************************************/

   void CPiPuckOmnidirectionalCameraDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_PiPuckOmnidirectionalCameraSensor::Init(t_tree);
         /* allocate image memory */
         m_ptImage = ::image_u8_create_alignment(
            static_cast<unsigned int>(CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetX()),
            static_cast<unsigned int>(CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetY()),
            96);
         /* set the camera matrix */
         m_cCameraMatrix.SetIdentityMatrix();
         m_cCameraMatrix(0,0) = CCI_PiPuckOmnidirectionalCameraSensor::m_cFocalLength.GetX();
         m_cCameraMatrix(1,1) = CCI_PiPuckOmnidirectionalCameraSensor::m_cFocalLength.GetY();
         m_cCameraMatrix(0,2) = CCI_PiPuckOmnidirectionalCameraSensor::m_cPrincipalPoint.GetX();
         m_cCameraMatrix(1,2) = CCI_PiPuckOmnidirectionalCameraSensor::m_cPrincipalPoint.GetY();
         /* set the tag detection info structure */
         m_tTagDetectionInfo.fx = CCI_PiPuckOmnidirectionalCameraSensor::m_cFocalLength.GetX();
         m_tTagDetectionInfo.fy = CCI_PiPuckOmnidirectionalCameraSensor::m_cFocalLength.GetY();
         m_tTagDetectionInfo.cx = CCI_PiPuckOmnidirectionalCameraSensor::m_cPrincipalPoint.GetX();
         m_tTagDetectionInfo.cy = CCI_PiPuckOmnidirectionalCameraSensor::m_cPrincipalPoint.GetY();
         m_tTagDetectionInfo.tagsize = TAG_SIDE_LENGTH;
         /* open and configure the video device */
         if ((m_nCameraHandle = ::open(FILE_VIDEO_DEVICE, O_RDWR, 0)) < 0)
            THROW_ARGOSEXCEPTION("Could not open " << FILE_VIDEO_DEVICE);
         int nInput = 0;
         if (::ioctl(m_nCameraHandle, VIDIOC_S_INPUT, &nInput) < 0)
            THROW_ARGOSEXCEPTION("Could not set " << FILE_VIDEO_DEVICE << " as an input");
         /* set camera format*/
         v4l2_format sFormat;
         //if (::ioctl(m_nCameraHandle, VIDIOC_G_FMT, &sFmt) < 0)
         //   THROW_ARGOSEXCEPTION("Could not get the camera format");
         sFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         sFormat.fmt.pix.width =
            static_cast<unsigned int>(CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetX());
         sFormat.fmt.pix.height =
            static_cast<unsigned int>(CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetY());;
         sFormat.fmt.pix.sizeimage =
            sFormat.fmt.pix.width * sFormat.fmt.pix.height * IMAGE_BYTES_PER_PIXEL;
         sFormat.fmt.pix.bytesperline =
            sFormat.fmt.pix.width * IMAGE_BYTES_PER_PIXEL;
         sFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
         sFormat.fmt.pix.field = V4L2_FIELD_NONE;
         if (::ioctl(m_nCameraHandle, VIDIOC_S_FMT, &sFormat) < 0)
            THROW_ARGOSEXCEPTION("Could not set the camera format");
         /* request camera buffers */
         v4l2_requestbuffers sRequest;
         ::memset(&sRequest, 0, sizeof(sRequest));
         sRequest.count = m_arrBuffers.size();
         sRequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         sRequest.memory= V4L2_MEMORY_MMAP;
         if (::ioctl(m_nCameraHandle, VIDIOC_REQBUFS, &sRequest) < 0) {
            THROW_ARGOSEXCEPTION("Could not request buffers");
         }
         if (sRequest.count < m_arrBuffers.size()) {
            THROW_ARGOSEXCEPTION("Could not get the requested number of buffers");
         }
         /* remap and enqueue the buffers */
         v4l2_buffer sBuffer;
         UInt32 unBufferIndex = 0;
         for(std::pair<UInt32, void*>& t_buffer : m_arrBuffers) {
            /* zero the buffer */
            ::memset(&sBuffer, 0, sizeof(v4l2_buffer));
            sBuffer.type = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
            sBuffer.memory = ::V4L2_MEMORY_MMAP;
            sBuffer.index = unBufferIndex;
            if(::ioctl(m_nCameraHandle, VIDIOC_QUERYBUF, &sBuffer) < 0) {
               THROW_ARGOSEXCEPTION("Could not query buffer");
            }
            std::get<UInt32>(t_buffer) =
               unBufferIndex;
            std::get<void*>(t_buffer) =
               ::mmap(nullptr, sBuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                      m_nCameraHandle, sBuffer.m.offset);
            /* increment the buffer index */
            unBufferIndex++;
         }
         /* intialize iterators for the buffers */
         m_itNextBuffer = std::begin(m_arrBuffers);
         m_itCurrentBuffer = std::end(m_arrBuffers);
         /* start the stream */
         enum v4l2_buf_type eBufferType = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
         if(::ioctl(m_nCameraHandle, VIDIOC_STREAMON, &eBufferType) < 0) {
            THROW_ARGOSEXCEPTION("Could not start the stream");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPiPuckOmnidirectionalCameraDefaultSensor::Enable() {
      if(!IsEnabled()) {
         /* enqueue the first buffer */
         ::v4l2_buffer sBuffer;
         ::memset(&sBuffer, 0, sizeof(::v4l2_buffer));
         sBuffer.type = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
         sBuffer.memory = ::V4L2_MEMORY_MMAP;
         sBuffer.index = m_itNextBuffer->first;
         if(::ioctl(m_nCameraHandle, VIDIOC_QBUF, &sBuffer) < 0) {
            THROW_ARGOSEXCEPTION("Could not enqueue used buffer");
         }
         CCI_PiPuckOmnidirectionalCameraSensor::Enable();
      }
   }

   /****************************************/
   /****************************************/

   void CPiPuckOmnidirectionalCameraDefaultSensor::Disable() {
      if(IsEnabled()) {
         /* dequeue the next buffer */
         ::v4l2_buffer sBuffer;
         memset(&sBuffer, 0, sizeof(v4l2_buffer));
         sBuffer.type = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
         sBuffer.memory = ::V4L2_MEMORY_MMAP;
         sBuffer.index = m_itNextBuffer->first;
         if(::ioctl(m_nCameraHandle, VIDIOC_DQBUF, &sBuffer) < 0) {
            LOGERR << "[WARNING] Could not dequeue buffer" << std::endl;
         }
         CCI_PiPuckOmnidirectionalCameraSensor::Disable();
      }
   }

   /****************************************/
   /****************************************/

   void CPiPuckOmnidirectionalCameraDefaultSensor::Update() {
      /* clear out previous readings */
      m_vecTags.clear();
      /* update the timestamp in the control interface */
      using namespace std::chrono;
      m_fTimestamp = duration_cast<duration<Real> >(steady_clock::now() - m_tpInit).count();
      /* if enabled, process the next buffer */
      if(IsEnabled()) {
         try {
            /* update the buffer iterators */
            m_itCurrentBuffer = m_itNextBuffer;
            m_itNextBuffer++;
            if(m_itNextBuffer == std::end(m_arrBuffers)) {
               m_itNextBuffer = std::begin(m_arrBuffers);
            }
            /* dequeue the buffer */
            ::v4l2_buffer sBuffer;
            memset(&sBuffer, 0, sizeof(v4l2_buffer));
            sBuffer.type = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
            sBuffer.memory = ::V4L2_MEMORY_MMAP;
            sBuffer.index = m_itCurrentBuffer->first;
            if(::ioctl(m_nCameraHandle, VIDIOC_DQBUF, &sBuffer) < 0) {
               THROW_ARGOSEXCEPTION("Could not dequeue buffer");
            }
            /* get a pointer to the image data */
            uint8_t* punImageData = static_cast<uint8_t*>(m_itCurrentBuffer->second);
            /* create the gray scale image based on the luminance data */
            UInt32 unSourceIndex = 0;
            UInt32 unDestinationIndex = 0;
            const UInt32 unImageStride = m_ptImage->stride;
            const UInt32 unImageWidth = m_ptImage->width;
            const UInt32 unImageHeight = m_ptImage->height;
            /* extract the luminance from the data */
            for (UInt32 un_height_index = 0; un_height_index < unImageHeight; un_height_index++) {
               for (UInt32 un_width_index = 0; un_width_index < unImageWidth; un_width_index++) {
                  /* extract the luminance from the data (assumes V4L2_PIX_FMT_YUYV) */
                  m_ptImage->buf[unDestinationIndex++] = punImageData[unSourceIndex];
                  /* move to the next pixel */
                  unSourceIndex += 2;
               }
               unDestinationIndex += (unImageStride - unImageWidth);
            }
            /* detect the tags */
            CVector2 cCenterPixel;
            std::array<CVector2, 4> arrCornerPixels;
            /* run the apriltags algorithm */
            ::zarray_t* ptDetectionArray =
                 ::apriltag_detector_detect(m_ptTagDetector, m_ptImage);
            /* get the detected tags count */
            size_t unTagCount = static_cast<size_t>(::zarray_size(ptDetectionArray));
            /* reserve space for the tags */
            m_vecTags.reserve(unTagCount);
            /* copy detection data to the control interface */
            for(size_t un_index = 0; un_index < unTagCount; un_index++) {
               ::apriltag_detection_t *ptDetection;
               ::zarray_get(ptDetectionArray, un_index, &ptDetection);
               /* copy the tag corner coordinates */
               arrCornerPixels[0].Set(ptDetection->p[0][0], ptDetection->p[0][1]);
               arrCornerPixels[1].Set(ptDetection->p[1][0], ptDetection->p[1][1]);
               arrCornerPixels[2].Set(ptDetection->p[2][0], ptDetection->p[2][1]);
               arrCornerPixels[3].Set(ptDetection->p[3][0], ptDetection->p[3][1]);
               /* copy the tag center coordinate */
               cCenterPixel.Set(ptDetection->c[0], ptDetection->c[1]);
               /* calculate tag pose */
               apriltag_pose_t tPose;
               m_tTagDetectionInfo.det = ptDetection;
               ::estimate_tag_pose(&m_tTagDetectionInfo, &tPose);
               CRotationMatrix3 cTagOrientation(tPose.R->data);
               CVector3 cTagPosition(tPose.t->data[0], tPose.t->data[1], tPose.t->data[2]);
                /* copy readings */
               m_vecTags.emplace_back(ptDetection->id, cTagPosition, cTagOrientation, cCenterPixel, arrCornerPixels);
            }
            /* destroy the readings array */
            ::apriltag_detections_destroy(ptDetectionArray);
            /* enqueue the next buffer */
            ::memset(&sBuffer, 0, sizeof(::v4l2_buffer));
            sBuffer.type = ::V4L2_BUF_TYPE_VIDEO_CAPTURE;
            sBuffer.memory = ::V4L2_MEMORY_MMAP;
            sBuffer.index = m_itNextBuffer->first;
            if(::ioctl(m_nCameraHandle, VIDIOC_QBUF, &sBuffer) < 0) {
               THROW_ARGOSEXCEPTION("Can not enqueue used buffer");
            }
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error updating the camera sensor", ex);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CPiPuckOmnidirectionalCameraDefaultSensor::Destroy() {
      /* disable the sensor */
      Disable();
      /* stop the stream */
      enum v4l2_buf_type eBufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (::ioctl(m_nCameraHandle, VIDIOC_STREAMOFF, &eBufferType) < 0) {
         LOGERR << "[WARNING] Could not stop the stream" << std::endl;
      }
      /* close the camera */
      ::close(m_nCameraHandle);
      /* deallocate image memory */
      ::image_u8_destroy(m_ptImage);
   }

   /****************************************/
   /****************************************/

   CPiPuckOmnidirectionalCameraDefaultSensor::ELedState
      CPiPuckOmnidirectionalCameraDefaultSensor::DetectLed(const CVector3& c_position) const {
      /* project the LED position onto the sensor array */
      CMatrix<3,1> cLedPosition;
      cLedPosition(0) = c_position.GetX();
      cLedPosition(1) = c_position.GetY();
      cLedPosition(2) = c_position.GetZ();
      const CMatrix<3,1> cProjection = m_cCameraMatrix * cLedPosition;
      CVector2 cCenter(cProjection(0,0) / cProjection(2,0),
                       cProjection(1,0) / cProjection(2,0));
      CVector2 cSize(DETECT_LED_WIDTH, DETECT_LED_HEIGHT);
      /* declare ranges for truncation */
      CRange<Real> m_cColumnRange(0.0f, CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetX() - 1.0f);
      CRange<Real> m_cRowRange(0.0f, CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetY() - 1.0f);
      /* calculate the corners of the region of interest */
      CVector2 cMinCorner(cCenter - 0.5f * cSize);
      CVector2 cMaxCorner(cCenter + 0.5f * cSize);
      /* clamp the region of interest to the image size */
      Real fColumnStart = cMinCorner.GetX();
      Real fColumnEnd = cMaxCorner.GetX();
      Real fRowStart = cMinCorner.GetY();
      Real fRowEnd = cMaxCorner.GetY();
      m_cColumnRange.TruncValue(fColumnStart);
      m_cColumnRange.TruncValue(fColumnEnd);
      m_cRowRange.TruncValue(fRowStart);
      m_cRowRange.TruncValue(fRowEnd);
      /* get the start/end indices */
      UInt32 unColumnStart = static_cast<UInt32>(std::round(fColumnStart));
      UInt32 unColumnEnd = static_cast<UInt32>(std::round(fColumnEnd));
      UInt32 unRowStart = static_cast<UInt32>(std::round(fRowStart));
      UInt32 unRowEnd = static_cast<UInt32>(std::round(fRowEnd));
      /* column must start and end at an even number due to pixel format */
      if(unColumnStart % 2) {
         ++unColumnStart;
      }
      if(unColumnEnd % 2) {
         --unColumnEnd;
      }
      /* initialize sums */
      Real fWeightedSumU = 0.0f;
      Real fSumY0 = 0.0f;
      Real fWeightedSumV = 0.0f;
      Real fSumY1 = 0.0f;
      /* get a pointer to the current image data */
      if(m_itCurrentBuffer == std::end(m_arrBuffers)) {
         THROW_ARGOSEXCEPTION("Current buffer is not ready");
      }
      UInt8* punImageData = static_cast<UInt8*>(m_itCurrentBuffer->second);
      /* extract the data */    
      for(UInt32 un_row = unRowStart; un_row < unRowEnd; un_row += 1) {
         UInt32 unRowIndex = 
            static_cast<UInt32>(CCI_PiPuckOmnidirectionalCameraSensor::m_cResolution.GetX()) *
            un_row * IMAGE_BYTES_PER_PIXEL;
         for(UInt32 un_column = unColumnStart * IMAGE_BYTES_PER_PIXEL;
             un_column < unColumnEnd * IMAGE_BYTES_PER_PIXEL;
             un_column += (2 * IMAGE_BYTES_PER_PIXEL)) {
            /* get a pointer to the start of the macro pixel */
            UInt8* punMacroPixel = punImageData + unRowIndex + un_column;
            /* extract the macro pixel */
            fWeightedSumU += static_cast<Real>(punMacroPixel[0]) * punMacroPixel[1];
            fSumY0 += static_cast<Real>(punMacroPixel[1]);
            fWeightedSumV += static_cast<Real>(punMacroPixel[2]) * punMacroPixel[3];
            fSumY1 += static_cast<Real>(punMacroPixel[3]);           
         }
      }
      /* default values if fSumY1,0 are zero */
      UInt8 unAverageU = 128u;
      UInt8 unAverageV = 128u;     
      if(fSumY0 != 0.0f) {
         unAverageU = static_cast<UInt8>(std::round(fWeightedSumU / fSumY0));
      }
      if(fSumY1 != 0.0f) {
         unAverageV = static_cast<UInt8>(std::round(fWeightedSumV / fSumY1));
      }
      /* initialize the LED state to off */
      ELedState eLedState = ELedState::OFF;    
      /* deduce the LED state from the UV values */
      if((unAverageV > DETECT_LED_THRES_Q1_MIN_V) && (unAverageU > DETECT_LED_THRES_Q1_MIN_U)) {
         eLedState = ELedState::Q1;
      }
      else if((unAverageV > DETECT_LED_THRES_Q2_MIN_V) && (unAverageU < DETECT_LED_THRES_Q2_MAX_U)) {
         eLedState = ELedState::Q2;
      }
      else if((unAverageV < DETECT_LED_THRES_Q3_MAX_V) && (unAverageU < DETECT_LED_THRES_Q3_MAX_U)) {
         eLedState = ELedState::Q3;
      }
      else if((unAverageV < DETECT_LED_THRES_Q4_MAX_V) && (unAverageU > DETECT_LED_THRES_Q4_MIN_U)) {
         eLedState = ELedState::Q4;
      }
      return eLedState;
   }

   /****************************************/
   /****************************************/
   
   REGISTER_SENSOR(CPiPuckOmnidirectionalCameraDefaultSensor,
                   "pipuck_omnidirectional_camera", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "Omnidirectional camera sensor for the PiPuck Robot",
                   "<TODO>",
                   "Under development");

   /****************************************/
   /****************************************/

}
