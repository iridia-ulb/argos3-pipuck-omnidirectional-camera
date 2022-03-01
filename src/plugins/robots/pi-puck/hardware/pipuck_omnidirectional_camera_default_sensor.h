
#ifndef PIPUCK_OMNIDIRECTIONAL_CAMERA_DEFAULT_SENSOR_H
#define PIPUCK_OMNIDIRECTIONAL_CAMERA_DEFAULT_SENSOR_H

struct apriltag_family;
struct apriltag_detector;
struct v4l2_buffer;

#include <array>
#include <chrono>

#include <apriltag/apriltag_pose.h>
#include <apriltag/common/image_u8.h>

#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

#include <argos3/plugins/robots/pi-puck/hardware/sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_omnidirectional_camera_sensor.h>

#include <linux/videodev2.h>

namespace argos {

   class CPiPuckOmnidirectionalCameraDefaultSensor : public CPhysicalSensor,
                                                     public CCI_PiPuckOmnidirectionalCameraSensor {

   public:

      CPiPuckOmnidirectionalCameraDefaultSensor();

      virtual ~CPiPuckOmnidirectionalCameraDefaultSensor();

      void Init(TConfigurationNode& t_tree);

      void Destroy();

      void Update();

      void Enable();

      void Disable();

      virtual ELedState DetectLed(const CVector3& c_position) const;

      virtual const STag::TVector& GetTags() const {
         return m_vecTags;
      }

      virtual Real GetTimestamp() const {
         return m_fTimestamp;
      }

   private:
      CSquareMatrix<3> m_cCameraMatrix;

      std::array<std::pair<UInt32, void*>, 2> m_arrBuffers;
      std::array<std::pair<UInt32, void*>, 2>::iterator m_itCurrentBuffer;
      std::array<std::pair<UInt32, void*>, 2>::iterator m_itNextBuffer;

      ::image_u8_t* m_ptImage;
      ::apriltag_family* m_ptTagFamily;
      ::apriltag_detector* m_ptTagDetector;
      ::apriltag_detection_info_t m_tTagDetectionInfo;

      /* camera device handle */
      SInt32 m_nCameraHandle;

      /* time at initialization */
      std::chrono::steady_clock::time_point m_tpInit;

      STag::TVector m_vecTags;
      Real m_fTimestamp;

      /****************************************/
      /****************************************/

   };
}

#endif
