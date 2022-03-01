#include "ci_pipuck_omnidirectional_camera_sensor.h"

#define ARGOS_WITH_LUA

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#include <argos3/core/wrappers/lua/lua_vector3.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   void CCI_PiPuckOmnidirectionalCameraSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* get the camera rotation and update the value in the control interface */
         Real fRotationDegrees = 0;
         GetNodeAttributeOrDefault(t_tree, "rotation", fRotationDegrees, DEFAULT_ROTATION);
         CRadians cRotation;
         cRotation.FromValueInDegrees(fRotationDegrees);
         m_cOrientationOffset = CQuaternion(cRotation, CVector3::Z);
         GetNodeAttributeOrDefault(t_tree, "resolution", m_cResolution, DEFAULT_RESOLUTION);
         /* calculate the default principal point */
         m_cPrincipalPoint = m_cResolution * 0.5;
         GetNodeAttributeOrDefault(t_tree, "principal_point", m_cPrincipalPoint, m_cPrincipalPoint);
         GetNodeAttributeOrDefault(t_tree, "focal_length", m_cFocalLength, DEFAULT_FOCAL_LENGTH);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in control interface", ex);
      }
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaEnablePiPuckOmnidirectionalCameraSensor(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 0) {
         return luaL_error(pt_lua_state, "robot.omnidirectional_camera.enable() expects zero arguments");
      }
      /* Get the camera sensor */
      CCI_PiPuckOmnidirectionalCameraSensor* pcCameraSensor = 
         CLuaUtility::GetDeviceInstance<CCI_PiPuckOmnidirectionalCameraSensor>(pt_lua_state, "omnidirectional_camera");
      /* Set the enable member */
      pcCameraSensor->Enable();
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaDisablePiPuckOmnidirectionalCameraSensor(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 0) {
         return luaL_error(pt_lua_state, "robot.omnidirectional_camera.disable() expects zero arguments");
      }
      /* Get the camera sensor */
      CCI_PiPuckOmnidirectionalCameraSensor* pcCameraSensor = 
         CLuaUtility::GetDeviceInstance<CCI_PiPuckOmnidirectionalCameraSensor>(pt_lua_state, "omnidirectional_camera");
      /* Set the enable member */
      pcCameraSensor->Disable();
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * the stack must contain a single table with keys named x, y, and z
    * which represent the X, Y, and Z components of a 3D vector
    */
   int LuaPiPuckOmnidirectionalCameraSensorDetectLed(lua_State* pt_lua_state) {
      /* check parameters */
      if(lua_gettop(pt_lua_state) != 1) {
         return luaL_error(pt_lua_state, "robot.omnidirectional_camera.detect_led() expects a single argument");
      }
      const CVector3& cPosition = CLuaVector3::ToVector3(pt_lua_state, 1);
      /* get the camera sensor */
      CCI_PiPuckOmnidirectionalCameraSensor* pcCameraSensor = 
         CLuaUtility::GetDeviceInstance<CCI_PiPuckOmnidirectionalCameraSensor>(pt_lua_state, "omnidirectional_camera");
      /* get the LED state */
      const CCI_PiPuckOmnidirectionalCameraSensor::ELedState& eLedState =
         pcCameraSensor->DetectLed(cPosition);
      /* convert the LED state to an integer and push it onto the stack */
      lua_pushinteger(pt_lua_state, static_cast<UInt8>(eLedState));
      /* return a single result, the integer */
      return 1;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PiPuckOmnidirectionalCameraSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "omnidirectional_camera");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state,
                              "enable",
                              &LuaEnablePiPuckOmnidirectionalCameraSensor);
      CLuaUtility::AddToTable(pt_lua_state,
                              "disable",
                              &LuaDisablePiPuckOmnidirectionalCameraSensor);
      CLuaUtility::AddToTable(pt_lua_state,
                              "detect_led",
                              &LuaPiPuckOmnidirectionalCameraSensorDetectLed);
      CLuaUtility::AddToTable(pt_lua_state, "timestamp", 0.0f);
      CLuaUtility::StartTable(pt_lua_state, "tags");
      const STag::TVector& vecTags = GetTags();
      for(size_t i = 0; i < vecTags.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i + 1);
         CLuaUtility::AddToTable(pt_lua_state, "id", vecTags[i].Id);
         CLuaUtility::AddToTable(pt_lua_state, "position", vecTags[i].Position);
         CLuaUtility::AddToTable(pt_lua_state, "orientation", vecTags[i].Orientation);
         CLuaUtility::AddToTable(pt_lua_state, "center", vecTags[i].Center);
         /* start corners */
         CLuaUtility::StartTable(pt_lua_state, "corners");
         for(size_t j = 0; j < vecTags[i].Corners.size(); ++j) {           
            CLuaUtility::AddToTable(pt_lua_state, j + 1, vecTags[i].Corners[j]);
         }
         CLuaUtility::EndTable(pt_lua_state);
         /* end corners */
         CLuaUtility::EndTable(pt_lua_state);
      }
      CLuaUtility::EndTable(pt_lua_state);
      CLuaUtility::StartTable(pt_lua_state, "transform");
      CLuaUtility::AddToTable(pt_lua_state, "position", POSITION_OFFSET);
      CLuaUtility::AddToTable(pt_lua_state, "orientation", m_cOrientationOffset);
      CLuaUtility::AddToTable(pt_lua_state, "anchor", "origin");
      CLuaUtility::EndTable(pt_lua_state);
      CLuaUtility::AddToTable(pt_lua_state, "resolution", m_cResolution);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PiPuckOmnidirectionalCameraSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "omnidirectional_camera");
      CLuaUtility::AddToTable(pt_lua_state, "timestamp", GetTimestamp());
      lua_getfield(pt_lua_state, -1, "tags");
      /* get the tag count from last time */
      size_t unLastTagCount = lua_rawlen(pt_lua_state, -1);
      const STag::TVector& vecTags = GetTags();
      for(size_t i = 0; i < vecTags.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i + 1);
         CLuaUtility::AddToTable(pt_lua_state, "id", vecTags[i].Id);
         CLuaUtility::AddToTable(pt_lua_state, "position", vecTags[i].Position);
         CLuaUtility::AddToTable(pt_lua_state, "orientation", vecTags[i].Orientation);
         CLuaUtility::AddToTable(pt_lua_state, "center", vecTags[i].Center);
         /* start corners */
         CLuaUtility::StartTable(pt_lua_state, "corners");
         for(size_t j = 0; j < vecTags[i].Corners.size(); ++j) {           
            CLuaUtility::AddToTable(pt_lua_state, j + 1, vecTags[i].Corners[j]);
         }
         CLuaUtility::EndTable(pt_lua_state);
         /* end corners */
         CLuaUtility::EndTable(pt_lua_state);
      }
      if(vecTags.size() < unLastTagCount) {
         /* Remove extra tags from the last update by setting them to nil */
         for(size_t i = vecTags.size() + 1; i <= unLastTagCount; ++i) {
            lua_pushnumber(pt_lua_state,  i);
            lua_pushnil   (pt_lua_state    );
            lua_settable  (pt_lua_state, -3);
         }
      }
      lua_pop(pt_lua_state, 1);
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/
   
   const CVector3 CCI_PiPuckOmnidirectionalCameraSensor::POSITION_OFFSET = CVector3(0.0, 0.0, 0.0925);
   const Real CCI_PiPuckOmnidirectionalCameraSensor::DEFAULT_ROTATION = 0.0;
   const CVector2 CCI_PiPuckOmnidirectionalCameraSensor::DEFAULT_RESOLUTION = CVector2(640, 480);
   const CVector2 CCI_PiPuckOmnidirectionalCameraSensor::DEFAULT_FOCAL_LENGTH = CVector2(1000, 1000);

   /****************************************/
   /****************************************/

}
