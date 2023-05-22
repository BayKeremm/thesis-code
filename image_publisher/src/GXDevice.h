
#pragma once 
#include <camera/DxImageProc.h>
#include <camera/GxIAPI.h>

/* NOTE: RGGB is the color scheme */ 


class GXDevice{
    private: 
      GX_STATUS m_Status; 
     GX_DEV_HANDLE  m_DevHandle; 
     GX_OPEN_PARAM m_OpenParams; 
     PGX_FRAME_BUFFER m_ImageBuffer; 
     bool OpenDevice(); 
     bool CloseDevice(); 
  

    public: 
    // This class is assuming 1 device 
   
     GXDevice() = default; 
     ~GXDevice() = default; 
     bool Init(); 
     bool Shutdown(); 

    /*
        I decided to aquire images one frame at a time, so that we're able to process an image before we receive the new one ; meaning that the program wait until a new frame is aquired

    */
    
    bool StartStream(); 
    bool StopStream(); 
    PGX_FRAME_BUFFER AquireImage(); 
    
     /*bool AquireNewImage(){
        // Test speed of aquisition
        GX_CALL(&m_Status, GXSendCommand(m_DevHandle, GX_COMMAND_ACQUISITION_START)); 
        GX_CALL(&m_Status, GXSendCommand(m_DevHandle, GX_COMMAND_ACQUISITION_STOP));
        // test end 
        return true;  
    }
    
     bool SetDeviceCallback(void (*callback)(GX_FRAME_CALLBACK_PARAM* p_FrameData)){
        GX_CALL(&m_Status, GXRegisterCaptureCallback(m_DevHandle,NULL,callback)); 
        return true; 
    }
    */

    /*  GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_GAIN, 16));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.4));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.5));*/
}; 