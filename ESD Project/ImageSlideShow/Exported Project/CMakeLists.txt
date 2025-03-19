# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_OBJECT_PATH_MAX 247)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(${PICO_SDK_INIT_CMAKE_FILE})

project(ImageSlide C CXX ASM)

# Initialise the Pico SDK
pico_sdk_init()

include_directories(
"Esd_Core"
"FT_Esd_Framework"
"FT_Esd_Widgets"
"FT_Esd_Widgets/ThirdPartylibs"
"FT_Eve_Hal"
"FT_Eve_Hal/Hdr"
"ImageSlide"
"ThirdPartyLib/ff14a"
"ThirdPartyLib/include"
"ThirdPartyLib/sd_driver"
)

# Add executable. Default name is the project name, version 0.1

add_executable(ImageSlide 
"Esd_Core/Esd_AnimationInfo.c"
"Esd_Core/Esd_Base.c"
"Esd_Core/Esd_BitmapHandle.c"
"Esd_Core/Esd_BitmapInfo.c"
"Esd_Core/Esd_CheckTypeSizes.c"
"Esd_Core/Esd_CoWidget.c"
"Esd_Core/Esd_Context.c"
"Esd_Core/Esd_FontInfo.c"
"Esd_Core/Esd_GpuAlloc.c"
"Esd_Core/Esd_LittleFS.c"
"Esd_Core/Esd_Math.c"
"Esd_Core/Esd_MemoryPool.c"
"Esd_Core/Esd_Render.c"
"Esd_Core/Esd_Render_Bitmap.c"
"Esd_Core/Esd_Render_Circle.c"
"Esd_Core/Esd_Render_Gradient.c"
"Esd_Core/Esd_Render_Rect.c"
"Esd_Core/Esd_Render__h__Simulation.c"
"Esd_Core/Esd_ResourceInfo.c"
"Esd_Core/Esd_Scissor.c"
"Esd_Core/Esd_Support.c"
"Esd_Core/Esd_TouchTag.c"
"Esd_Core/Esd_Utility.c"
"FT_Esd_Framework/Ft_Esd_CoCmd__h__Simulation.c"
"FT_Esd_Framework/Ft_Esd_IdleChecker__Generated.c"
"FT_Esd_Framework/Ft_Esd_Layout_AutoSwitch.c"
"FT_Esd_Framework/Ft_Esd_Layout_Display.c"
"FT_Esd_Framework/Ft_Esd_Layout_Fill.c"
"FT_Esd_Framework/Ft_Esd_Layout_Fixed.c"
"FT_Esd_Framework/Ft_Esd_Layout_Linear.c"
"FT_Esd_Framework/Ft_Esd_Layout_Scroll.c"
"FT_Esd_Framework/Ft_Esd_Layout_Stretch__Generated.c"
"FT_Esd_Framework/Ft_Esd_Layout_Switch.c"
"FT_Esd_Framework/Ft_Esd_Layout_Tag.c"
"FT_Esd_Framework/Ft_Esd_Layout_Theme.c"
"FT_Esd_Framework/Ft_Esd_Primitives__h__Simulation.c"
"FT_Esd_Framework/Ft_Esd_RenderForwarder__Generated.c"
"FT_Esd_Framework/Ft_Esd_Support.c"
"FT_Esd_Framework/Ft_Esd_Theme.c"
"FT_Esd_Framework/Ft_Esd_Timer.c"
"FT_Esd_Framework/Ft_Esd_Timer__Generated.c"
"FT_Esd_Framework/Ft_Esd_TouchArea.c"
"FT_Esd_Framework/Ft_Esd_TouchArea__Generated.c"
"FT_Esd_Framework/Ft_Esd_TouchScrollLogic.c"
"FT_Esd_Framework/Ft_Esd_TouchScrollLogic__Generated.c"
"FT_Esd_Framework/Ft_Esd_TouchTagRenderInterface.c"
"FT_Esd_Framework/Ft_Esd_Widget.c"
"FT_Esd_Framework/Ft_Esd_Widget_Utils.c"
"FT_Esd_Widgets/Ft_Esd_Animation.c"
"FT_Esd_Widgets/Ft_Esd_Animation__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ArcLine.c"
"FT_Esd_Widgets/Ft_Esd_ArcLine__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ArcSlider.c"
"FT_Esd_Widgets/Ft_Esd_ArcSlider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_CheckBox__Generated.c"
"FT_Esd_Widgets/Ft_Esd_CircleLine.c"
"FT_Esd_Widgets/Ft_Esd_CircleLine__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Circle_Raised__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Circle_Sunken__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Circle__Generated.c"
"FT_Esd_Widgets/Ft_Esd_CircularGradient.c"
"FT_Esd_Widgets/Ft_Esd_CircularGradientSlider.c"
"FT_Esd_Widgets/Ft_Esd_CircularGradientSlider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_CircularGradient__Generated.c"
"FT_Esd_Widgets/Ft_Esd_CircularSlider.c"
"FT_Esd_Widgets/Ft_Esd_CircularSlider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Clock__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ColorPicker.c"
"FT_Esd_Widgets/Ft_Esd_ColorPicker__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Elements.c"
"FT_Esd_Widgets/Ft_Esd_Elements__h__Simulation.c"
"FT_Esd_Widgets/Ft_Esd_FixedPointNumericLabel.c"
"FT_Esd_Widgets/Ft_Esd_FixedPointNumericLabel__Generated.c"
"FT_Esd_Widgets/Ft_Esd_FontIcon.c"
"FT_Esd_Widgets/Ft_Esd_FontIcon__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Gauge__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Gradient.c"
"FT_Esd_Widgets/Ft_Esd_Gradient_Arc_Line.c"
"FT_Esd_Widgets/Ft_Esd_Gradient_Arc_Line__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Gradient__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Image.c"
"FT_Esd_Widgets/Ft_Esd_ImageButton__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Image_Rotate.c"
"FT_Esd_Widgets/Ft_Esd_Image_Rotate__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Image_SlideShow__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Image__Generated.c"
"FT_Esd_Widgets/Ft_Esd_IntCounter__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Joypad.c"
"FT_Esd_Widgets/Ft_Esd_Joypad__Generated.c"
"FT_Esd_Widgets/Ft_Esd_LabelButton.c"
"FT_Esd_Widgets/Ft_Esd_LabelButton__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Label__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Layout_Dynamic_Switch.c"
"FT_Esd_Widgets/Ft_Esd_Line__Generated.c"
"FT_Esd_Widgets/Ft_Esd_LinearRoller.c"
"FT_Esd_Widgets/Ft_Esd_LinearRoller__Generated.c"
"FT_Esd_Widgets/Ft_Esd_MultiGradientRounded__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Numberpad.c"
"FT_Esd_Widgets/Ft_Esd_Numberpad__Generated.c"
"FT_Esd_Widgets/Ft_Esd_NumericLabel.c"
"FT_Esd_Widgets/Ft_Esd_NumericLabel__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Panel_Color__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Panel_Raised__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Panel_Sunken__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Panel__Generated.c"
"FT_Esd_Widgets/Ft_Esd_PartialRing__Generated.c"
"FT_Esd_Widgets/Ft_Esd_PolygonWidget.c"
"FT_Esd_Widgets/Ft_Esd_PolygonWidget__Generated.c"
"FT_Esd_Widgets/Ft_Esd_PosSizeToRadius__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ProgressBar.c"
"FT_Esd_Widgets/Ft_Esd_ProgressBar__Generated.c"
"FT_Esd_Widgets/Ft_Esd_PushButton__Generated.c"
"FT_Esd_Widgets/Ft_Esd_QRCode.c"
"FT_Esd_Widgets/Ft_Esd_QRCode__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RSSIBar.c"
"FT_Esd_Widgets/Ft_Esd_RSSIBar__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RadioButton.c"
"FT_Esd_Widgets/Ft_Esd_RadioButton__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RadioGroup.c"
"FT_Esd_Widgets/Ft_Esd_RadioGroup__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RangeSlider.c"
"FT_Esd_Widgets/Ft_Esd_RangeSliderInterval.c"
"FT_Esd_Widgets/Ft_Esd_RangeSliderInterval__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RangeSlider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Rectangle__Generated.c"
"FT_Esd_Widgets/Ft_Esd_RingSlider.c"
"FT_Esd_Widgets/Ft_Esd_RingSlider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Ring__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ScrollBar.c"
"FT_Esd_Widgets/Ft_Esd_ScrollBar__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ScrollPanel.c"
"FT_Esd_Widgets/Ft_Esd_ScrollPanel__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ScrollSwitch.c"
"FT_Esd_Widgets/Ft_Esd_ScrollSwitch__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ScrollableImageWidget__Generated.c"
"FT_Esd_Widgets/Ft_Esd_ScrollingText.c"
"FT_Esd_Widgets/Ft_Esd_ScrollingText__Generated.c"
"FT_Esd_Widgets/Ft_Esd_SignalSwitch.c"
"FT_Esd_Widgets/Ft_Esd_SignalSwitch__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Sketch.c"
"FT_Esd_Widgets/Ft_Esd_Sketch__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Slider.c"
"FT_Esd_Widgets/Ft_Esd_Slider__Generated.c"
"FT_Esd_Widgets/Ft_Esd_SlidingButton.c"
"FT_Esd_Widgets/Ft_Esd_SlidingButton__Generated.c"
"FT_Esd_Widgets/Ft_Esd_SpinBox__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Spinner__Generated.c"
"FT_Esd_Widgets/Ft_Esd_TextBox.c"
"FT_Esd_Widgets/Ft_Esd_TextBox__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Theme_DarkOrange.c"
"FT_Esd_Widgets/Ft_Esd_Theme_LightBlue.c"
"FT_Esd_Widgets/Ft_Esd_Toggle.c"
"FT_Esd_Widgets/Ft_Esd_Toggle__Generated.c"
"FT_Esd_Widgets/Ft_Esd_TouchPanel__Generated.c"
"FT_Esd_Widgets/Ft_Esd_Video.c"
"FT_Esd_Widgets/Ft_Esd_Video__Generated.c"
"FT_Esd_Widgets/ThirdPartylibs/qrcodegen.c"
"FT_Eve_Hal/EVE_Cmd.c"
"FT_Eve_Hal/EVE_CoCmd.c"
"FT_Eve_Hal/EVE_CoCmd_IO.c"
"FT_Eve_Hal/EVE_CoCmd_State.c"
"FT_Eve_Hal/EVE_CoCmd_Widgets.c"
"FT_Eve_Hal/EVE_CoCmd__h__Simulation.c"
"FT_Eve_Hal/EVE_CoDl.c"
"FT_Eve_Hal/EVE_GpuDefs_BT81X.c"
"FT_Eve_Hal/EVE_GpuDefs_FT80X.c"
"FT_Eve_Hal/EVE_GpuDefs_FT81X.c"
"FT_Eve_Hal/EVE_Hal.c"
"FT_Eve_Hal/EVE_HalImpl_BT8XXEMU.c"
"FT_Eve_Hal/EVE_HalImpl_BT8XXEMUC.c"
"FT_Eve_Hal/EVE_HalImpl_FT4222.c"
"FT_Eve_Hal/EVE_HalImpl_FT9XX.c"
"FT_Eve_Hal/EVE_HalImpl_MPSSE.c"
"FT_Eve_Hal/EVE_HalImpl_RP2040.c"
"FT_Eve_Hal/EVE_HalImpl_STM32.c"
"FT_Eve_Hal/EVE_HalImpl_WIN32.c"
"FT_Eve_Hal/EVE_IDM2040_21R_RP2040.c"
"FT_Eve_Hal/EVE_IDP_4000_04A_RP2040.c"
"FT_Eve_Hal/EVE_ILI9488_FT9XX.c"
"FT_Eve_Hal/EVE_ILI9488_RP2040.c"
"FT_Eve_Hal/EVE_KD2401.c"
"FT_Eve_Hal/EVE_LoadFile_FATFS.c"
"FT_Eve_Hal/EVE_LoadFile_STDIO.c"
"FT_Eve_Hal/EVE_MediaFifo.c"
"FT_Eve_Hal/EVE_Util.c"
"ImageSlide/AppScreen__Generated.c"
"ImageSlide/App__Generated.c"
"ImageSlide/DefaultTheme.c"
"ImageSlide/FirstPage__Generated.c"
"ImageSlide/Usr_Page__Generated.c"
"ImageSlide/barbecue.c"
"ImageSlide/chess.c"
"ImageSlide/diveevo.c"
"ImageSlide/fire.c"
"ImageSlide/keyboard.c"
"ImageSlide/locomotive.c"
"ImageSlide/person.c"
"ImageSlide/rose.c"
"ImageSlide/seagull.c"
"ThirdPartyLib/ff14a/ff.c"
"ThirdPartyLib/ff14a/ffsystem.c"
"ThirdPartyLib/ff14a/ffunicode.c"
"ThirdPartyLib/sd_driver/crc.c"
"ThirdPartyLib/sd_driver/demo_logging.c"
"ThirdPartyLib/sd_driver/hw_config.c"
"ThirdPartyLib/sd_driver/my_debug.c"
"ThirdPartyLib/sd_driver/sd_card.c"
"ThirdPartyLib/sd_driver/sd_spi.c"
"ThirdPartyLib/sd_driver/spi.c"
"ThirdPartyLib/src/f_util.c"
"ThirdPartyLib/src/ff_stdio.c"
"ThirdPartyLib/src/glue.c"
"ThirdPartyLib/src/my_debug.c"
"ThirdPartyLib/src/rtc.c"
)

add_definitions(
-DNDEBUG
-DEVE_GRAPHICS_IDP_1000_04A
-DEVE_DISPLAY_WXGA
-DEVE_PLATFORM_RP2040
-DEVE_TOUCH_FOCAL
-DEVE_SPICHANNEL_QUAD
)

pico_set_program_name(ImageSlide "ImageSlide")

pico_enable_stdio_uart(ImageSlide 0)
pico_enable_stdio_usb(ImageSlide 1)

# Add the standard library to the build
target_link_libraries(ImageSlide pico_stdlib)

# Add any user requested libraries
target_link_libraries(ImageSlide
        hardware_spi
        hardware_timer
		hardware_dma
		hardware_rtc
        )

pico_add_extra_outputs(ImageSlide)

add_custom_command(
        TARGET ImageSlide POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/ImageSlide.uf2
                ${CMAKE_SOURCE_DIR}/Bin/ImageSlide.uf2)

