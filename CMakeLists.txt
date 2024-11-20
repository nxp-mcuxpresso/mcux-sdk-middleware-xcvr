# Copyright 2024 NXP
# SPDX-License-Identifier: BSD-3-Clause

if(CONFIG_MCUX_COMPONENT_middleware.wireless.xcvr)

    # add other CmakeLists files if you need to
    # mcux_add_cmakelists(${PATH})

    # add source files and header file - header file are required for IAR project generation
    mcux_add_source(
        SOURCES 
            drv/nb2p4ghz/nxp2p4_xcvr.c
            drv/nb2p4ghz/nxp2p4_xcvr.h
            drv/nb2p4ghz/nxp_xcvr_trim.c
            drv/nb2p4ghz/nxp_xcvr_trim.h
            drv/rfmc_ctrl.c
            drv/rfmc_ctrl.h
            drv/nxp_xcvr_lcl_ctrl.c
            drv/nxp_xcvr_lcl_ctrl.h
            drv/nxp_xcvr_ext_ctrl.c
            drv/nxp_xcvr_ext_ctrl.h
            drv/dbg_ram_capture.c
            drv/dbg_ram_capture.h
            drv/dma_capture.c
            drv/dma_capture.h
            drv/mathfp.c
            drv/mathfp.h
    )

    # add include path
    mcux_add_include(
        INCLUDES 
            drv/.
            drv/nb2p4ghz/.
    )

    if(CONFIG_MCUX_COMPONENT_middleware.wireless.xcvr.kw45_k32w1_mcxw71)
        # add sources and include path for families kw45_k32w1_mcxw71
        mcux_add_source(
            SOURCES 
                drv/nb2p4ghz/configs/gen45/*.c
                drv/nb2p4ghz/configs/gen45/*.h
                drv/nxp_xcvr_lcl_step_mgr.h
                drv/nxp_xcvr_lcl_step_structs.h
        )
        mcux_add_include(
            INCLUDES 
                drv/nb2p4ghz/configs/gen45/.
        )
    endif()

    if(CONFIG_MCUX_COMPONENT_middleware.wireless.xcvr.kw47_mcxw72)
        # add sources and include path for families kw47_mcxw72
        mcux_add_source(
            SOURCES 
                drv/nb2p4ghz/configs/gen47/*.c
                drv/nb2p4ghz/configs/gen47/*.h
				drv/nxp_xcvr_lcl_step_mgr.c
				drv/nxp_xcvr_lcl_step_mgr.h
				drv/nxp_xcvr_lcl_step_structs.h
        )
        mcux_add_include(
            INCLUDES 
                drv/nb2p4ghz/configs/gen47/.
        )
    endif()

endif()

