//#ifdef CONFIG_A500CG
#define HW_ID_EVB	0x00000007
#define HW_ID_SR	0x00000001
#define HW_ID_ER	0x00000003
#define HW_ID_PR	0x00000006
#define HW_ID_MP	0x00000005

#define LCD_ID_TM	0
#define LCD_ID_CTC	0
#define LCD_ID_HSD	1

#define A502CG_LCD_ID_GIS				0
#define A502CG_LCD_ID_TM				1
#define A502CG_LCD_ID_OFILM			2
#define A502CG_LCD_ID_TXD			3

#define PROJ_ID_A500CG  				0x00000000
//Indonesia = ID
#define PROJ_ID_A500CG_ID               0x00000001
#define PROJ_ID_A501CG_ID               0x00000002
#define PROJ_ID_A501CG_BZ  				0x00000003
#define PROJ_ID_A501CG  				0x00000004
#define PROJ_ID_A502CG  				0x0000000d
#define PROJ_ID_A600CG  				0x00000007
#define PROJ_ID_A601CG  				0x00000005
/*
#else // CONFIG_A500CG

#ifdef CONFIG_PF450CL
#define HW_ID_EVB			0x00000001
#define HW_ID_SR1			0x00000002
#define HW_ID_SR2                       0x00000000
#define HW_ID_ER			0x00000003
#define HW_ID_ER2                       0x00000004
#define HW_ID_PR			0x00000005
#define HW_ID_MP			0x00000006
#define HW_ID_MP2                       0x00000007

#elif defined(CONFIG_ME372CL)
#define HW_ID_EVB			0x00000000
#define HW_ID_SR1			0x00000000
#define HW_ID_SR2			0x00000004
#define HW_ID_ER			0x00000002
#define HW_ID_PR			0x00000006
#define HW_ID_MP			0x00000001
#define HW_ID_ER2                       0x00000007
#define HW_ID_MP2                       0x00000005

#else
#define HW_ID_EVB			0x00000001
#define HW_ID_SR1			0x00000002
#define HW_ID_SR2			0x00000004
#define HW_ID_ER			0x00000003
#define HW_ID_ER2                       0x00000005
#define HW_ID_PR			0x00000006
#define HW_ID_MP			0x00000007
#define HW_ID_MP2                       0x00000008
#endif

#define PROJ_ID_PF450CL                 0x00000000
#define PROJ_ID_ME302C                  0x00000000
#define PROJ_ID_ME372CG                 0x00000002
#define PROJ_ID_ME372CL                 0x00000004
#define PROJ_ID_GEMINI                  0x00000001
#define PROJ_ID_ME175CG                 0x00000006
#define PROJ_ID_PF400CG                 0x00000005
#define PROJ_ID_ME372CG_CD              0x00000007
#define PROJ_ID_A400CG                  0x00000003
#define PROJ_ID_FE380CG                 0x00000038
#define PROJ_ID_A450CG                  0x00000039

#endif*/ /* CONFIG_A500CG */
