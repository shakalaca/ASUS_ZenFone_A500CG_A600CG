/*************************************************************************/ /*!
@File
@Title          Rgx debug information
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX debugging functions
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/
//#define PVR_DPF_FUNCTION_TRACE_ON 1
#undef PVR_DPF_FUNCTION_TRACE_ON

#include "rgxdefs_km.h"
#include "rgxdevice.h"
#include "allocmem.h"
#include "osfunc.h"

#include "lists.h"

#include "rgxdebug.h"
#include "pvrversion.h"
#include "pvr_debug.h"
#include "srvkm.h"
#include "rgxutils.h"
#include "tlstream.h"
#include "rgxfwutils.h"
#include "pvrsrv.h"

#include "devicemem_pdump.h"

#include "rgx_fwif.h"
#include "pvrsrv.h"

#define RGX_DEBUG_STR_SIZE	(150)

IMG_CHAR* pszPowStateName [] = {
#define X(NAME)	#NAME,
	RGXFWIF_POW_STATES
#undef X
};


extern IMG_UINT32 g_ui32HostSampleIRQCount;

/*!
*******************************************************************************

 @Function	_RGXDecodePMPC

 @Description

 Return the name for the PM managed Page Catalogues

 @Input ui32PC	 - Page Catalogue number

 @Return   IMG_VOID

******************************************************************************/
static IMG_CHAR* _RGXDecodePMPC(IMG_UINT32 ui32PC)
{
	IMG_CHAR* pszPMPC = " (-)";

	switch (ui32PC)
	{
		case 0x8: pszPMPC = " (PM-VCE0)"; break;
		case 0x9: pszPMPC = " (PM-TE0)"; break;
		case 0xA: pszPMPC = " (PM-ZLS0)"; break;
		case 0xB: pszPMPC = " (PM-ALIST0)"; break;
		case 0xC: pszPMPC = " (PM-VCE1)"; break;
		case 0xD: pszPMPC = " (PM-TE1)"; break;
		case 0xE: pszPMPC = " (PM-ZLS1)"; break;
		case 0xF: pszPMPC = " (PM-ALIST1)"; break;
	}

	return pszPMPC;
}

/*!
*******************************************************************************

 @Function	_RGXDecodeBIFReqTags

 @Description

 Decode the BIF Tag ID and SideBand data fields from BIF_FAULT_BANK_REQ_STATUS regs

 @Input ui32TagID           - Tag ID value
 @Input ui32TagSB           - Tag Sideband data
 @Output ppszTagID          - Decoded string from the Tag ID
 @Output ppszTagSB          - Decoded string from the Tag SB
 @Output pszScratchBuf      - Buffer provided to the function to generate the debug strings
 @Input ui32ScratchBufSize  - Size of the provided buffer

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDecodeBIFReqTags(IMG_UINT32		ui32TagID, 
									 IMG_UINT32		ui32TagSB, 
									 IMG_CHAR		**ppszTagID, 
									 IMG_CHAR		**ppszTagSB,
									 IMG_CHAR		*pszScratchBuf,
									 IMG_UINT32		ui32ScratchBufSize)
{
	/* default to unknown */
	IMG_CHAR *pszTagID = "-";
	IMG_CHAR *pszTagSB = "-";

	switch (ui32TagID)
	{
		case 0x0:
		{
			pszTagID = "MMU";
			break;
		}
		case 0x1:
		{
			pszTagID = "TLA";
			break;
		}
		case 0x2:
		{
			pszTagID = "HOST";
			break;
		}
		case 0x3:
		{
			pszTagID = "META";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "dcache"; break;
				case 0x1: pszTagSB = "icache"; break;
				case 0x2: pszTagSB = "jtag"; break;
				case 0x3: pszTagSB = "slave bus"; break;
			}
			break;
		}
		case 0x4:
		{
			pszTagID = "USC";
			break;
		}
		case 0x5:
		{
			pszTagID = "PBE";
			break;
		}
		case 0x6:
		{
			pszTagID = "ISP";
			break;
		}
		case 0x7:
		{
			pszTagID = "IPF";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Macrotile Header"; break;
				case 0x1: pszTagSB = "Region Header"; break;
				case 0x2: pszTagSB = "DBSC"; break;
				case 0x3: pszTagSB = "CPF"; break;
				case 0x4: 
				case 0x6:
				case 0x8: pszTagSB = "Control Stream"; break;
				case 0x5: 
				case 0x7:
				case 0x9: pszTagSB = "Primitive Block"; break;
			}
			break;
		}
		case 0x8:
		{
			pszTagID = "CDM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Control Stream"; break;
				case 0x1: pszTagSB = "Indirect Data"; break;
				case 0x2: pszTagSB = "Event Write"; break;
				case 0x3: pszTagSB = "Context State"; break;
			}
			break;
		}
		case 0x9:
		{
			pszTagID = "VDM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "Control Stream"; break;
				case 0x1: pszTagSB = "PPP State"; break;
				case 0x2: pszTagSB = "Index Data"; break;
				case 0x4: pszTagSB = "Call Stack"; break;
				case 0x8: pszTagSB = "Context State"; break;
			}
			break;
		}
		case 0xA:
		{
			pszTagID = "PM";
			switch (ui32TagSB)
			{
				case 0x0: pszTagSB = "PMA_TAFSTACK"; break;
				case 0x1: pszTagSB = "PMA_TAMLIST"; break;
				case 0x2: pszTagSB = "PMA_3DFSTACK"; break;
				case 0x3: pszTagSB = "PMA_3DMLIST"; break;
				case 0x4: pszTagSB = "PMA_PMCTX0"; break;
				case 0x5: pszTagSB = "PMA_PMCTX1"; break;
				case 0x6: pszTagSB = "PMA_MAVP"; break;
				case 0x7: pszTagSB = "PMA_UFSTACK"; break;
				case 0x8: pszTagSB = "PMD_TAFSTACK"; break;
				case 0x9: pszTagSB = "PMD_TAMLIST"; break;
				case 0xA: pszTagSB = "PMD_3DFSTACK"; break;
				case 0xB: pszTagSB = "PMD_3DMLIST"; break;
				case 0xC: pszTagSB = "PMD_PMCTX0"; break;
				case 0xD: pszTagSB = "PMD_PMCTX1"; break;
				case 0xF: pszTagSB = "PMD_UFSTACK"; break;
				case 0x10: pszTagSB = "PMA_TAMMUSTACK"; break;
				case 0x11: pszTagSB = "PMA_3DMMUSTACK"; break;
				case 0x12: pszTagSB = "PMD_TAMMUSTACK"; break;
				case 0x13: pszTagSB = "PMD_3DMMUSTACK"; break;
				case 0x14: pszTagSB = "PMA_TAUFSTACK"; break;
				case 0x15: pszTagSB = "PMA_3DUFSTACK"; break;
				case 0x16: pszTagSB = "PMD_TAUFSTACK"; break;
				case 0x17: pszTagSB = "PMD_3DUFSTACK"; break;
				case 0x18: pszTagSB = "PMA_TAVFP"; break;
				case 0x19: pszTagSB = "PMD_3DVFP"; break;
				case 0x1A: pszTagSB = "PMD_TAVFP"; break;
			}
			break;
		}
		case 0xB:
		{
			pszTagID = "TA";
			switch (ui32TagSB)
			{
				case 0x1: pszTagSB = "VCE"; break;
				case 0x2: pszTagSB = "TPC"; break;
				case 0x3: pszTagSB = "TE Control Stream"; break;
				case 0x4: pszTagSB = "TE Region Header"; break;
				case 0x5: pszTagSB = "TE Render Target Cache"; break;
				case 0x6: pszTagSB = "TEAC Render Target Cache"; break;
				case 0x7: pszTagSB = "VCE Render Target Cache"; break;
			}
			break;
		}
		case 0xC:
		{
			pszTagID = "TPF";
			break;
		}
		case 0xD:
		{
			pszTagID = "PDS";
			break;
		}
		case 0xE:
		{
			pszTagID = "MCU";
			{
				IMG_UINT32 ui32Burst = (ui32TagSB >> 5) & 0x7;
				IMG_UINT32 ui32GroupEnc = (ui32TagSB >> 2) & 0x7;
				IMG_UINT32 ui32Group = ui32TagSB & 0x2;

				IMG_CHAR* pszBurst = "";
				IMG_CHAR* pszGroupEnc = "";
				IMG_CHAR* pszGroup = "";

				switch (ui32Burst)
				{
					case 0x4: pszBurst = "Lower 256bits"; break;
					case 0x5: pszBurst = "Upper 256bits"; break;
					case 0x6: pszBurst = "512 bits"; break;
					default:  pszBurst = (ui32Burst & 0x2)?
								"128bit word within the Lower 256bits":
								"128bit word within the Upper 256bits"; break;
				}
				switch (ui32GroupEnc)
				{
					case 0x0: pszGroupEnc = "TPUA_USC"; break;
					case 0x1: pszGroupEnc = "TPUB_USC"; break;
					case 0x2: pszGroupEnc = "USCA_USC"; break;
					case 0x3: pszGroupEnc = "USCB_USC"; break;
					case 0x4: pszGroupEnc = "PDS_USC"; break;
#if (RGX_BVNC_KM_N < 6)
					case 0x5: pszGroupEnc = "PDSRW"; break;
#elif (RGX_BVNC_KM_N == 6)
					case 0x5: pszGroupEnc = "UPUC_USC"; break;
					case 0x6: pszGroupEnc = "TPUC_USC"; break;
					case 0x7: pszGroupEnc = "PDSRW"; break;
#endif
				}
				switch (ui32Group)
				{
					case 0x0: pszGroup = "Banks 0-3"; break;
					case 0x1: pszGroup = "Banks 4-7"; break;
					case 0x2: pszGroup = "Banks 8-11"; break;
					case 0x3: pszGroup = "Banks 12-15"; break;
				}

				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
								"%s, %s, %s", pszBurst, pszGroupEnc, pszGroup);
				pszTagSB = pszScratchBuf;
			}
			break;
		}
		case 0xF:
		{
			pszTagID = "FB_CDC";
			{
				IMG_UINT32 ui32Req = (ui32TagSB >> 2) & 0x3;
				IMG_UINT32 ui32MCUSB = ui32TagSB & 0x3;

				IMG_CHAR* pszReqId = (ui32TagSB & 0x10)?"FBDC":"FBC";
				IMG_CHAR* pszOrig = "";

				switch (ui32Req)
				{
					case 0x0: pszOrig = "ZLS"; break;
					case 0x1: pszOrig = (ui32TagSB & 0x10)?"MCU":"PBE"; break;
					case 0x2: pszOrig = "Host"; break;
					case 0x3: pszOrig = "TLA"; break;
				}
				OSSNPrintf(pszScratchBuf, ui32ScratchBufSize,
							"%s Request, originator %s, MCU sideband 0x%X",
							pszReqId, pszOrig, ui32MCUSB);
				pszTagSB = pszScratchBuf;
			}
			break;
		}
	} /* switch(TagID) */

	*ppszTagID = pszTagID;
	*ppszTagSB = pszTagSB;
}

/*!
*******************************************************************************

 @Function	_RGXDumpRGXBIFBank

 @Description

 Dump BIF Bank state in human readable form.

 @Input psDevInfo				- RGX device info
 @Input ui32BankID	 			- BIF Bank identification number
 @Input ui32MMUStatusRegAddr	- MMU Status register address
 @Input ui32ReqStatusRegAddr	- BIF request Status register address

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpRGXBIFBank(	PVRSRV_RGXDEV_INFO	*psDevInfo, 
									IMG_UINT32			ui32BankID, 
									IMG_UINT32			ui32MMUStatusRegAddr,
									IMG_UINT32			ui32ReqStatusRegAddr)
{
	IMG_UINT64	ui64RegVal;

	ui64RegVal = OSReadHWReg64(psDevInfo->pvRegsBaseKM, ui32MMUStatusRegAddr);

	if (ui64RegVal == 0x0)
	{
		PVR_LOG(("BIF%d - OK", ui32BankID));
	}
	else
	{
		/* Bank 0 & 1 share the same fields */
		PVR_LOG(("BIF%d - FAULT:", ui32BankID));

		/* MMU Status */
		{
			IMG_UINT32 ui32PC = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_SHIFT;

			IMG_UINT32 ui32PageSize = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_PAGE_SIZE_SHIFT;

			IMG_UINT32 ui32MMUDataType = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_SHIFT;

			IMG_BOOL bROFault = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_RO_EN) != 0;
			IMG_BOOL bProtFault = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_PM_META_RO_EN) != 0;

			PVR_LOG(("  * MMU status (0x%016llX): PC = %d%s, Page Size = %d, MMU data type = %d%s%s.",
						ui64RegVal,
						ui32PC,
						(ui32PC < 0x8)?"":_RGXDecodePMPC(ui32PC),
						ui32PageSize,
						ui32MMUDataType,
						(bROFault)?", Read Only fault":"",
						(bProtFault)?", PM/META protection fault":""));
		}

		/* Req Status */
		ui64RegVal = OSReadHWReg64(psDevInfo->pvRegsBaseKM, ui32ReqStatusRegAddr);
		{
			IMG_CHAR *pszTagID;
			IMG_CHAR *pszTagSB;
			IMG_CHAR aszScratch[RGX_DEBUG_STR_SIZE];

			IMG_BOOL bRead = (ui64RegVal & RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_RNW_EN) != 0;
			IMG_UINT32 ui32TagSB = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_CLRMSK) >>
					RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_SHIFT;
			IMG_UINT32 ui32TagID = 
				(ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_CLRMSK) >>
							RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_SHIFT;
			IMG_UINT64 ui64Addr = (ui64RegVal & ~RGX_CR_BIF_FAULT_BANK0_REQ_STATUS_ADDRESS_CLRMSK);

			_RGXDecodeBIFReqTags(ui32TagID, ui32TagSB, &pszTagID, &pszTagSB, &aszScratch[0], RGX_DEBUG_STR_SIZE);

			PVR_LOG(("  * Request (0x%016llX): %s (%s), %s 0x%010llX.",
						ui64RegVal,
						pszTagID,
						pszTagSB,
						(bRead)?"Reading from":"Writing to",
						ui64Addr));
		}
	}

}


/*!
*******************************************************************************

 @Function	_RGXDumpFWAssert

 @Description

 Dump FW assert strings when a thread asserts.

 @Input psRGXFWIfTraceBufCtl	- RGX FW trace buffer

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpFWAssert(RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl)
{
	IMG_CHAR    *pszTraceAssertPath;
	IMG_CHAR    *pszTraceAssertInfo;
	IMG_INT32   ui32TraceAssertLine;
	IMG_UINT32  i;

	for (i = 0; i < RGXFW_THREAD_NUM; i++)
	{
		pszTraceAssertPath = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.szPath;
		pszTraceAssertInfo = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.szInfo;
		ui32TraceAssertLine = psRGXFWIfTraceBufCtl->sTraceBuf[i].sAssertBuf.ui32LineNum;

		/* print non null assert strings */
		if (*pszTraceAssertInfo)
		{
			PVR_LOG(("FW-T%d Assert: %s (%s:%d)", 
						i, pszTraceAssertInfo, pszTraceAssertPath, ui32TraceAssertLine));
		}
	}
}

static IMG_VOID _RGXDumpFWPoll(RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl)
{
	IMG_UINT32 i;
	for (i = 0; i < RGXFW_THREAD_NUM; i++)
	{
		if (psRGXFWIfTraceBufCtl->aui32CrPollAddr[i])
		{
			PVR_LOG(("T%u polling %s (reg:0x%08X val:0x%08X)",
						i,
						((psRGXFWIfTraceBufCtl->aui32CrPollAddr[i] & RGXFW_POLL_TYPE_SET)?("set"):("unset")), 
						psRGXFWIfTraceBufCtl->aui32CrPollAddr[i] & ~RGXFW_POLL_TYPE_SET, 
						psRGXFWIfTraceBufCtl->aui32CrPollValue[i]));
		}
	}

}

static IMG_VOID _RGXDumpFWHWRInfo(RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl, RGXFWIF_HWRINFOBUF *psHWInfoBuf)
{
	IMG_BOOL        bAnyLocked = IMG_FALSE;
	IMG_UINT32      dm, i;
	IMG_CHAR	    *pszLine, *pszTemp;
	IMG_UINT32      ui32LineSize;
	const IMG_CHAR * apszDmNames[RGXFWIF_DM_MAX + 1] = { "GP(", "2D(", "TA(", "3D(", "CDM(",
#if defined(RGX_FEATURE_RAY_TRACING)
								 "RTU(", "SHG(",
#endif /* RGX_FEATURE_RAY_TRACING */
								 NULL };

	const IMG_CHAR* pszMsgHeader = "Number of HWR: ";
	IMG_UINT32      ui32MsgHeaderSize = OSStringLength(pszMsgHeader);

	for (dm = 0; dm < RGXFWIF_DM_MAX; dm++)
	{
		if (psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[dm])
		{
			bAnyLocked = IMG_TRUE;
			break;					
		}
	}

	if (!bAnyLocked && (psRGXFWIfTraceBufCtl->ui32HWRStateFlags & RGXFWIF_HWR_HARDWARE_OK))
	{
		/* No HWR situation, print nothing */
		return;
	}

	ui32LineSize = sizeof(IMG_CHAR) * (	ui32MsgHeaderSize + 
			(RGXFWIF_DM_MAX*(	4/*DM name + left parenthesis*/ + 
								5/*UINT16 max num of digits*/ + 
								1/*slash*/ + 
								5/*UINT16 max num of digits*/ + 
								3/*right parenthesis + comma + space*/)) + 
			1/* \0 */);

	pszLine = OSAllocMem(ui32LineSize);
	if (pszLine == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"_RGXDumpRGXDebugSummary: Out of mem allocating line string (size: %d)", ui32LineSize));
		return;
	}

	OSStringCopy(pszLine,pszMsgHeader);
	pszTemp = pszLine + ui32MsgHeaderSize;

	for (dm = 0; (dm < RGXFWIF_DM_MAX) && (apszDmNames[dm] != IMG_NULL); dm++)
	{
		OSStringCopy(pszTemp,apszDmNames[dm]);
		pszTemp += OSStringLength(apszDmNames[dm]);
		pszTemp += OSSNPrintf(pszTemp, 
				5 + 1 + 5 + 1 + 1 + 1 + 1 /* UINT16 + slash + UINT16 + right parenthesis + comma + space + \0 */,
				"%u/%u), ",
				psRGXFWIfTraceBufCtl->aui16HwrDmRecoveredCount[dm],
				psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[dm]);
	}

	PVR_LOG((pszLine));

	OSFreeMem(pszLine);

	/* Print out per HWR info */
	for (dm = 0; (dm < RGXFWIF_DM_MAX) && (apszDmNames[dm] != IMG_NULL); dm++)
	{
		if (dm == RGXFWIF_DM_GP)
		{
			PVR_LOG(("DM %d (GP)", dm));
		}
		else
		{
			PVR_LOG(("DM %d (HWRflags 0x%08x)", dm, psRGXFWIfTraceBufCtl->aui32HWRRecoveryFlags[dm]));
		}

		for (i=0;i<RGXFWIF_MAX_HWINFO;i++)
		{
			IMG_UINT32 ui32Offset = (psHWInfoBuf->ui32WriteIndex[dm] + i) % RGXFWIF_MAX_HWINFO;
			RGX_HWRINFO *psHWRInfo = &psHWInfoBuf->sHWRInfo[dm][ui32Offset];

			if (psHWRInfo->ui32HWRNumber != 0) 
			{
				PVR_LOG(("Recovery %d: PID = %d, frame = %d, HWRTData = 0x%08X",
							psHWRInfo->ui32HWRNumber,
							psHWRInfo->ui32PID,
							psHWRInfo->ui32FrameNum,
							psHWRInfo->ui32ActiveHWRTData));
			}
		}
	}	

}

/*!
*******************************************************************************

 @Function	_RGXDumpRGXDebugSummary

 @Description

 Dump a summary in human readable form with the RGX state

 @Input psDevInfo	 - RGX device info

 @Return   IMG_VOID

******************************************************************************/
static IMG_VOID _RGXDumpRGXDebugSummary(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_BOOL bRGXPoweredON)
{
	IMG_CHAR *pszState;
	RGXFWIF_TRACEBUF *psRGXFWIfTraceBuf = psDevInfo->psRGXFWIfTraceBuf;

	if (bRGXPoweredON)
	{
		_RGXDumpRGXBIFBank(psDevInfo, 0, 
				RGX_CR_BIF_FAULT_BANK0_MMU_STATUS, 
				RGX_CR_BIF_FAULT_BANK0_REQ_STATUS);

		_RGXDumpRGXBIFBank(psDevInfo, 1, 
				RGX_CR_BIF_FAULT_BANK1_MMU_STATUS, 
				RGX_CR_BIF_FAULT_BANK1_REQ_STATUS);
	}

	/* Firmware state */
	switch (psDevInfo->psDeviceNode->eHealthStatus)
	{
		case PVRSRV_DEVICE_HEALTH_STATUS_OK:
		{
			pszState = "OK";
			break;
		}
		
		case PVRSRV_DEVICE_HEALTH_STATUS_DEAD:
		{
			pszState = "DEAD";
			break;
		}
		
		default:
		{
			pszState = "UNKNOWN";
			break;
		}
	}

	if (psRGXFWIfTraceBuf == IMG_NULL)
	{
		PVR_LOG(("RGX FW State: %s", pszState));

		/* can't dump any more information */
		return;
	}
	
	PVR_LOG(("RGX FW State: %s (HWRState 0x%08x)", pszState, psRGXFWIfTraceBuf->ui32HWRStateFlags));
	PVR_LOG(("RGX FW Power State: %s (APM %s: %d ok, %d denied, %d other, %d total)", 
	     pszPowStateName[psRGXFWIfTraceBuf->ePowState],
		 (psDevInfo->pfnActivePowerCheck)?"enabled":"disabled",
		 psDevInfo->ui32ActivePMReqOk,
		 psDevInfo->ui32ActivePMReqDenied,
		 psDevInfo->ui32ActivePMReqTotal - psDevInfo->ui32ActivePMReqOk - psDevInfo->ui32ActivePMReqDenied,
		 psDevInfo->ui32ActivePMReqTotal));


	_RGXDumpFWAssert(psRGXFWIfTraceBuf);

	_RGXDumpFWPoll(psRGXFWIfTraceBuf);

	_RGXDumpFWHWRInfo(psRGXFWIfTraceBuf, psDevInfo->psRGXFWIfHWRInfoBuf);

}

static IMG_VOID _RGXDumpMetaSPExtraDebugInfo(PVRSRV_RGXDEV_INFO *psDevInfo)
{
/* List of extra META Slave Port debug registers */
#define RGX_META_SP_EXTRA_DEBUG \
			X(RGX_CR_META_SP_MSLVCTRL0) \
			X(RGX_CR_META_SP_MSLVCTRL1) \
			X(RGX_CR_META_SP_MSLVIRQSTATUS) \
			X(RGX_CR_META_SP_MSLVIRQENABLE) \
			X(RGX_CR_META_SP_MSLVIRQLEVEL)

	IMG_UINT32 ui32Idx, ui32RegIdx;
	IMG_UINT32 ui32RegVal;
	IMG_UINT32 ui32RegAddr;

	const IMG_UINT32 aui32DebugRegAddr [] = {
#define X(A) A,
		RGX_META_SP_EXTRA_DEBUG
#undef X
		};

	const IMG_CHAR* apszDebugRegName [] = {
#define X(A) #A,
	RGX_META_SP_EXTRA_DEBUG
#undef X
	};
	
	const IMG_UINT32 aui32Debug2RegAddr [] = {0xA28, 0x0A30, 0x0A38};

	PVR_LOG(("META Slave Port extra debug:"));

	/* dump first set of Slave Port debug registers */
	for (ui32Idx = 0; ui32Idx < sizeof(aui32DebugRegAddr)/sizeof(IMG_UINT32); ui32Idx++)
	{
		const IMG_CHAR* pszRegName = apszDebugRegName[ui32Idx];

		ui32RegAddr = aui32DebugRegAddr[ui32Idx];
		ui32RegVal = OSReadHWReg32(psDevInfo->pvRegsBaseKM, ui32RegAddr);
		PVR_LOG(("  * %s: 0x%8.8X", pszRegName, ui32RegVal));
	}

	/* dump second set of Slave Port debug registers */
	for (ui32Idx = 0; ui32Idx < 4; ui32Idx++)
	{
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, 0xA20, ui32Idx);
		ui32RegVal = OSReadHWReg32(psDevInfo->pvRegsBaseKM, 0xA20);
		PVR_LOG(("  * 0xA20[%d]: 0x%8.8X", ui32Idx, ui32RegVal));

	}

	for (ui32RegIdx = 0; ui32RegIdx < sizeof(aui32Debug2RegAddr)/sizeof(IMG_UINT32); ui32RegIdx++)
	{
		ui32RegAddr = aui32Debug2RegAddr[ui32RegIdx];
		for (ui32Idx = 0; ui32Idx < 2; ui32Idx++)
		{
			OSWriteHWReg32(psDevInfo->pvRegsBaseKM, ui32RegAddr, ui32Idx);
			ui32RegVal = OSReadHWReg32(psDevInfo->pvRegsBaseKM, ui32RegAddr);
			PVR_LOG(("  * 0x%X[%d]: 0x%8.8X", ui32RegAddr, ui32Idx, ui32RegVal));
		}
	}

}

/*
	RGXDumpDebugInfo
*/
IMG_VOID RGXDumpDebugInfo (PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	IMG_UINT32 i;

	for(i=0;i<DEBUG_REQUEST_VERBOSITY_MAX;i++)
	{
		RGXDebugRequestProcess(psDevInfo, i);
	}
}

static IMG_CHAR* _RGXGetDebugDevPowerStateString(PVRSRV_DEV_POWER_STATE ePowerState)
{
	switch(ePowerState)
	{
		case PVRSRV_DEV_POWER_STATE_DEFAULT: return "DEFAULT";
		case PVRSRV_DEV_POWER_STATE_OFF: return "OFF";
		case PVRSRV_DEV_POWER_STATE_ON: return "ON";
		default: return "UNKNOWN";
	}
}

IMG_VOID RGXDebugRequestProcess(PVRSRV_RGXDEV_INFO	*psDevInfo,
							    IMG_UINT32			ui32VerbLevel)
{

	switch (ui32VerbLevel)
	{
		case DEBUG_REQUEST_VERBOSITY_LOW :
		{
			PVRSRV_ERROR            eError;
			IMG_UINT32              ui32DeviceIndex;
			PVRSRV_DEV_POWER_STATE  ePowerState;
			IMG_BOOL                bRGXPoweredON;

			ui32DeviceIndex = psDevInfo->psDeviceNode->sDevId.ui32DeviceIndex;

			eError = PVRSRVGetDevicePowerState(ui32DeviceIndex, &ePowerState);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "RGXDebugRequestProcess: Error retrieving RGX power state. No debug info dumped."));
				return;
			}

			bRGXPoweredON = (ePowerState == PVRSRV_DEV_POWER_STATE_ON);

			PVR_LOG(("------[ RGX summary ]------"));
			PVR_LOG(("RGX BVNC: %s", RGX_BVNC_KM));
			PVR_LOG(("RGX Power State: %s", _RGXGetDebugDevPowerStateString(ePowerState)));

			_RGXDumpRGXDebugSummary(psDevInfo, bRGXPoweredON);

			if (bRGXPoweredON)
			{

				PVR_LOG(("------[ RGX registers ]------"));
				PVR_LOG(("RGX Register Base Address (Linear):   0x%p", psDevInfo->pvRegsBaseKM));
				PVR_LOG(("RGX Register Base Address (Physical): 0x%08lX", (unsigned long)psDevInfo->sRegsPhysBase.uiAddr));

				/* Forcing bit 6 of MslvCtrl1 to 0 to avoid internal reg read going though the core */
				OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVCTRL1, 0x0);

				eError = RGXRunScript(psDevInfo, psDevInfo->psScripts->asDbgCommands, RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_WARNING,"RGXDumpDebugInfo: RGXRunScript failed (%d) - Retry", eError));

					/* use thread1 for slave port accesses */
					OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVCTRL1, 0x1 << RGX_CR_META_SP_MSLVCTRL1_THREAD_SHIFT);

					eError = RGXRunScript(psDevInfo, psDevInfo->psScripts->asDbgCommands, RGX_MAX_INIT_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
					if (eError != PVRSRV_OK)
					{
						PVR_DPF((PVR_DBG_ERROR,"RGXDumpDebugInfo: RGXRunScript retry failed (%d) - Dump Slave Port debug information", eError));
						_RGXDumpMetaSPExtraDebugInfo(psDevInfo);
					}

					/* use thread0 again */
					OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVCTRL1, 0x0 << RGX_CR_META_SP_MSLVCTRL1_THREAD_SHIFT);
				}
			}
			else
			{
				PVR_LOG((" (!) RGX power is down. No registers dumped"));
			}

			{
				RGXFWIF_DM	eKCCBType;
				
				/*
		 			Dump out the kernel CCBs.
		 		*/
				for (eKCCBType = 0; eKCCBType < RGXFWIF_DM_MAX; eKCCBType++)
				{
					RGXFWIF_CCB_CTL	*psKCCBCtl = psDevInfo->apsKernelCCBCtl[eKCCBType];
		
					if (psKCCBCtl != IMG_NULL)
					{
						PVR_LOG(("RGX Kernel CCB %u WO:0x%X RO:0x%X",
								 eKCCBType, psKCCBCtl->ui32WriteOffset, psKCCBCtl->ui32ReadOffset));
					}
				}
		 	}

		 	/* Dump the IRQ info */
			{
					PVR_LOG(("RGX FW IRQ count = %d, last sampled in MISR = %d",
						psDevInfo->psRGXFWIfTraceBuf->ui32InterruptCount,
						g_ui32HostSampleIRQCount));
			}

			/* Dump the FW config flags */
			{
				RGXFWIF_INIT		*psRGXFWInit;

				eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc,
						(IMG_VOID **)&psRGXFWInit);

				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR,"RGXDumpDebugInfo: Failed to acquire kernel fw if ctl (%u)",
								eError));
					return;
				}

				PVR_LOG(("RGX FW config flags = 0x%X", psRGXFWInit->ui32ConfigFlags));

				DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
			}

			break;

		}
		case DEBUG_REQUEST_VERBOSITY_MEDIUM :
		{
			IMG_INT tid;
			/* Dump FW trace information */
			if (psDevInfo->psRGXFWIfTraceBuf != IMG_NULL)
			{
				RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
		
				for ( tid = 0 ; tid < RGXFW_THREAD_NUM ; tid++) 
				{
					IMG_UINT32	i;
					IMG_BOOL	bPrevLineWasZero = IMG_FALSE;
					IMG_BOOL	bLineIsAllZeros = IMG_FALSE;
					IMG_UINT32	ui32CountLines = 0;
					IMG_UINT32	*pui32TraceBuffer;
					IMG_CHAR	*pszLine;
		
					pui32TraceBuffer = &psRGXFWIfTraceBufCtl->sTraceBuf[tid].aui32TraceBuffer[0];
		
					/* each element in the line is 8 characters plus a space.  The '+1' is because of the final trailing '\0'. */
					pszLine = OSAllocMem(9*RGXFW_TRACE_BUFFER_LINESIZE+1);
					if (pszLine == IMG_NULL)
					{
						PVR_DPF((PVR_DBG_ERROR,"RGXDumpDebugInfo: Out of mem allocating line string (size: %d)", 9*RGXFW_TRACE_BUFFER_LINESIZE));
						return;
					}
		
					/* Print the tracepointer */
					if (psRGXFWIfTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_GROUP_MASK)
					{
						PVR_LOG(("Debug log type: %s ( %s%s%s%s%s%s%s%s%s%s%s)", 
							((psRGXFWIfTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_TRACE)?("trace"):("tbi")),
							RGXFWIF_LOG_ENABLED_GROUPS_LIST(psRGXFWIfTraceBufCtl->ui32LogType)
							));
					}
					else
					{
						PVR_LOG(("Debug log type: none"));
					}
					
					PVR_LOG(("------[ RGX FW thread %d trace START ]------", tid));
					PVR_LOG(("FWT[traceptr]: %X", psRGXFWIfTraceBufCtl->sTraceBuf[tid].ui32TracePointer));
		
					for (i = 0; i < RGXFW_TRACE_BUFFER_SIZE; i += RGXFW_TRACE_BUFFER_LINESIZE)
					{
						IMG_UINT32 k = 0;
						IMG_UINT32 ui32Line = 0x0;
						IMG_UINT32 ui32LineOffset = i*sizeof(IMG_UINT32);
						IMG_CHAR   *pszBuf = pszLine;
		
						for (k = 0; k < RGXFW_TRACE_BUFFER_LINESIZE; k++)
						{
							ui32Line |= pui32TraceBuffer[i + k];
		
							/* prepare the line to print it. The '+1' is because of the trailing '\0' added */
							OSSNPrintf(pszBuf, 9 + 1, " %08x", pui32TraceBuffer[i + k]);
							pszBuf += 9; /* write over the '\0' */
						}
		
						bLineIsAllZeros = (ui32Line == 0x0);
		
						if (bLineIsAllZeros && bPrevLineWasZero)
						{
							ui32CountLines++;
						}
						else if (bLineIsAllZeros && !bPrevLineWasZero)
						{
							bPrevLineWasZero = IMG_TRUE;
							ui32CountLines = 0;
							PVR_LOG(("FWT[%08x]: 00000000 ... 00000000", ui32LineOffset))
						}
						else
						{
							if (bPrevLineWasZero)
							{
								PVR_LOG(("FWT[%08x]: %d lines were all zero", ui32LineOffset, ui32CountLines));
							}
							else
							{
		
								PVR_LOG(("FWT[%08x]:%s", ui32LineOffset, pszLine));
							}
							bPrevLineWasZero = IMG_FALSE;
						}
		
					}
					if (bPrevLineWasZero)
					{
						PVR_LOG(("FWT[END]: %d lines were all zero", ui32CountLines));
					}
		
					PVR_LOG(("------[ RGX FW thread %d trace END ]------", tid));
		
					OSFreeMem(pszLine);
				}
			}
			break;
		}
		case DEBUG_REQUEST_VERBOSITY_HIGH:
		{
			PVRSRV_ERROR            eError;
			IMG_UINT32              ui32DeviceIndex;
			PVRSRV_DEV_POWER_STATE  ePowerState;
			IMG_BOOL                bRGXPoweredON;

			ui32DeviceIndex = psDevInfo->psDeviceNode->sDevId.ui32DeviceIndex;

			eError = PVRSRVGetDevicePowerState(ui32DeviceIndex, &ePowerState);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "RGXDebugRequestProcess: Error retrieving RGX power state. No debug info dumped."));
				return;
			}

			bRGXPoweredON = (ePowerState == PVRSRV_DEV_POWER_STATE_ON);

			PVR_LOG(("------[ Debug bus ]------"));

			_RGXDumpRGXDebugSummary(psDevInfo, bRGXPoweredON);

			if (bRGXPoweredON)
			{
				eError = RGXRunScript(psDevInfo, psDevInfo->psScripts->asDbgBusCommands, RGX_MAX_DBGBUS_COMMANDS, PDUMP_FLAGS_CONTINUOUS);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_WARNING,"RGXDumpDebugBusInfo: RGXRunScript failed (%s)", PVRSRVGetErrorStringKM(eError)));
				}
				break;
			}
		}
		default:
			break;
	}
}

/*
	RGXPanic
*/
IMG_VOID RGXPanic(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	PVR_LOG(("RGX panic"));
	PVRSRVDebugRequest(DEBUG_REQUEST_VERBOSITY_MAX);
	OSPanic();
}

/*
	RGXQueryDMState
*/
PVRSRV_ERROR RGXQueryDMState(PVRSRV_RGXDEV_INFO *psDevInfo, RGXFWIF_DM eDM, RGXFWIF_DM_STATE *peState, RGXFWIF_DEV_VIRTADDR *psCommonContextDevVAddr)
{
	PVRSRV_ERROR	eError = PVRSRV_OK;
	RGXFWIF_TRACEBUF *psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	if (eDM >= RGXFWIF_DM_MAX)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: eDM parameter is out of range (%u)",eError));
		return eError;
	}

	if (peState == IMG_NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: peState is NULL (%u)",eError));
		return eError;
	}

	if (psCommonContextDevVAddr == IMG_NULL)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: ppCommonContext is NULL (%u)",eError));
		return eError;
	}

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXQueryDMState: Failed (%d) to acquire adress for trace buffer", eError));
		return eError;
	}

	if (psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[eDM] != psRGXFWIfTraceBufCtl->aui16HwrDmLockedUpCount[eDM])
	{
		*peState = RGXFWIF_DM_STATE_LOCKEDUP;
	}
	else
	{
		*peState = RGXFWIF_DM_STATE_NORMAL;
	}
	
	*psCommonContextDevVAddr = psRGXFWIfTraceBufCtl->apsHwrDmFWCommonContext[eDM];

	return eError;
}


/******************************************************************************
 End of file (rgxdebug.c)
******************************************************************************/
