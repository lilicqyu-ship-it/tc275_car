#include "mw/calib/calib_store.h"

#include "IfxFlash.h"
#include "bsp/stime.h"
#include "bsp/uart.h"
#include "bsp/wdg.h"
#include "mw/sf/sf_frame.h"
#include "mw/xcore/xcore.h"

/* ---- storage slot -----------------------------------------------------------
 * Values read out of the iLLD, not from memory (_Impl/IfxFlash_cfg.h and
 * IfxFlash_cfg.c): DF0 base IFXFLASH_DFLASH_START 0xAF000000, logical sector
 * 0x2000, page IFXFLASH_DFLASH_PAGE_LENGTH 8 bytes, and the tables put the
 * HSM log at 0xAF110000.. and the UCB at 0xAF100000.. - clear of the sector
 * used here.
 *
 * Sector 15 = the last one inside the DF0 size SDD SS4.2 states for TC275
 * (128 KB = 16 sectors). iLLD's logical table lists 48 sectors (up to
 * 0xAF05FFFF), which is a family maximum, so the slot sits inside DF0 under
 * both readings, and leaves sectors 0..14 to the §4.3 config/black-box pages
 * when they land - they must not claim sector 15.
 * Before the second DFlash user appears, confirm the real sector count from
 * the device datasheet plus a bench erase/program/read cycle here.
 * Endurance: bench saves <=10/day vs >=1e5 cycles -> decades, no wear
 * levelling (doc 34 SS8.1). */
#define CALIB_SECTOR_ADDR \
    (IFXFLASH_DFLASH_START + (15u * 0x2000u))

/* Wait budget for erase/program: datasheet tens of ms; the bound exists so a
 * stuck FMU costs a logged write failure, not a watchdog reset (doc 34
 * SS8.3's pause budget assumes the operation completes). */
#define CALIB_FLASH_TIMEOUT_MS  200u

/* Deferred write only once the bench is quiet this long after the trigger
 * (or the last motion sighting), so a save never overlaps motors running
 * (doc 34 SS8.3). */
#define CALIB_WRITE_IDLE_MS     500u

/* Deferred operation slots: one record write, or CLEAR's erase only. */
#define CALIB_ACT_IDLE   0u
#define CALIB_ACT_WRITE  1u
#define CALIB_ACT_ERASE  2u

/* A deferred operation gives up after this many failed flash attempts
 * (doc 34 SS11.4 C6 traded an unbounded retry for "the result must not be
 * lost silently" - but every attempt is a new erase/program cycle on the
 * single slot, and a persistently failing FMU then hammers it every
 * CALIB_WRITE_IDLE_MS forever. Bounded now: the live record stays applied
 * in RAM and the console line says why it never reached DFlash.) */
#define CALIB_FLASH_RETRIES  3u

typedef struct
{
    uint8       action;          /* CALIB_ACT_*                               */
    CalibRecord rec;
    uint32      quietMs;         /* earliest moment a write may start         */
    uint8       attempts;        /* failed flash attempts so far              */
} PendingWrite;

static PendingWrite g_pending;

/* Arm a deferred operation. CALIB_tick runs it once the bench has been quiet
 * for CALIB_WRITE_IDLE_MS, and re-arms through calib_delayWrite() on motion or
 * a failed flash operation. */
static void calib_queueWrite(uint8 action, const CalibRecord *rec)
{
    g_pending.action   = action;
    g_pending.rec      = *rec;
    g_pending.quietMs  = STIME_nowMs();
    g_pending.attempts = 0u;
}

static void calib_delayWrite(void)
{
    g_pending.quietMs = STIME_nowMs() + CALIB_WRITE_IDLE_MS;
}

/* A DONE calibration result holds its EVT 0x22 until the auto-persist has
 * run, so the frame's saved byte is the truth (doc 34 SS9.4). */
static XcoreCalibResult g_holdResult;
static boolean          g_holdActive;

/* ---- low-level flash -------------------------------------------------------- */

/* Poll the DF0 busy flag. Runs inside this core's interrupt mask, so the
 * timebase is the free-running STM (no tick interrupt needed) and the CPU
 * watchdog is refreshed here: erase + three page programs can legitimately
 * outlast the ~0.5 s window, and a stuck FMU still hits the per-operation
 * deadline and reports a failed save instead of a reset. */
static boolean calib_flashWaitD0(void)
{
    uint32 deadline = STIME_nowMs() + CALIB_FLASH_TIMEOUT_MS;

    while (FLASH0_FSR.B.D0BUSY != 0u)
    {
        WDG_serviceCpu();
        if ((sint32)(STIME_nowMs() - deadline) >= 0)
        {
            return FALSE;
        }
    }
    return TRUE;
}

/* Wait for the FMU and vet the outcome: a command the flash rejected
 * (protection / operation error) leaves D0BUSY clear and the error latched
 * in FSR, which a bare busy-wait reads as success. Detected here, reported
 * as a failed operation (same contract flash_ota.c's flashota_waitBank
 * uses on the PFlash banks). */
static boolean calib_flashOpOk(void)
{
    if (calib_flashWaitD0() == FALSE)
    {
        return FALSE;
    }
    if ((FLASH0_FSR.B.OPER != 0u) || (FLASH0_FSR.B.PROER != 0u))
    {
        IfxFlash_clearStatus(0u);
        return FALSE;
    }
    return TRUE;
}

/* Erase + program the blob into the slot, then read the record bytes back.
 * iLLD gives a 20 B record and an 8 B DFlash page (IFXFLASH_DFLASH_PAGE_
 * LENGTH), and ECC is computed per page, so the blob is programmed one page
 * at a time with its own enter/load/write sequence; the last page is zero
 * padded and the padding is not part of the read-back compare. */
#define CALIB_FLASH_PAGE_LEN IFXFLASH_DFLASH_PAGE_LENGTH
#define CALIB_FLASH_PAGE_CNT \
    ((CALIB_REC_BLOB_LEN + CALIB_FLASH_PAGE_LEN - 1u) / CALIB_FLASH_PAGE_LEN)

static boolean calib_flashWritePage(uint32 pageAddr, const uint8 *bytes)
{
    boolean ok;

    IfxFlash_clearStatus(0u);
    ok = (IfxFlash_enterPageMode(pageAddr) == 0u);
    if (ok != FALSE)
    {
        ok = calib_flashOpOk();         /* page mode must not race the erase */
    }
    if (ok != FALSE)
    {
        IfxFlash_loadPage(pageAddr,
                          (uint32)CALIBREC_getI32(&bytes[0]),
                          (uint32)CALIBREC_getI32(&bytes[4]));
        IfxFlash_writePage(pageAddr);
        ok = calib_flashOpOk();
    }
    IfxFlash_clearStatus(0u);
    return ok;
}

static boolean calib_flashWrite(const uint8 *blob)
{
    uint8  page[CALIB_FLASH_PAGE_LEN];
    uint32 p;
    uint32 i;

    for (p = 0u; p < CALIB_FLASH_PAGE_CNT; p++)
    {
        for (i = 0u; i < CALIB_FLASH_PAGE_LEN; i++)
        {
            uint32 idx = (p * CALIB_FLASH_PAGE_LEN) + i;

            page[i] = (idx < CALIB_REC_BLOB_LEN) ? blob[idx] : 0u;
        }
        if (calib_flashWritePage(CALIB_SECTOR_ADDR +
                                 (p * CALIB_FLASH_PAGE_LEN), page) == FALSE)
        {
            return FALSE;
        }
    }

    for (i = 0u; i < CALIB_REC_BLOB_LEN; i++)
    {
        if (*(volatile uint8 *)(CALIB_SECTOR_ADDR + i) != blob[i])
        {
            return FALSE;
        }
    }
    return TRUE;
}

static void calib_flashRead(uint8 *blob)
{
    uint32 i;

    for (i = 0u; i < CALIB_REC_BLOB_LEN; i++)
    {
        blob[i] = *(volatile uint8 *)(CALIB_SECTOR_ADDR + i);
    }
}

/* Encode scratch buffer. Kept at file scope rather than as a calib_flashSave
 * local because the robot task runs on 2 x configMINIMAL_STACK_SIZE. */
static uint8 g_opBlob[CALIB_REC_BLOB_LEN];

/* Save sequence per doc 34 SS8.3: feed -> mask -> erase + program + read
 * back -> unmask -> feed. Called from the robot task, never from a critical
 * section, so the plain mask/unmask is safe here. The masked window is tens
 * of ms in practice (bounded by CALIB_FLASH_TIMEOUT_MS per operation); CPU0
 * is the only core that fetches from DFlash, so CPU1/CPU2 do not stall.
 *
 * Ordering is load-bearing against the bench "calibrate once, TC275 dead"
 * failure: with a status flag latched (a rejected earlier command), the FMU
 * silently ignores the erase while the per-page sequences below - each
 * starting with their own clearStatus - still execute, programming pages of
 * a sector that was never erased. Reprogramming a live DFlash page corrupts
 * its ECC, and every later read of the slot (the verify here, CALIB_init at
 * each boot) takes a synchronous-data-error trap on CPU0 - before the CPU
 * sync event at boot, hanging all three cores. So: clear the status before
 * the erase, check FSR after every command, and only ever program a sector
 * that verifiably reads erased (DFlash erase level = 0xFF, per the SBL's
 * own DFlash meta magic check on a fresh bank). */
static boolean calib_flashSave(const CalibRecord *rec)
{
    boolean ok;

    CALIBREC_encode(rec, g_opBlob);

    WDG_serviceCpu();
    __disable();
    IfxFlash_clearStatus(0u);
    IfxFlash_eraseSector(CALIB_SECTOR_ADDR);
    ok = calib_flashOpOk();
    if (ok != FALSE)
    {
        ok = (*(volatile uint8 *)CALIB_SECTOR_ADDR == 0xFFu) ? TRUE : FALSE;
    }
    if (ok != FALSE)
    {
        ok = calib_flashWrite(g_opBlob);
    }
    __enable();
    WDG_serviceCpu();
    return ok;
}

static boolean calib_flashErase(void)
{
    boolean ok;

    WDG_serviceCpu();
    __disable();
    IfxFlash_clearStatus(0u);
    IfxFlash_eraseSector(CALIB_SECTOR_ADDR);
    ok = calib_flashOpOk();
    __enable();
    WDG_serviceCpu();
    return ok;
}

/* ---- live-record plumbing --------------------------------------------------- */

/* EVT 0x23 echo of the live record. crcOk rides the source: only the default
 * fallback (no record / failed validation, doc 34 SS8.1) reads 0. */
void CALIB_sendRecord(void)
{
    XcoreEvtFrame   frame;
    XcoreRecordLive live;

    XCORE_recordGet(&live);

    frame.type = SF_TYPE_EVT;
    frame.cid  = SF_CID_DPT_REC;
    frame.len  = CALIB_EVT_REC_LEN;
    CALIBREC_buildEvtRec(frame.payload, &live.rec,
                         (live.rec.src == CALIB_SRC_DEFAULT) ? 0u : 1u);
    (void)XCORE_evtPush(&frame);
}

static void calib_sendResultEvt(const XcoreCalibResult *res, uint8 saved)
{
    XcoreEvtFrame frame;

    frame.type = SF_TYPE_EVT;
    frame.cid  = SF_CID_DPT_RESULT;
    frame.len  = CALIB_EVT_RESULT_LEN;
    CALIBREC_buildEvtResult(frame.payload, res->status, res->invert,
                            res->delta, saved);
    (void)XCORE_evtPush(&frame);
}

/* Merge an invert set into the live record and queue the write (doc 34
 * SS8.3: 0x70 success auto-persists, everything else is kept). */
static void calib_persistInvert(const sint8 invert[CALIB_REC_WHEELS])
{
    XcoreRecordLive live;
    CalibRecord     rec;
    uint8           i;

    XCORE_recordGet(&live);
    rec = live.rec;
    for (i = 0u; i < CALIB_REC_WHEELS; i++)
    {
        rec.invert[i] = invert[i];
    }
    if (CALIBREC_paramsOk(&rec) == 0u)
    {
        return;                  /* live params corrupted: refuse to persist  */
    }
    rec.src = CALIB_SRC_DFLASH;
    XCORE_recordSet(&rec);

    calib_queueWrite(CALIB_ACT_WRITE, &rec);
}

/* ---- vehicle-quiet gate ------------------------------------------------------ */

static uint8 calib_motionSeen(void)
{
    XcoreJog        jog;
    XcoreEncoder    enc;
    sint16          tgtL, tgtR;
    boolean         tgtEstop;
    uint8           i;

    (void)XCORE_jogGet(&jog);
    for (i = 0u; i < CALIB_REC_WHEELS; i++)
    {
        if (jog.duty[i] != 0)
        {
            return 1u;
        }
    }

    (void)XCORE_motorGetTarget(&tgtL, &tgtR, &tgtEstop);
    if ((tgtL != 0) || (tgtR != 0))
    {
        return 1u;
    }

    XCORE_encoderRead(&enc);
    if ((enc.pctLeft != 0) || (enc.pctRight != 0))
    {
        return 1u;
    }
    return 0u;
}

/* ---- public ----------------------------------------------------------------- */

void CALIB_init(void)
{
    uint8       blob[CALIB_REC_BLOB_LEN];
    CalibRecord rec;

    calib_flashRead(blob);
    if (CALIBREC_decode(blob, &rec) != 0u)
    {
        UART_println("CALIBREC loaded from DFLASH");
    }
    else
    {
        /* CALIBREC_decode already leaves the defaults with src=DEFAULT */
        UART_println("CALIBREC invalid, defaults");
    }
    XCORE_recordSet(&rec);

    g_pending.action = CALIB_ACT_IDLE;
    g_holdActive     = FALSE;
}

static void calib_handleResult(void)
{
    XcoreCalibResult res;

    if (g_holdActive != FALSE)
    {
        /* A second DONE result arrived while the first save is still in
         * flight: answer the older one as if the save failed, then follow
         * the newer (cannot legitimately happen - one run at a time). */
        calib_sendResultEvt(&g_holdResult, CALIB_SAVED_FAILED);
        g_holdActive = FALSE;
    }

    if (XCORE_calibResultTake(&res) == FALSE)
    {
        return;
    }

    if (res.status == CALIB_STATUS_DONE)
    {
        calib_persistInvert(res.invert);
        if (g_pending.action == CALIB_ACT_WRITE)
        {
            g_holdResult = res;
            g_holdActive = TRUE;
            return;              /* the frame rides the save outcome */
        }
        /* persistence refused (invalid live params): say so honestly */
        calib_sendResultEvt(&res, CALIB_SAVED_FAILED);
        return;
    }

    calib_sendResultEvt(&res, CALIB_SAVED_NONE);
}

void CALIB_recordSet(const uint8 *data, uint8 len)
{
    XcoreRecordLive live;
    CalibRecord     rec;

    XCORE_recordGet(&live);
    rec = live.rec;
    if (CALIBREC_recSetDecode(data, len, &rec) == 0u)
    {
        /* Invalid body: live state untouched, the echo says what is in force. */
        CALIB_sendRecord();
        return;
    }

    rec.src = CALIB_SRC_ONLINE;
    XCORE_recordSet(&rec);
    CALIB_sendRecord();

    calib_queueWrite(CALIB_ACT_WRITE, &rec);
}

void CALIB_recordClear(void)
{
    CalibRecord rec;

    CALIBREC_fillDefaults(&rec);
    XCORE_recordSet(&rec);
    CALIB_sendRecord();

    calib_queueWrite(CALIB_ACT_ERASE, &rec);
}

void CALIB_tick(void)
{
    /* Drain CPU1's result mailbox on the 10 ms rhythm: one frame per run. */
    calib_handleResult();

    if (g_pending.action == CALIB_ACT_IDLE)
    {
        return;
    }

    if (calib_motionSeen() != 0u)
    {
        calib_delayWrite();
        return;
    }
    if ((sint32)(STIME_nowMs() - g_pending.quietMs) < 0)
    {
        return;
    }

    if (g_pending.action == CALIB_ACT_ERASE)
    {
        if (calib_flashErase() != FALSE)
        {
            g_pending.action = CALIB_ACT_IDLE;
        }
        else if (++g_pending.attempts >= CALIB_FLASH_RETRIES)
        {
            /* Give up, loudly: the live record already answered with the
             * defaults, but a stale DFlash copy would resurface at the next
             * boot. The console line is the bench operator's cue. */
            g_pending.action = CALIB_ACT_IDLE;
            XCORE_logln("CALCLEAR failed (flash)");
        }
        else
        {
            calib_delayWrite();
        }
        return;
    }

    {
        uint8 saved;

        saved = (calib_flashSave(&g_pending.rec) != FALSE)
                    ? CALIB_SAVED_WRITTEN : CALIB_SAVED_FAILED;

        if (g_holdActive != FALSE)
        {
            calib_sendResultEvt(&g_holdResult, saved);
            g_holdActive = FALSE;
        }

        if (saved == CALIB_SAVED_WRITTEN)
        {
            g_pending.action = CALIB_ACT_IDLE;
        }
        else if (++g_pending.attempts >= CALIB_FLASH_RETRIES)
        {
            /* Give up, loudly: the calibrated record stays applied in RAM
             * (signs, closed-loop gate) for this power cycle only; the
             * result frame already went out with saved=2. */
            g_pending.action = CALIB_ACT_IDLE;
            XCORE_logln("CALSAVE failed (flash)");
        }
        else
        {
            /* retry quietly; the result frame has already gone out */
            calib_delayWrite();
        }
    }
}
