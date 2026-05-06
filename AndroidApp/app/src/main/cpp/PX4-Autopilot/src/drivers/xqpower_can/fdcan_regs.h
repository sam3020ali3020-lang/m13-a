/**
 * @file fdcan_regs.h
 * @brief Minimal STM32H7 FDCAN register definitions for XQPOWER CAN driver
 *
 * Extracted from STM32H743 CMSIS device header (stm32h743xx.h)
 * Only includes registers and bits used by the xqpower_can driver.
 */

#pragma once

#include <stdint.h>

/* ===== FDCAN Register Structure ===== */

typedef struct {
	volatile uint32_t CREL;         /* 0x000 Core Release */
	volatile uint32_t ENDN;         /* 0x004 Endian */
	volatile uint32_t RESERVED1;    /* 0x008 */
	volatile uint32_t DBTP;         /* 0x00C Data Bit Timing & Prescaler */
	volatile uint32_t TEST;         /* 0x010 Test */
	volatile uint32_t RWD;          /* 0x014 RAM Watchdog */
	volatile uint32_t CCCR;         /* 0x018 CC Control */
	volatile uint32_t NBTP;         /* 0x01C Nominal Bit Timing & Prescaler */
	volatile uint32_t TSCC;         /* 0x020 Timestamp Counter Config */
	volatile uint32_t TSCV;         /* 0x024 Timestamp Counter Value */
	volatile uint32_t TOCC;         /* 0x028 Timeout Counter Config */
	volatile uint32_t TOCV;         /* 0x02C Timeout Counter Value */
	volatile uint32_t RESERVED2[4]; /* 0x030-0x03C */
	volatile uint32_t ECR;          /* 0x040 Error Counter */
	volatile uint32_t PSR;          /* 0x044 Protocol Status */
	volatile uint32_t TDCR;         /* 0x048 Transmitter Delay Compensation */
	volatile uint32_t RESERVED3;    /* 0x04C */
	volatile uint32_t IR;           /* 0x050 Interrupt Register */
	volatile uint32_t IE;           /* 0x054 Interrupt Enable */
	volatile uint32_t ILS;          /* 0x058 Interrupt Line Select */
	volatile uint32_t ILE;          /* 0x05C Interrupt Line Enable */
	volatile uint32_t RESERVED4[8]; /* 0x060-0x07C */
	volatile uint32_t GFC;          /* 0x080 Global Filter Configuration */
	volatile uint32_t SIDFC;        /* 0x084 Standard ID Filter Config */
	volatile uint32_t XIDFC;        /* 0x088 Extended ID Filter Config */
	volatile uint32_t RESERVED5;    /* 0x08C */
	volatile uint32_t XIDAM;        /* 0x090 Extended ID AND Mask */
	volatile uint32_t HPMS;         /* 0x094 High Priority Message Status */
	volatile uint32_t NDAT1;        /* 0x098 New Data 1 */
	volatile uint32_t NDAT2;        /* 0x09C New Data 2 */
	volatile uint32_t RXF0C;        /* 0x0A0 Rx FIFO 0 Configuration */
	volatile uint32_t RXF0S;        /* 0x0A4 Rx FIFO 0 Status */
	volatile uint32_t RXF0A;        /* 0x0A8 Rx FIFO 0 Acknowledge */
	volatile uint32_t RXBC;         /* 0x0AC Rx Buffer Configuration */
	volatile uint32_t RXF1C;        /* 0x0B0 Rx FIFO 1 Configuration */
	volatile uint32_t RXF1S;        /* 0x0B4 Rx FIFO 1 Status */
	volatile uint32_t RXF1A;        /* 0x0B8 Rx FIFO 1 Acknowledge */
	volatile uint32_t RXESC;        /* 0x0BC Rx Buffer/FIFO Element Size */
	volatile uint32_t TXBC;         /* 0x0C0 Tx Buffer Configuration */
	volatile uint32_t TXFQS;        /* 0x0C4 Tx FIFO/Queue Status */
	volatile uint32_t TXESC;        /* 0x0C8 Tx Buffer Element Size */
	volatile uint32_t TXBRP;        /* 0x0CC Tx Buffer Request Pending */
	volatile uint32_t TXBAR;        /* 0x0D0 Tx Buffer Add Request */
	volatile uint32_t TXBCR;        /* 0x0D4 Tx Buffer Cancellation Request */
	volatile uint32_t TXBTO;        /* 0x0D8 Tx Buffer Transmission Occurred */
	volatile uint32_t TXBCF;        /* 0x0DC Tx Buffer Cancellation Finished */
	volatile uint32_t TXBTIE;       /* 0x0E0 Tx Buffer Tx Interrupt Enable */
	volatile uint32_t TXBCIE;       /* 0x0E4 Tx Buffer Cancellation Finished IE */
	volatile uint32_t RESERVED6[2]; /* 0x0E8-0x0EC */
	volatile uint32_t TXEFC;        /* 0x0F0 Tx Event FIFO Configuration */
	volatile uint32_t TXEFS;        /* 0x0F4 Tx Event FIFO Status */
	volatile uint32_t TXEFA;        /* 0x0F8 Tx Event FIFO Acknowledge */
} FDCAN_Regs_t;

/* ===== Memory Addresses (STM32H743) ===== */

#define FDCAN1_BASE_ADDR     0x4000A000U
#define SRAMCAN_BASE_ADDR    0x4000AC00U

/* RCC registers for FDCAN clock enable */
#define RCC_BASE_ADDR        0x58024400U
#define RCC_APB1HENR_OFFSET  0x0ECU
#define RCC_APB1HRSTR_OFFSET 0x094U
#define RCC_FDCANEN_BIT      (1U << 8)   /* FDCANEN in APB1HENR */
#define RCC_FDCANRST_BIT     (1U << 8)   /* FDCANRST in APB1HRSTR */

/* ===== CCCR Register Bits ===== */

#define FDCAN_CCCR_INIT      (1U << 0)   /* Initialization */
#define FDCAN_CCCR_CCE       (1U << 1)   /* Configuration Change Enable */
#define FDCAN_CCCR_CSR       (1U << 4)   /* Clock Stop Request */
#define FDCAN_CCCR_CSA       (1U << 3)   /* Clock Stop Acknowledge */
#define FDCAN_CCCR_MON       (1U << 5)   /* Bus Monitoring Mode */
#define FDCAN_CCCR_DAR       (1U << 6)   /* Disable Auto Retransmission */
#define FDCAN_CCCR_TEST      (1U << 7)   /* Test Mode Enable */
#define FDCAN_CCCR_FDOE      (1U << 8)   /* FD Operation Enable */

/* ===== NBTP Register (Nominal Bit Timing & Prescaler) ===== */

#define FDCAN_NBTP_TSEG2_Pos   0U
#define FDCAN_NBTP_NTSEG1_Pos  8U
#define FDCAN_NBTP_NBRP_Pos    16U
#define FDCAN_NBTP_NSJW_Pos    25U

/* ===== GFC Register (Global Filter Configuration) ===== */

#define FDCAN_GFC_ANFS_Pos   4U        /* Accept Non-matching Frames Standard */
#define FDCAN_GFC_ANFE_Pos   2U        /* Accept Non-matching Frames Extended */
#define FDCAN_GFC_ANFS_Msk   (0x3U << FDCAN_GFC_ANFS_Pos)
#define FDCAN_GFC_ANFE_Msk   (0x3U << FDCAN_GFC_ANFE_Pos)

/* ===== RXF0C Register (Rx FIFO 0 Configuration) ===== */

#define FDCAN_RXF0C_F0SA_Pos  2U
#define FDCAN_RXF0C_F0S_Pos   16U

/* ===== RXF0S Register (Rx FIFO 0 Status) ===== */

#define FDCAN_RXF0S_F0FL_Msk  0x7FU     /* Rx FIFO 0 Fill Level */
#define FDCAN_RXF0S_F0GI_Pos  8U
#define FDCAN_RXF0S_F0GI_Msk  (0x3FU << FDCAN_RXF0S_F0GI_Pos)

/* ===== RXF0A Register (Rx FIFO 0 Acknowledge) ===== */

#define FDCAN_RXF0A_F0AI_Msk  0x3FU

/* ===== TXBC Register (Tx Buffer Configuration) ===== */

#define FDCAN_TXBC_TBSA_Pos   2U
#define FDCAN_TXBC_TFQS_Pos   24U
#define FDCAN_TXBC_TFQM       (1U << 30)

/* ===== TXFQS Register (Tx FIFO/Queue Status) ===== */

#define FDCAN_TXFQS_TFQF      (1U << 21) /* Tx FIFO/Queue Full */
#define FDCAN_TXFQS_TFQPI_Pos 16U
#define FDCAN_TXFQS_TFQPI_Msk (0x1FU << FDCAN_TXFQS_TFQPI_Pos)

/* ===== TEST Register ===== */

#define FDCAN_TEST_LBCK       (1U << 4)  /* Loop Back mode */

/* ===== Message RAM Element Size ===== */

#define FDCAN_ELEMENT_WORDS   4U   /* 4 words per element (classic CAN) */
#define FDCAN_ELEMENT_BYTES   16U  /* 16 bytes per element */
