#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>

enum {
REG_COMMON_BASE,
    REG_COMMON_REGS ,
    REG_MR          ,
    REG_MR_RST            ,
    REG_MR_PB             ,
    REG_MR_AI             ,
    REG_MR_IND            ,
    REG_SHAR        ,
    REG_IR          ,
    REG_COMMON_REGS_LEN,

    REG_RTR	    ,
    REG_RTR_DEFAULT		,
    REG_RMSR		,
    REG_TMSR		,
    REG_SIMR		,
    REG_Sn_MR       ,
    REG_Sn_CR       ,
    REG_Sn_IR       ,
    REG_set_Sn_IR   ,
    REG_Sn_SR       ,
    REG_Sn_TX_FSR   ,
    REG_Sn_TX_RD    ,
    REG_Sn_TX_WR    ,
    REG_Sn_RX_RSR   ,
    REG_Sn_RX_RD    ,
    REG_S0_MR_MACRAW      ,
    REG_S0_MR_MF          ,
    REG_S0_CR_OPEN        ,
    REG_S0_CR_CLOSE       ,
    REG_S0_CR_SEND        ,
    REG_S0_CR_RECV        ,
    REG_S0_IR_SENDOK      ,
    REG_S0_IR_RECV        ,
    REG_S0_SR_MACRAW      ,
    REG_S0_REGS_LEN ,
    REG_TX_MEM_START,
    REG_TX_MEM_SIZE ,
    REG_RX_MEM_START,
    REG_RX_MEM_SIZE ,
    REG_IMR		,
    REG_IR_S0			,
    REG_S0_REGS	,
    REG_Sn_RX_BSR   ,
    REG_Sn_TX_BSR   ,
    REG_PHYCFG      ,
    REG_PHYSTATE    ,
    REG_PHYCFG_AUTONEG ,
    NUM_REGS       
};

/* Chip-specific register maps */
extern const u32 wiz5100_map[NUM_REGS];
extern const u32 wiz5200_map[NUM_REGS];
extern const u32 wiz5500_map[NUM_REGS];
extern const u32 wiz6100_map[NUM_REGS];
extern const u32 wiz6300_map[NUM_REGS];

//#define MR   (priv->map[IDX_MR])

/*
 * W5100/W5200/W5500 common registers
 */

// W6100 register map (기존 W5100/W5500 매크로 이름 그대로 사용하면서 주소만 W6100에 맞게 수정)


#define DRV_NAME	"w5100"
#define DRV_VERSION	"2012-04-04"

#define COMMON_REGS       0x0000
#define MR                (priv->map[REG_MR]) /* Mode Register */
#define   MR_RST          (priv->map[REG_MR_RST]) /* S/W reset */
#define   MR_PB           (priv->map[REG_MR_PB]) /* Ping block */
#define   MR_AI           (priv->map[REG_MR_AI]) /* Address Auto-Increment */
#define   MR_IND         (priv->map[REG_MR_IND]) /* Indirect mode */
#define SHAR              (priv->map[REG_SHAR]) /* Source MAC address */
#define IR                (priv->map[REG_IR]) /* Interrupt Register */
#define COMMON_REGS_LEN   (priv->map[REG_COMMON_REGS_LEN])  /* Common register length */

/* Socket register dynamic mapping */

#define Sn_MR             (priv->map[REG_Sn_MR]) /* Sn Mode Register */
#define Sn_CR             (priv->map[REG_Sn_CR]) /* Sn Command Register */
#define Sn_IR             (priv->map[REG_Sn_IR]) /* Sn Interrupt Register */
#define set_Sn_IR         (priv->map[REG_set_Sn_IR]) /* Sn Interrupt Set Register */
#define Sn_SR             (priv->map[REG_Sn_SR]) /* Sn Status Register */
#define Sn_TX_FSR         (priv->map[REG_Sn_TX_FSR]) /* Sn TX Free Size */
#define Sn_TX_RD          (priv->map[REG_Sn_TX_RD]) /* Sn TX Read Ptr */
#define Sn_TX_WR          (priv->map[REG_Sn_TX_WR]) /* Sn TX Write Ptr */
#define Sn_RX_RSR         (priv->map[REG_Sn_RX_RSR]) /* Sn RX Received Size */
#define Sn_RX_RD          (priv->map[REG_Sn_RX_RD]) /* Sn RX Read Ptr */

/* Socket register values (chip-specific) */
#define S0_MR_MACRAW      (priv->map[REG_S0_MR_MACRAW])
#define S0_MR_MF          (priv->map[REG_S0_MR_MF])
#define S0_CR_OPEN        (priv->map[REG_S0_CR_OPEN])
#define S0_CR_CLOSE       (priv->map[REG_S0_CR_CLOSE])
#define S0_CR_SEND        (priv->map[REG_S0_CR_SEND])
#define S0_CR_RECV        (priv->map[REG_S0_CR_RECV])
#define S0_IR_SENDOK      (priv->map[REG_S0_IR_SENDOK])
#define S0_IR_RECV        (priv->map[REG_S0_IR_RECV])

#define S0_SR_MACRAW      (priv->map[REG_S0_SR_MACRAW])

#define S0_REGS_LEN       (priv->map[REG_S0_REGS_LEN]) /* Socket register block length */

// TX/RX 메모리 영역 (W6100 기준)
//#define TX_MEM_START      0x8000 /* TX Memory start address */
//#define TX_MEM_SIZE       0x08000 /* 32KB */
//#define RX_MEM_START      0xC000 /* RX Memory start address */
//#define RX_MEM_SIZE       0x08000 /* 32KB */




/*
 * Dynamic register mapping for all chips
 */
#define IMR               (priv->map[REG_IMR]) /* Interrupt Mask Register */
#define   IR_S0           (priv->map[REG_IR_S0]) /* S0 interrupt */
#define RTR               (priv->map[REG_RTR]) /* Retry Time-value Register */
#define   RTR_DEFAULT     (priv->map[REG_RTR_DEFAULT]) /* Default retry time */

/* Memory size registers (chip-specific) */
#define RMSR              (priv->map[REG_RMSR]) /* Receive Memory Size */
#define TMSR              (priv->map[REG_TMSR]) /* Transmit Memory Size */

/* Memory mapping (chip-specific) */
#define TX_MEM_START      (priv->map[REG_TX_MEM_START])
#define TX_MEM_SIZE       (priv->map[REG_TX_MEM_SIZE])
#define RX_MEM_START      (priv->map[REG_RX_MEM_START])
#define RX_MEM_SIZE       (priv->map[REG_RX_MEM_SIZE])

/* W5500/W6100 specific */
#define SIMR              (priv->map[REG_SIMR]) /* Socket Interrupt Mask Register */

/* W6100 specific constants using dynamic mapping */
#define Sn_RX_BSR         (priv->map[REG_Sn_RX_BSR])
#define Sn_TX_BSR         (priv->map[REG_Sn_TX_BSR])

#define PHYCFG            (priv->map[REG_PHYCFG])
#define PHYSTATE          (priv->map[REG_PHYSTATE])
#define PHYCFG_AUTONEG    (priv->map[REG_PHYCFG_AUTONEG])

/* Chip-specific memory size helper macros */
#define W5500_Sn_RXMEM_SIZE(n)	(0x1001e + (n) * 0x40000) /* Sn RX Memory Size */
#define W5500_Sn_TXMEM_SIZE(n)	(0x1001f + (n) * 0x40000) /* Sn TX Memory Size */

#define W5200_Sn_RXMEM_SIZE(n)	(0x401e + (n) * 0x0100) /* Sn RX Memory Size */
#define W5200_Sn_TXMEM_SIZE(n)	(0x401f + (n) * 0x0100) /* Sn TX Memory Size */

#define W6100_Sn_RXMEM_SIZE(n)  (Sn_RX_BSR +(((1+4*n)<<3)<<16))  /* Sn RX Memory Size */
#define W6100_Sn_TXMEM_SIZE(n)  (Sn_TX_BSR +(((1+4*n)<<3)<<16)) /* Sn TX Memory Size */

/* Socket accessor macros using dynamic mapping */
// #define S0_REGS(priv)     ((priv)->s0_regs)
#define S0_REGS(priv)          ((priv->map[REG_S0_REGS]))
#define S0_MR(priv)       (S0_REGS(priv) + (priv->map[REG_Sn_MR]))
#define S0_CR(priv)       (S0_REGS(priv) + (priv->map[REG_Sn_CR]))
#define S0_IR(priv)       (S0_REGS(priv) + (priv->map[REG_Sn_IR]))
#define set_S0_IR(priv)   (S0_REGS(priv) + (priv->map[REG_set_Sn_IR]))
#define S0_SR(priv)       (S0_REGS(priv) + (priv->map[REG_Sn_SR]))
#define S0_TX_FSR(priv)   (S0_REGS(priv) + (priv->map[REG_Sn_TX_FSR]))
#define S0_TX_RD(priv)    (S0_REGS(priv) + (priv->map[REG_Sn_TX_RD]))
#define S0_TX_WR(priv)    (S0_REGS(priv) + (priv->map[REG_Sn_TX_WR]))
#define S0_RX_RSR(priv)   (S0_REGS(priv) + (priv->map[REG_Sn_RX_RSR]))
#define S0_RX_RD(priv)    (S0_REGS(priv) + (priv->map[REG_Sn_RX_RD]))