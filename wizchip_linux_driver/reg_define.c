#include "reg_define.h"

const u32 wiz5100_map[NUM_REGS] = {
    /* Common Registers (0x0000 ~ 0x003F) */
    [REG_COMMON_BASE ]      = 0x0000,
    [REG_COMMON_REGS ]      = 0x0000,
    [REG_COMMON_REGS_LEN]   = 0x0040,   /* Common register length */
    
    [REG_MR          ]      = 0x0000,   /* Mode Register */
        [REG_MR_RST  ]      = 0x80,     /* S/W reset */
        [REG_MR_PB   ]      = 0x10,     /* Ping block */
        [REG_MR_AI   ]      = 0x02,     /* Address Auto-Increment */
        [REG_MR_IND  ]      = 0x01,     /* Indirect mode */

    [REG_SHAR        ]      = 0x0009,   /* Source MAC address */
    [REG_IR          ]      = 0x0015,   /* Interrupt Register */
        [REG_IR_S0   ]      = 0x01,     /* Socket 0 interrupt bit */
    
    [REG_IMR         ]      = 0x0016,   /* Interrupt Mask Register */
    [REG_RTR         ]      = 0x0017,   /* Retry Time-value Register */
        [REG_RTR_DEFAULT]   = 0x07d0,   /* Default retry time (2000) */
    
    [REG_RMSR        ]      = 0x001a,   /* RX Memory Size Register */
    [REG_TMSR        ]      = 0x001b,   /* TX Memory Size Register */ 
    // 여기까지 확인 ㅠ
    
    /* Socket Registers (relative offsets from socket base) */
    [REG_S0_REGS     ]      = 0x0400,   /* Socket 0 register base */
    [REG_S0_REGS_LEN ]      = 0x0040,   /* Socket register length */
    
    [REG_Sn_MR       ]      = 0x0000,   /* Socket n Mode Register (relative) */
        [REG_S0_MR_MACRAW]  = 0x04,     /* MAC RAW mode */
        [REG_S0_MR_MF    ]  = 0x40,     /* MAC Filter for W5100 */
    
    [REG_Sn_CR       ]      = 0x0001,   /* Socket n Command Register (relative) */
        [REG_S0_CR_OPEN  ]  = 0x01,     /* OPEN command */
        [REG_S0_CR_CLOSE ]  = 0x10,     /* CLOSE command */
        [REG_S0_CR_SEND  ]  = 0x20,     /* SEND command */
        [REG_S0_CR_RECV  ]  = 0x40,     /* RECV command */
    
    [REG_Sn_IR       ]      = 0x0002,   /* Socket n Interrupt Register (relative) */
    [REG_set_Sn_IR   ]      = 0x0002,   /* Same as Sn_IR for W5100 */
        [REG_S0_IR_SENDOK]  = 0x10,     /* SEND_OK interrupt */
        [REG_S0_IR_RECV  ]  = 0x04,     /* RECV interrupt */
    
    [REG_Sn_SR       ]      = 0x0003,   /* Socket n Status Register (relative) */
        [REG_S0_SR_MACRAW]  = 0x42,     /* MACRAW status */
    
    [REG_Sn_TX_FSR   ]      = 0x0020,   /* Socket n TX Free Size Register (relative) */
    [REG_Sn_TX_RD    ]      = 0x0022,   /* Socket n TX Read Pointer Register (relative) */
    [REG_Sn_TX_WR    ]      = 0x0024,   /* Socket n TX Write Pointer Register (relative) */
    [REG_Sn_RX_RSR   ]      = 0x0026,   /* Socket n RX Received Size Register (relative) */
    [REG_Sn_RX_RD    ]      = 0x0028,   /* Socket n RX Read Pointer Register (relative) */
    
    /* Memory Map */
    [REG_TX_MEM_START]      = 0x4000,   /* TX memory start address */
    [REG_TX_MEM_SIZE ]      = 0x2000,   /* TX memory size (8KB) */
    [REG_RX_MEM_START]      = 0x6000,   /* RX memory start address */
    [REG_RX_MEM_SIZE ]      = 0x2000,   /* RX memory size (8KB) */
    
    /* Not used in W5100 */
    [REG_SIMR        ]      = 0x0000,   /* Not used in W5100 (for W5500) */
    [REG_Sn_RX_BSR   ]      = 0x0000,   /* Not used in W5100 */
    [REG_Sn_TX_BSR   ]      = 0x0000,   /* Not used in W5100 */
    [REG_PHYCFG      ]      = 0x0000,   /* Not used in W5100 */
    [REG_PHYSTATE    ]      = 0x0000,   /* Not used in W5100 */
    [REG_PHYCFG_AUTONEG]    = 0x0000,   /* Not used in W5100 */
}; 

const u32 wiz5200_map[NUM_REGS] = {
    /* Common Registers (0x0000 ~ 0x003F) */
    [REG_COMMON_BASE ]      = 0x0000,
    [REG_COMMON_REGS ]      = 0x0000,
    [REG_COMMON_REGS_LEN]   = 0x0040,   /* Common register length */
    
    [REG_MR          ]      = 0x0000,   /* Mode Register */
        [REG_MR_RST  ]      = 0x80,     /* S/W reset */
        [REG_MR_PB   ]      = 0x10,     /* Ping block */
        [REG_MR_AI   ]      = 0x02,     /* Address Auto-Increment */
        [REG_MR_IND  ]      = 0x01,     /* Indirect mode */

    [REG_SHAR        ]      = 0x0009,   /* Source MAC address */
    [REG_IR          ]      = 0x0015,   /* Interrupt Register */
        [REG_IR_S0   ]      = 0x01,     /* Socket 0 interrupt bit */
    
    [REG_IMR         ]      = 0x0016,   /* Interrupt Mask Register */
    [REG_RTR         ]      = 0x0017,   /* Retry Time-value Register */
        [REG_RTR_DEFAULT]   = 0x07d0,   /* Default retry time (2000) */
    
    [REG_RMSR        ]      = 0x001a,   /* RX Memory Size Register - ⚠️ W5200 uses different memory config */
    [REG_TMSR        ]      = 0x001b,   /* TX Memory Size Register - ⚠️ W5200 uses different memory config */
    
    /* Socket Registers (relative offsets from socket base) */
    [REG_S0_REGS     ]      = 0x4000,   /* Socket 0 register base - ⚠️ Different from W5100 */
    [REG_S0_REGS_LEN ]      = 0x0100,   /* Socket register length - ⚠️ Larger than W5100 */
    
    [REG_Sn_MR       ]      = 0x0000,   /* Socket n Mode Register (relative) */
        [REG_S0_MR_MACRAW]  = 0x04,     /* MAC RAW mode */
        [REG_S0_MR_MF    ]  = 0x40,     /* MAC Filter for W5200 */
    
    [REG_Sn_CR       ]      = 0x0001,   /* Socket n Command Register (relative) */
        [REG_S0_CR_OPEN  ]  = 0x01,     /* OPEN command */
        [REG_S0_CR_CLOSE ]  = 0x10,     /* CLOSE command */
        [REG_S0_CR_SEND  ]  = 0x20,     /* SEND command */
        [REG_S0_CR_RECV  ]  = 0x40,     /* RECV command */
    
    [REG_Sn_IR       ]      = 0x0002,   /* Socket n Interrupt Register (relative) */
    [REG_set_Sn_IR   ]      = 0x0002,   /* Same as Sn_IR for W5200 */
        [REG_S0_IR_SENDOK]  = 0x10,     /* SEND_OK interrupt */
        [REG_S0_IR_RECV  ]  = 0x04,     /* RECV interrupt */
    
    [REG_Sn_SR       ]      = 0x0003,   /* Socket n Status Register (relative) */
        [REG_S0_SR_MACRAW]  = 0x42,     /* MACRAW status */
    
    [REG_Sn_TX_FSR   ]      = 0x0020,   /* Socket n TX Free Size Register (relative) */
    [REG_Sn_TX_RD    ]      = 0x0022,   /* Socket n TX Read Pointer Register (relative) */
    [REG_Sn_TX_WR    ]      = 0x0024,   /* Socket n TX Write Pointer Register (relative) */
    [REG_Sn_RX_RSR   ]      = 0x0026,   /* Socket n RX Received Size Register (relative) */
    [REG_Sn_RX_RD    ]      = 0x0028,   /* Socket n RX Read Pointer Register (relative) */
    
    /* Memory Map - ⚠️ W5200 has different memory layout */
    [REG_TX_MEM_START]      = 0x8000,   /* TX memory start address */
    [REG_TX_MEM_SIZE ]      = 0x4000,   /* TX memory size (16KB) */
    [REG_RX_MEM_START]      = 0xC000,   /* RX memory start address */
    [REG_RX_MEM_SIZE ]      = 0x4000,   /* RX memory size (16KB) */
    
    /* Not used in W5200 */
    [REG_SIMR        ]      = 0x0000,   /* Not used in W5200 (for W5500) */
    [REG_Sn_RX_BSR   ]      = 0x401e,   /* Not used in W5200 */
    [REG_Sn_TX_BSR   ]      = 0x401f,   /* Not used in W5200 */
    [REG_PHYCFG      ]      = 0x0000,   /* Not used in W5200 */
    [REG_PHYSTATE    ]      = 0x0000,   /* Not used in W5200 */
    [REG_PHYCFG_AUTONEG]    = 0x0000,   /* Not used in W5200 */
};

const u32 wiz5500_map[NUM_REGS] = {
    /* Common Registers (0x0000 ~ 0x003F) */
    [REG_COMMON_BASE ]      = 0x0000,
    [REG_COMMON_REGS ]      = 0x0000,
    [REG_COMMON_REGS_LEN]   = 0x003F,   /* Common register length */
    
    [REG_MR          ]      = 0x0000,   /* Mode Register */
        [REG_MR_RST  ]      = 0x80,     /* S/W reset */
        [REG_MR_PB   ]      = 0x10,     /* Ping block */
        [REG_MR_AI   ]      = 0x02,     /* Address Auto-Increment - not used in SPI */
        [REG_MR_IND  ]      = 0x01,     /* Indirect mode - not used in SPI */

    [REG_SHAR        ]      = 0x0009,   /* Source MAC address */
    [REG_IR          ]      = 0x0015,   /* Interrupt Register */
        [REG_IR_S0   ]      = 0x01,     /* Socket 0 interrupt bit */
    
    [REG_IMR         ]      = 0x0016,   /* Interrupt Mask Register */
    [REG_SIMR        ]      = 0x0018,   /* Socket Interrupt Mask Register - W5500 specific */
    [REG_RTR         ]      = 0x0019,   /* Retry Time-value Register */
        [REG_RTR_DEFAULT]   = 0x07d0,   /* Default retry time (2000) */
    
    [REG_RMSR        ]      = 0x0000,   /* Not used in W5500 - individual socket memory config */
    [REG_TMSR        ]      = 0x0000,   /* Not used in W5500 - individual socket memory config */
    
    /* Socket Registers (W5500 uses different addressing with BSB) */
    [REG_S0_REGS     ]      = 0x0000,   /* Socket 0 register base (BSB=0x08) */
    [REG_S0_REGS_LEN ]      = 0x0030,   /* Socket register length */
    
    [REG_Sn_MR       ]      = 0x0000,   /* Socket n Mode Register (relative) */
        [REG_S0_MR_MACRAW]  = 0x04,     /* MAC RAW mode */
        [REG_S0_MR_MF    ]  = 0x80,     /* MAC Filter for W5500 - different bit! */
    
    [REG_Sn_CR       ]      = 0x0001,   /* Socket n Command Register (relative) */
        [REG_S0_CR_OPEN  ]  = 0x01,     /* OPEN command */
        [REG_S0_CR_CLOSE ]  = 0x10,     /* CLOSE command */
        [REG_S0_CR_SEND  ]  = 0x20,     /* SEND command */
        [REG_S0_CR_RECV  ]  = 0x40,     /* RECV command */
    
    [REG_Sn_IR       ]      = 0x0002,   /* Socket n Interrupt Register (relative) */
    [REG_set_Sn_IR   ]      = 0x0002,   /* Same as Sn_IR for W5500 */
        [REG_S0_IR_SENDOK]  = 0x10,     /* SEND_OK interrupt */
        [REG_S0_IR_RECV  ]  = 0x04,     /* RECV interrupt */
    
    [REG_Sn_SR       ]      = 0x0003,   /* Socket n Status Register (relative) */
        [REG_S0_SR_MACRAW]  = 0x42,     /* MACRAW status */
    
    [REG_Sn_TX_FSR   ]      = 0x0020,   /* Socket n TX Free Size Register (relative) */
    [REG_Sn_TX_RD    ]      = 0x0022,   /* Socket n TX Read Pointer Register (relative) */
    [REG_Sn_TX_WR    ]      = 0x0024,   /* Socket n TX Write Pointer Register (relative) */
    [REG_Sn_RX_RSR   ]      = 0x0026,   /* Socket n RX Received Size Register (relative) */
    [REG_Sn_RX_RD    ]      = 0x0028,   /* Socket n RX Read Pointer Register (relative) */
    
    /* Memory Map - W5500 uses per-socket memory with BSB */
    [REG_TX_MEM_START]      = 0x20000,   /* TX memory start (BSB-based) */
    [REG_TX_MEM_SIZE ]      = 0x04000,   /* TX memory size per socket (2KB default) */
    [REG_RX_MEM_START]      = 0x30000,   /* RX memory start (BSB-based) */
    [REG_RX_MEM_SIZE ]      = 0x04000,   /* RX memory size per socket (2KB default) */
    
    /* W5500 specific registers */
    [REG_Sn_RX_BSR   ]      = 0x1001e,   /* Not used in W5500 (W6100 specific) */
    [REG_Sn_TX_BSR   ]      = 0x1001f,   /* Not used in W5500 (W6100 specific) */
    [REG_PHYCFG      ]      = 0x0000,   /* Not used in W5500 */
    [REG_PHYSTATE    ]      = 0x0000,   /* Not used in W5500 */
    [REG_PHYCFG_AUTONEG]    = 0x0000,   /* Not used in W5500 */
};

const u32 wiz6100_map[NUM_REGS] = {
    /* Common Registers - W6100 has expanded register map */
    [REG_COMMON_BASE ]      = 0x0000,
    [REG_COMMON_REGS ]      = 0x0000,
    [REG_COMMON_REGS_LEN]   = 0x0100,   /* W6100 has larger common register space */
    
    [REG_MR          ]      = 0x0000,   /* Mode Register */
        [REG_MR_RST  ]      = 0x80,     /* S/W reset */
        [REG_MR_PB   ]      = 0x10,     /* Ping block */
        [REG_MR_AI   ]      = 0x02,     /* Address Auto-Increment */
        [REG_MR_IND  ]      = 0x01,     /* Indirect mode */

    [REG_SHAR        ]      = 0x4120,   /* Source MAC address - ⚠️ W6100 다른 주소 */
    [REG_IR          ]      = 0x2100,   /* Interrupt Register - */
        [REG_IR_S0   ]      = 0x01,     /* Socket 0 interrupt bit */
    
    [REG_IMR         ]      = 0x2104,   /* Interrupt Mask Register - */
    [REG_SIMR        ]      = 0x2114,   /* Socket Interrupt Mask Register (W6100 global IMR) */
    [REG_RTR         ]      = 0x4200,   /* Retry Time-value Register (W6100 address) */
        [REG_RTR_DEFAULT]   = 0x07d0,   /* Default retry time (2000) */
    
    [REG_RMSR        ]      = 0x0000,   /* Not used in W6100 - different memory config */
    [REG_TMSR        ]      = 0x0000,   /* Not used in W6100 - different memory config */
    
    /* Socket Registers - W6100 has different socket register layout */
    [REG_S0_REGS     ]      = 0x080000,   /* Socket 0 register base - ⚠️ W6100 확인 필요 */
    [REG_S0_REGS_LEN ]      = 0x0100,   /* Socket register length - ⚠️ 확인 필요 */
    
    [REG_Sn_MR       ]      = 0x0000,   /* Socket n Mode Register (relative) */
        [REG_S0_MR_MACRAW]  = 0x07,     /* MAC RAW mode - ⚠️ W6100 값 확인 필요 */
        [REG_S0_MR_MF    ]  = 0x40,     /* MAC Filter */
    
    [REG_Sn_CR       ]      = 0x0010,   /* Socket n Command Register - ⚠️ W6100 다른 오프셋 */
        [REG_S0_CR_OPEN  ]  = 0x01,     /* OPEN command */
        [REG_S0_CR_CLOSE ]  = 0x10,     /* CLOSE command */
        [REG_S0_CR_SEND  ]  = 0x20,     /* SEND command */
        [REG_S0_CR_RECV  ]  = 0x40,     /* RECV command */
    
    [REG_Sn_IR       ]      = 0x0020,   /* Socket n Interrupt Register - ⚠️ W6100 다른 오프셋 */
    [REG_set_Sn_IR   ]      = 0x0028,   /* Socket n Interrupt Clear Register - ⚠️ W6100 확인 필요 */
        [REG_S0_IR_SENDOK]  = 0x10,     /* SEND_OK interrupt */
        [REG_S0_IR_RECV  ]  = 0x04,     /* RECV interrupt */
    
    [REG_Sn_SR       ]      = 0x0030,   /* Socket n Status Register - ⚠️ W6100 다른 오프셋 */
        [REG_S0_SR_MACRAW]  = 0x42,     /* MACRAW status */
    
    [REG_Sn_TX_FSR   ]      = 0x0204,   /* Socket n TX Free Size - ⚠️ W6100 확인 필요 */
    [REG_Sn_TX_RD    ]      = 0x0208,   /* Socket n TX Read Pointer - ⚠️ W6100 확인 필요 */
    [REG_Sn_TX_WR    ]      = 0x020c,   /* Socket n TX Write Pointer - ⚠️ W6100 확인 필요 */
    [REG_Sn_RX_RSR   ]      = 0x0224,   /* Socket n RX Received Size - ⚠️ W6100 확인 필요 */
    [REG_Sn_RX_RD    ]      = 0x0228,   /* Socket n RX Read Pointer - ⚠️ W6100 확인 필요 */
    

    
    /* W6100 specific registers */
    [REG_Sn_RX_BSR   ]      = 0x0220,   /* Socket RX BSR register */
    [REG_Sn_TX_BSR   ]      = 0x0200,   /* Socket TX BSR register */
    
    [REG_PHYCFG      ]      = 0x2E00,   /* PHY Configuration register */
    [REG_PHYSTATE    ]      = 0x2E00,   /* PHY State register (same as PHYCFG) */
    [REG_PHYCFG_AUTONEG]    = 0xC8,     /* PHY Auto-negotiation value */

    /* Memory Map - ⚠️ W6100 메모리 맵 전체 확인 필요 */
    [REG_TX_MEM_START]      = 0x100000,  /* ⚠️ 확인 필요 */
    [REG_TX_MEM_SIZE ]      = 0x4000,   /* ⚠️ 확인 필요 */
    [REG_RX_MEM_START]      = 0x180000,  /* ⚠️ 확인 필요 */
    [REG_RX_MEM_SIZE ]      = 0x4000,   /* ⚠️ 확인 필요 */
};

const u32 wiz6300_map[NUM_REGS] = {
    /* ⚠️ W6300 데이터시트 전체 확인 필요 - W6100과 유사할 것으로 예상 */
    /* Common Registers */
    [REG_COMMON_BASE ]      = 0x0000,   /* ⚠️ 확인 필요 */
    [REG_COMMON_REGS ]      = 0x0000,   /* ⚠️ 확인 필요 */
    [REG_COMMON_REGS_LEN]   = 0x0100,   /* ⚠️ 확인 필요 */
    
    [REG_MR          ]      = 0x0000,   /* Mode Register - ⚠️ 확인 필요 */
        [REG_MR_RST  ]      = 0x80,     /* S/W reset */
        [REG_MR_PB   ]      = 0x10,     /* Ping block */
        [REG_MR_AI   ]      = 0x02,     /* Address Auto-Increment */
        [REG_MR_IND  ]      = 0x01,     /* Indirect mode */

    [REG_SHAR        ]      = 0x4120,   /* Source MAC address -  */
    [REG_IR          ]      = 0x2100,   /* Interrupt Register -  */
        [REG_IR_S0   ]      = 0x01,     /* Socket 0 interrupt bit */
    
    [REG_IMR         ]      = 0x2104,   /* Interrupt Mask Register - ⚠️ 확인 필요 */
    [REG_SIMR        ]      = 0x2114,   /* Socket Interrupt Mask Register (W6300 global IMR) */
    [REG_RTR         ]      = 0x4200,   /* Retry Time-value Register (W6300 address) */
        [REG_RTR_DEFAULT]   = 0x07d0,   /* Default retry time (2000) */
    
    [REG_RMSR        ]      = 0x0000,   /* ⚠️ W6300 메모리 구성 확인 필요 */
    [REG_TMSR        ]      = 0x0000,   /* ⚠️ W6300 메모리 구성 확인 필요 */
    
    /* Socket Registers - ⚠️ W6300 소켓 레지스터 맵 전체 확인 필요 */
    [REG_S0_REGS     ]      = 0x0800,   /* ⚠️ 확인 필요 */
    [REG_S0_REGS_LEN ]      = 0x0100,   /* ⚠️ 확인 필요 */
    
    [REG_Sn_MR       ]      = 0x0000,   /* ⚠️ 확인 필요 */
        [REG_S0_MR_MACRAW]  = 0x07,     /* ⚠️ 확인 필요 */
        [REG_S0_MR_MF    ]  = 0x40,     /* ⚠️ 확인 필요 */
    
    [REG_Sn_CR       ]      = 0x0010,   /* ⚠️ 확인 필요 */
        [REG_S0_CR_OPEN  ]  = 0x01,     /* OPEN command */
        [REG_S0_CR_CLOSE ]  = 0x10,     /* CLOSE command */
        [REG_S0_CR_SEND  ]  = 0x20,     /* SEND command */
        [REG_S0_CR_RECV  ]  = 0x40,     /* RECV command */
    
    [REG_Sn_IR]             = 0x0020,   /* ⚠️ 확인 필요 */
    [REG_set_Sn_IR]         = 0x0028,   /* ⚠️ 확인 필요 */
        [REG_S0_IR_SENDOK]  = 0x10,     /* SEND_OK interrupt */
        [REG_S0_IR_RECV]  = 0x04,     /* RECV interrupt */
    
    [REG_Sn_SR]      = 0x0030,   /* ⚠️ 확인 필요 */
        [REG_S0_SR_MACRAW]  = 0x42,     /* MACRAW status */
    
    [REG_Sn_TX_FSR   ]      = 0x0204,   /* ⚠️ 확인 필요 */
    [REG_Sn_TX_RD    ]      = 0x0208,   /* ⚠️ 확인 필요 */
    [REG_Sn_TX_WR    ]      = 0x020c,   /* ⚠️ 확인 필요 */
    [REG_Sn_RX_RSR   ]      = 0x0224,   /* ⚠️ 확인 필요 */
    [REG_Sn_RX_RD    ]      = 0x0228,   /* ⚠️ 확인 필요 */
    
    /* Memory Map - ⚠️ W6300 메모리 맵 전체 확인 필요 */
    [REG_TX_MEM_START]      = 0x20000,  /* ⚠️ 확인 필요 */
    [REG_TX_MEM_SIZE ]      = 0x8000,   /* ⚠️ 확인 필요 - W6300은 더 클 수 있음 */
    [REG_RX_MEM_START]      = 0x30000,  /* ⚠️ 확인 필요 */
    [REG_RX_MEM_SIZE ]      = 0x8000,   /* ⚠️ 확인 필요 - W6300은 더 클 수 있음 */
    
    /* W6300 specific registers (assumed same as W6100) */
    [REG_Sn_RX_BSR   ]      = 0x0220,   /* ⚠️ 확인 필요 - W6100과 동일한지 확인 */
    [REG_Sn_TX_BSR   ]      = 0x0200,   /* ⚠️ 확인 필요 - W6100과 동일한지 확인 */
    [REG_PHYCFG      ]      = 0x2E00,   /* ⚠️ 확인 필요 - W6100과 동일한지 확인 */
    [REG_PHYSTATE    ]      = 0x2E00,   /* ⚠️ 확인 필요 - W6100과 동일한지 확인 */
    [REG_PHYCFG_AUTONEG]    = 0xC8,     /* ⚠️ 확인 필요 - W6100과 동일한지 확인 */
};