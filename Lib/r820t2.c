/* vim: set ai et ts=4 sw=4: */
/*
 * R820T2 downconverter driver by Eric Brombaugh, 2017
 * Ported to HAL and refactored by Aleksander Alekseev, 2018
 */

#include "r820t2.h"

#define R820T2_I2C_ADDRESS   ((0x1A)<<1)

/*
 * Freq calcs
 */
#define XTAL_FREQ           28800000
#define IF_FREQ             5000000
#define CALIBRATION_LO      88000

#define R820T2_WRITE_START  5

/* initial values from airspy */
/* initial freq @ 128MHz -> ~5MHz IF due to xtal mismatch */
static const uint8_t regs_init_array[R820T2_NUM_REGS] = {
    0x00, 0x00, 0x00, 0x00,    0x00,        /* 00 to 04 */
    /* 05 */ 0x90, // LNA manual gain mode, init to 0
    /* 06 */ 0x80,
    /* 07 */ 0x60,
    /* 08 */ 0x80, // Image Gain Adjustment
    /* 09 */ 0x40, // Image Phase Adjustment
    /* 0A */ 0xA8, // Channel filter [0..3]: 0 = widest, f = narrowest - Optimal. Don't touch!
    /* 0B */ 0x0F, // High pass filter - Optimal. Don't touch!
    /* 0C */ 0x40, // VGA control by code, init at 0
    /* 0D */ 0x63, // LNA AGC settings: [0..3]: Lower threshold; [4..7]: High threshold
    /* 0E */ 0x75,
    /* 0F */ 0xF8, // Filter Widest, LDO_5V OFF, clk out OFF,
    /* 10 */ 0x7C,
    /* 11 */ 0x83,
    /* 12 */ 0x80,
    /* 13 */ 0x00,
    /* 14 */ 0x0F,
    /* 15 */ 0x00,
    /* 16 */ 0xC0,
    /* 17 */ 0x30,
    /* 18 */ 0x48,
    /* 19 */ 0xCC,
    /* 1A */ 0x60,
    /* 1B */ 0x00,
    /* 1C */ 0x54,
    /* 1D */ 0xAE,
    /* 1E */ 0x0A,
    /* 1F */ 0xC0
};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/*
 * Tuner frequency ranges
 * Kanged & modified from airspy firmware to include freq for scanning table
 * "Copyright (C) 2013 Mauro Carvalho Chehab"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 */
struct r820t2_freq_range {
    uint16_t freq;
    uint8_t open_d;
    uint8_t rf_mux_ploy;
    uint8_t tf_c;
};

const struct r820t2_freq_range freq_ranges[] = {
  {
  /* 0 MHz */              0,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0xDF, /* R27[7:0]  band2,band0 */
  }, {
  /* 50 MHz */            50,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0xBE, /* R27[7:0]  band4,band1  */
  }, {
  /* 55 MHz */            55,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x8B, /* R27[7:0]  band7,band4 */
  }, {
  /* 60 MHz */            60,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x7B, /* R27[7:0]  band8,band4 */
  }, {
  /* 65 MHz */            65,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x69, /* R27[7:0]  band9,band6 */
  }, {
  /* 70 MHz */            70,
  /* .open_d = */       0x08, /* low */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x58, /* R27[7:0]  band10,band7 */
  }, {
  /* 75 MHz */            75,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x44, /* R27[7:0]  band11,band11 */
  }, {
  /* 80 MHz */            80,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x44, /* R27[7:0]  band11,band11 */
  }, {
  /* 90 MHz */            90,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x34, /* R27[7:0]  band12,band11 */
  }, {
  /* 100 MHz */          100,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x34, /* R27[7:0]  band12,band11 */
  }, {
  /* 110 MHz */          110,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x24, /* R27[7:0]  band13,band11 */
  }, {
  /* 120 MHz */          120,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x24, /* R27[7:0]  band13,band11 */
  }, {
  /* 140 MHz */          140,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x14, /* R27[7:0]  band14,band11 */
  }, {
  /* 180 MHz */          180,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x13, /* R27[7:0]  band14,band12 */
  }, {
  /* 220 MHz */          220,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x13, /* R27[7:0]  band14,band12 */
  }, {
  /* 250 MHz */          250,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x11, /* R27[7:0]  highest,highest */
  }, {
  /* 280 MHz */          280,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x02, /* R26[7:6]=0 (LPF)  R26[1:0]=2 (low) */
  /* .tf_c = */         0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 310 MHz */          310,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x41, /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
  /* .tf_c = */         0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 450 MHz */          450,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x41, /* R26[7:6]=1 (bypass)  R26[1:0]=1 (middle) */
  /* .tf_c = */         0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 588 MHz */          588,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x40, /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
  /* .tf_c = */         0x00, /* R27[7:0]  highest,highest */
  }, {
  /* 650 MHz */          650,
  /* .open_d = */       0x00, /* high */
  /* .rf_mux_ploy = */  0x40, /* R26[7:6]=1 (bypass)  R26[1:0]=0 (highest) */
  /* .tf_c = */         0x00, /* R27[7:0]  highest,highest */
  }
};

// Cached register values are used in R820T2_write_cache_mask
uint8_t regs_cache[R820T2_NUM_REGS];

// Note: apparently R820T2 allows to read only from address 0x00
void R820T2_read(uint8_t addr, uint8_t *data, uint8_t num) {
    static const uint8_t reverse_table[] = {
        0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
        0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF
    };

    HAL_I2C_Mem_Read(&R820T2_I2C_PORT, R820T2_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT,
        data, num, HAL_MAX_DELAY);

    // During reading bits are transfered in reverse order,
    // thus we have to restore original bit order.
    for(uint16_t i = 0; i < num; i++) {
        uint8_t val = data[i];
        data[i] = (reverse_table[val & 0xF] << 4) | reverse_table[val >> 4];
    }
}

void R820T2_write(uint8_t addr, uint8_t *data, uint8_t num) {
    HAL_I2C_Mem_Write(&R820T2_I2C_PORT, R820T2_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT,
        data, num, HAL_MAX_DELAY);
}

uint8_t R820T2_read_reg(uint8_t n) {
    if(n >= R820T2_NUM_REGS)
        return 0;

    uint8_t regs[R820T2_NUM_REGS];
    R820T2_read(0x00, regs, sizeof(regs));
    return regs[n];
}

void R820T2_write_reg(uint8_t n, uint8_t data) {
    if(n >= R820T2_NUM_REGS)
        return;
    regs_cache[n] = data;
    R820T2_write(n, &regs_cache[n], 1);
}

/*
 * Write single R820T2 reg via I2C with mask vs cached
 */
void R820T2_write_cache_mask(uint8_t reg, uint8_t data, uint8_t mask) {
    /* check for legal reg */
    if(reg>=R820T2_NUM_REGS)
        return;
    
    /* mask vs cached reg */
    data = (data & mask) | (regs_cache[reg] & ~mask);
    regs_cache[reg] = data;
    
    /* send via I2C */
    R820T2_write(reg, &regs_cache[reg], 1);
}

/*
 * Update Tracking Filter
 * Kanged & Modified from airspy firmware
 * 
 * "inspired by Mauro Carvalho Chehab set_mux technique"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 * part of r820t_set_mux() (set tracking filter)
 */
static void R820T2_set_tf(uint32_t freq) {
    const struct r820t2_freq_range *range;
    unsigned int i;

    /* Get Freq in MHz */
    freq = (uint32_t)((uint64_t)freq * 4295 >> 32); // fast approach
    
    /* Scan array for proper range */
    for(i = 0; i < ARRAY_SIZE(freq_ranges)-1; i++) {
        if(freq < freq_ranges[i + 1].freq)
            break;
    }
    range = &freq_ranges[i];

    /* Open Drain */
    R820T2_write_cache_mask(0x17, range->open_d, 0x08);

    /* RF_MUX,Polymux */
    R820T2_write_cache_mask(0x1A, range->rf_mux_ploy, 0xC3);

    /* TF BAND */
    R820T2_write_reg(0x1B, range->tf_c);

    /* XTAL CAP & Drive */
    R820T2_write_cache_mask(0x10, 0x08, 0x0b);

    R820T2_write_cache_mask(0x08, 0x00, 0x3F);
 
    R820T2_write_cache_mask(0x09, 0x00, 0x3F);
}

/*
 * Update LO PLL
 */
void R820T2_set_pll(uint32_t freq) {
    const uint32_t vco_min = 1770000000;
    const uint32_t vco_max = 3900000000;
    uint32_t pll_ref = (XTAL_FREQ >> 1);
    uint32_t pll_ref_2x = XTAL_FREQ;

    uint32_t vco_exact;
    uint32_t vco_frac;
    uint32_t con_frac;
    uint32_t div_num;
    uint32_t n_sdm;
    uint16_t sdm;
    uint8_t ni;
    uint8_t si;
    uint8_t nint;

    /* Calculate VCO output divider */
    for(div_num = 0; div_num < 5; div_num++) {
        vco_exact = freq << (div_num + 1);
        if(vco_exact >= vco_min && vco_exact <= vco_max) {
            break;
        }
    }

    /* Calculate the integer PLL feedback divider */
    vco_exact = freq << (div_num + 1);
    nint = (uint8_t) ((vco_exact + (pll_ref >> 16)) / pll_ref_2x);
    vco_frac = vco_exact - pll_ref_2x * nint;

    nint -= 13;
    ni = (nint >> 2);
    si = nint - (ni << 2);

    /* Set the vco output divider */
    R820T2_write_cache_mask(0x10, (uint8_t) (div_num << 5), 0xE0);

    /* Set the PLL Feedback integer divider */
    R820T2_write_reg(0x14, (uint8_t) (ni + (si << 6)));

    /* Update Fractional PLL */
    if(vco_frac == 0) {
        /* Disable frac pll */
        R820T2_write_cache_mask(0x12, 0x08, 0x08);
    } else {
        /* Compute the Sigma-Delta Modulator */
        vco_frac += pll_ref >> 16;
        sdm = 0;
        for(n_sdm = 0; n_sdm < 16; n_sdm++) {
            con_frac = pll_ref >> n_sdm;
            if (vco_frac >= con_frac) {
                sdm |= (uint16_t) (0x8000 >> n_sdm);
                vco_frac -= con_frac;
                if (vco_frac == 0)
                    break;
            }
        }
       
        /* Update Sigma-Delta Modulator */
        R820T2_write_reg(0x15, (uint8_t)(sdm & 0xFF));
        R820T2_write_reg(0x16, (uint8_t)(sdm >> 8));

        /* Enable frac pll */
        R820T2_write_cache_mask(0x12, 0x00, 0x08);
    }
}

/*
 * Update Tracking Filter and LO to frequency
 */
void R820T2_set_frequency(uint32_t freq) {
    R820T2_set_tf(freq);
    R820T2_set_pll(freq + IF_FREQ);
}

/*
 * Set IF Bandwidth [0-15]
 */
void R820T2_set_bandwidth(uint8_t bw) {
    const uint8_t modes[] = { 0xE0, 0x80, 0x60, 0x00 };
    uint8_t a = 0xB0 | (0x0F-(bw & 0x0F));
    uint8_t b = 0x0F | modes[(bw & 0x3) >> 4];
    R820T2_write_reg(0x0A, a);
    R820T2_write_reg(0x0B, b);
}

/*
 * Set gain of LNA [0-15]
 */
void R820T2_set_lna_gain(uint8_t gain_index) {
    R820T2_write_cache_mask(0x05, gain_index, 0x0F);
}

/*
 * Set gain of mixer [0-15]
 */
void R820T2_set_mixer_gain(uint8_t gain_index) {
    R820T2_write_cache_mask(0x07, gain_index, 0x0F);
}

/*
 * Set gain of VGA [0-15]
 */
void R820T2_set_vga_gain(uint8_t gain_index) {
    R820T2_write_cache_mask(0x0C, gain_index, 0x0F);
}

/*
 * Enable/Disable LNA AGC [0 / 1]
 */
void R820T2_set_lna_agc(uint8_t value) {
    value = value != 0 ? 0x00 : 0x10;
    R820T2_write_cache_mask(0x05, value, 0x10);
}

/*
 * Enable/Disable Mixer AGC [0 / 1]
 */
void R820T2_set_mixer_agc(uint8_t value) {
    value = value != 0 ? 0x10 : 0x00;
    R820T2_write_cache_mask(0x07, value, 0x10);
}

/*
 * Calibrate 
 * Kanged from airspy firmware
 * "inspired by Mauro Carvalho Chehab calibration technique"
 * https://stuff.mit.edu/afs/sipb/contrib/linux/drivers/media/tuners/r820t.c
 */
int32_t R820T2_calibrate() {
    int32_t i, cal_code;

    for(i = 0; i < 5; i++) {
        /* Set filt_cap */
        R820T2_write_cache_mask(0x0B, 0x08, 0x60);

        /* set cali clk =on */
        R820T2_write_cache_mask(0x0F, 0x04, 0x04);

        /* X'tal cap 0pF for PLL */
        R820T2_write_cache_mask(0x10, 0x00, 0x03);

        /* freq used for calibration */
        R820T2_set_pll(CALIBRATION_LO * 1000);

        /* Start Trigger */
        R820T2_write_cache_mask(0x0B, 0x10, 0x10);

        HAL_Delay(2);

        /* Stop Trigger */
        R820T2_write_cache_mask(0x0B, 0x00, 0x10);

        /* set cali clk =off */
        R820T2_write_cache_mask(0x0F, 0x00, 0x04);

        /* Check if calibration worked */
        cal_code = R820T2_read_reg(0x04) & 0x0F;
        if(cal_code && cal_code != 0x0F)
            return 0;
    }

    /* cal failed */
    return -1;
}

/*
 * Initialize the R820T2
 */
void R820T2_init() {
    for(uint8_t i = R820T2_WRITE_START; i < R820T2_NUM_REGS; i++) {
        // R820T2_write_reg is used instead of R820T2_write
        // to initialize regs_cache properly
        R820T2_write_reg(i, regs_init_array[i]);
    }
}

